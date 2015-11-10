#include <appSec.h>
#include <string.h>
#include <stdio.h>
#include <heap.h>
#include <sha2.h>
#include <rsa.h>
#include <aes.h>


#define APP_HDR_SIZE                32                                    //headers are this size
#define APP_DATA_CHUNK_SIZE         (AES_BLOCK_WORDS * sizeof(uint32_t))  //data blocks are this size
#define APP_SIG_SIZE                RSA_BYTES

#define RSA_WORDS                   (RSA_BYTES / sizeof(uint32_t))

#define STATE_INIT                  0 //nothing gotten yet
#define STATE_RXING_HEADERS         1 //each is APP_HDR_SIZE bytes
#define STATE_RXING_DATA            2 //each data block is AES_BLOCK_WORDS 32-bit words (for AES reasons)
#define STATE_RXING_SIG_HASH        3 //each is RSA_BYTES bytes
#define STATE_RXING_SIG_PUBKEY      4 //each is RSA_BYTES bytes
#define STATE_DONE                  5 //all is finished and well
#define STATE_BAD                   6 //unrecoverable badness has happened. this will *NOT* fix itself. It is now ok to give up, start over, cry, or pray to your favourite deity for help

struct AppSecState {

    union { //we save some memory by reusing this space.
        struct {
            struct AesCbcContext cbc;
            struct Sha2state sha;
        };
        struct RsaState rsa;
    };
    uint32_t rsaTmp[RSA_WORDS];
    uint32_t lastHash[SHA2_HASH_WORDS];

    AppSecWriteCbk writeCbk;
    AppSecPubKeyFindCbk pubKeyFindCbk;
    AppSecGetAesKeyCbk aesKeyAccessCbk;

    union {
        union { //make the compiler work to make sure we have enough space
            uint8_t placeholderAppHdr[APP_HDR_SIZE];
            uint8_t placeholderDataChunk[APP_DATA_CHUNK_SIZE];
            uint8_t placeholderSigChunk[APP_SIG_SIZE];
            uint8_t placeholderAesKey[AES_KEY_WORDS * sizeof(uint32_t)];
        };
        uint8_t dataBytes[0]; //we actually use these two for access
        uint32_t dataWords[0];
    };

    uint32_t signedBytes;
    uint32_t encryptedBytes;
    uint32_t numSigs;

    uint8_t haveBytes;       //in dataBytes...
    uint8_t curState;
    uint8_t needSig    :1;
    uint8_t haveSig    :1;
    uint8_t haveEncr   :1;
};

struct AppSecSigHdr {
    uint8_t magic[8];
    uint32_t appDataLen;
    uint32_t numSigs;
};

struct AppSecEncrHdr {
    uint8_t magic[4];
    uint32_t dataLen;
    uint64_t keyID;
    uint32_t IV[AES_BLOCK_WORDS];
};

//init/deinit
struct AppSecState *appSecInit(AppSecWriteCbk writeCbk, AppSecPubKeyFindCbk pubKeyFindCbk, AppSecGetAesKeyCbk aesKeyAccessCbk, bool mandateSigning)
{
    struct AppSecState *state = heapAlloc(sizeof(struct AppSecState));

    if (!state)
        return NULL;

    state->writeCbk = writeCbk;
    state->pubKeyFindCbk = pubKeyFindCbk;
    state->aesKeyAccessCbk = aesKeyAccessCbk;
    state->haveBytes = 0;
    state->curState = STATE_INIT;
    state->signedBytes = 0;
    state->encryptedBytes = 0;
    state->needSig = mandateSigning ? 1 : 0;
    state->haveSig = 0;
    state->haveEncr = 0;

    return state;
}

void appSecDeinit(struct AppSecState *state)
{
    heapFree(state);
}

static AppSecErr appSecProcessIncomingHdr(struct AppSecState *state)
{
    static const char hdrAddEncrKey[] = "EncrKey+";
    static const char hdrDelEncrKey[] = "EncrKey+";
    static const char hdrNanoApp[] = "GoogleNanoApp\x00\xff\xff"; //we chack marker is set to 0xFF and version set to 0, as we must as per spec
    static const char hdrEncrHdr[] = "Encr";
    static const char hdrSigHdr[] = "SignedApp";

    //check for signature header
    if (!memcmp(state->dataBytes, hdrSigHdr, sizeof(hdrSigHdr) - 1)) {

        struct AppSecSigHdr *sigHdr = (struct AppSecSigHdr*)state->dataBytes;

        if (state->haveSig)    //we do not allow signing of already-signed data
            return APP_SEC_INVALID_DATA;

        if (state->haveEncr) //we do not allow encryption of signed data, only signing of encrypted data
            return APP_SEC_INVALID_DATA;

        if (!sigHdr->appDataLen || !sigHdr->numSigs) //no data bytes or no sigs?
            return APP_SEC_INVALID_DATA;

        state->signedBytes = sigHdr->appDataLen;
        state->numSigs = sigHdr->numSigs;
        state->haveSig = 1;
        sha2init(&state->sha);

        return APP_SEC_NO_ERROR;
    }

    //check for encryption header
    if (!memcmp(state->dataBytes, hdrEncrHdr, sizeof(hdrEncrHdr) - 1)) {

        struct AppSecEncrHdr *encrHdr = (struct AppSecEncrHdr*)state->dataBytes;
        AppSecErr ret;

        if (state->haveEncr) //we do not allow encryption of already-encrypted data
            return APP_SEC_INVALID_DATA;

        if (!encrHdr->dataLen || !encrHdr->keyID)
            return APP_SEC_INVALID_DATA;

        ret = state->aesKeyAccessCbk(encrHdr->keyID, state->dataBytes);
        if (ret)
            return ret;

        aesCbcInitForDecr(&state->cbc, state->dataWords, encrHdr->IV);
        state->encryptedBytes = encrHdr->dataLen;
        state->haveEncr = 1;

        //encr header is signed if signatures are on, so hash it
        if (state->haveSig) {
            if (APP_HDR_SIZE >= state->signedBytes)
                return APP_SEC_TOO_MUCH_DATA;
            state->signedBytes -= APP_HDR_SIZE;
            sha2processBytes(&state->sha, state->dataBytes, APP_HDR_SIZE);
        }

        return APP_SEC_NO_ERROR;
    }

    //check for valid app or sopmething else that we pass directly to caller
    if (memcmp(state->dataBytes, hdrAddEncrKey, sizeof(hdrAddEncrKey) - 1) && memcmp(state->dataBytes, hdrDelEncrKey, sizeof(hdrDelEncrKey) - 1) && memcmp(state->dataBytes, hdrNanoApp, sizeof(hdrNanoApp) - 1))
        return APP_SEC_HEADER_ERROR;

    //if we are in must-sign mode and no signature was provided, fail
    if (!state->haveSig && state->needSig)
        return APP_SEC_SIG_VERIFY_FAIL;

    //we're now in data-accepting state
    state->curState = STATE_RXING_DATA;

    //send data to caller as is
    return state->writeCbk(state->dataBytes, APP_HDR_SIZE);
}

static AppSecErr appSecProcessIncomingData(struct AppSecState *state, uint32_t len)
{
    //hash if sig-checking is on
    if (state->haveSig) {
        if (len >= state->signedBytes)
            return APP_SEC_TOO_MUCH_DATA;
        state->signedBytes -= len;
        sha2processBytes(&state->sha, state->dataBytes, len);
    }

    //decrypt if encryption is on
    if (state->haveEncr) {
        if (len != APP_DATA_CHUNK_SIZE)
            return APP_SEC_TOO_LITTLE_DATA;
        if (len >= state->encryptedBytes)
            return APP_SEC_TOO_MUCH_DATA;
        state->encryptedBytes -= len;
        aesCbcDecr(&state->cbc, state->dataWords, state->dataWords);
    }

    //check for data-ending conditions
    if (state->haveSig && !state->signedBytes) {      //we're all done with the signed portion of the data, now come the signatures
        if (state->haveEncr && state->encryptedBytes) //somehow we still have more "encrypted" bytes now - this is not valid
            return APP_SEC_INVALID_DATA;
        state->curState = STATE_RXING_SIG_HASH;

        //collect the hash
        memcpy(state->lastHash, sha2finish(&state->sha), SHA2_HASH_SIZE);
    }
    else if (state->haveEncr && !state->encryptedBytes) { //we're all done with encrypted bytes
        if (state->haveSig && state->signedBytes)         //somehow we still have more "signed" bytes now - this is not valid
            return APP_SEC_INVALID_DATA;
        state->curState = STATE_DONE;
    }

    //pass to caller
    return state->writeCbk(state->dataBytes, len);
}

static AppSecErr appSecProcessIncomingSigData(struct AppSecState *state)
{
    const uint32_t *result;
    uint32_t i;

    //if we're RXing the hash, just stash it away and move on
    if (state->curState == STATE_RXING_SIG_HASH) {
        if (!state->numSigs)
            return APP_SEC_TOO_MUCH_DATA;

        state->numSigs--;
        memcpy(state->rsaTmp, state->dataWords, APP_SIG_SIZE);
        state->curState = STATE_RXING_SIG_PUBKEY;
        return APP_SEC_NO_ERROR;
    }

    //if we just got the last sig, verify it is a known root
    if (!state->numSigs) {
        bool keyFound = false;
        AppSecErr ret;

        ret = state->pubKeyFindCbk(state->dataWords, &keyFound);
        if (ret != APP_SEC_NO_ERROR)
            return ret;
        if (!keyFound)
            return APP_SEC_SIG_ROOT_UNKNOWN;
    }

    //we now have the pubKey. decrypt.
    result = rsaPubOp(&state->rsa, state->rsaTmp, state->dataWords);

    //verify padding: all by first and last word of padding MUST have no zero bytes
    for (i = SHA2_HASH_WORDS + 1; i < RSA_WORDS - 1; i++) {
        if (!(uint8_t)(result[i] >>  0))
            return APP_SEC_SIG_DECODE_FAIL;
        if (!(uint8_t)(result[i] >>  8))
            return APP_SEC_SIG_DECODE_FAIL;
        if (!(uint8_t)(result[i] >> 16))
            return APP_SEC_SIG_DECODE_FAIL;
        if (!(uint8_t)(result[i] >> 24))
            return APP_SEC_SIG_DECODE_FAIL;
    }

    //verify padding: first padding word must have all nonzero bytes except low byte
    if ((result[SHA2_HASH_WORDS] & 0xff) || !(result[SHA2_HASH_WORDS] & 0xff00) || !(result[SHA2_HASH_WORDS] & 0xff0000) || !(result[SHA2_HASH_WORDS] & 0xff000000))
        return APP_SEC_SIG_DECODE_FAIL;

    //verify padding: last padding word must have 0x0002 in top 16 bits and nonzero random bytes in lower bytes
    if ((result[RSA_WORDS - 1] >> 16) != 2)
        return APP_SEC_SIG_DECODE_FAIL;
    if (!(result[RSA_WORDS - 1] & 0xff00) || !(result[RSA_WORDS - 1] & 0xff))
        return APP_SEC_SIG_DECODE_FAIL;

    //check if hashes match
    if (memcmp(state->lastHash, result, SHA2_HASH_SIZE))
        return APP_SEC_SIG_VERIFY_FAIL;

    //hash the provided pubkey if it is not the last
    if (state->numSigs) {
        sha2init(&state->sha);
        sha2processBytes(&state->sha, state->dataBytes, APP_SIG_SIZE);
        memcpy(state->lastHash, sha2finish(&state->sha), SHA2_HASH_SIZE);
        state->curState = STATE_RXING_SIG_HASH;
    }
    else
        state->curState = STATE_DONE;

    return APP_SEC_NO_ERROR;
}

AppSecErr appSecRxData(struct AppSecState *state, const void *dataP, uint32_t len)
{
    const uint8_t *data = (const uint8_t*)dataP;
    AppSecErr ret = APP_SEC_NO_ERROR;

    if (state->curState == STATE_INIT)
        state->curState = STATE_RXING_HEADERS;

    while (len--) {
        state->dataBytes[state->haveBytes++] = *data++;
        switch (state->curState) {

        case STATE_RXING_HEADERS:
            if (state->haveBytes == APP_HDR_SIZE) {
                ret = appSecProcessIncomingHdr(state);
                state->haveBytes = 0;
                if (ret != APP_SEC_NO_ERROR)
                    break;
            }
            break;

        case STATE_RXING_DATA:
            if (state->haveBytes == APP_DATA_CHUNK_SIZE) {
                ret = appSecProcessIncomingData(state, APP_DATA_CHUNK_SIZE);
                state->haveBytes = 0;
                if (ret != APP_SEC_NO_ERROR)
                    break;
            }
            break;

        case STATE_RXING_SIG_HASH:
        case STATE_RXING_SIG_PUBKEY:
            if (state->haveBytes == APP_SIG_SIZE) {
                ret = appSecProcessIncomingSigData(state);
                state->haveBytes = 0;
                if (ret != APP_SEC_NO_ERROR)
                    break;
            }
            break;

        default:
            state->curState = STATE_BAD;
            state->haveBytes = 0;
            return APP_SEC_BAD;
        }
    }

    if (ret != APP_SEC_NO_ERROR)
        state->curState = STATE_BAD;

    return ret;
}

AppSecErr appSecRxDataOver(struct AppSecState *state)
{
    AppSecErr ret;

    //Feed remianing data to data processor, if any
    if (state->haveBytes) {

        //not in data rx stage when the incoming data ends? This is not good
        if (state->curState != STATE_RXING_DATA) {
            state->curState = STATE_BAD;
            return APP_SEC_TOO_LITTLE_DATA;
        }

        //feed the remaining data to the data processor
        ret = appSecProcessIncomingData(state, state->haveBytes);
        if (ret != APP_SEC_NO_ERROR) {
            state->curState = STATE_BAD;
            return ret;
        }
    }

    //Check the state and return our verdict
    if(state->curState == STATE_DONE)
        return APP_SEC_NO_ERROR;

    state->curState = STATE_BAD;
    return APP_SEC_TOO_LITTLE_DATA;
}










