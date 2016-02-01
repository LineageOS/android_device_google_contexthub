/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

#define APP_SEC_SIG_ALIGN           APP_DATA_CHUNK_SIZE
#define APP_SEC_ENCR_ALIGN          APP_DATA_CHUNK_SIZE

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
        struct {
            struct RsaState rsa;
            uint32_t rsaState1, rsaState2, rsaStep;
        };
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

    uint32_t signedBytesIn;
    uint32_t encryptedBytesIn;
    uint32_t signedBytesOut;
    uint32_t encryptedBytesOut;
    uint32_t numSigs;

    uint16_t haveBytes;       //in dataBytes...
    uint8_t curState;
    uint8_t needSig    :1;
    uint8_t haveSig    :1;
    uint8_t haveEncr   :1;
    uint8_t doingRsa   :1;
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

    memset(state, 0, sizeof(struct AppSecState));

    state->writeCbk = writeCbk;
    state->pubKeyFindCbk = pubKeyFindCbk;
    state->aesKeyAccessCbk = aesKeyAccessCbk;
    state->curState = STATE_INIT;
    if (mandateSigning)
        state->needSig = 1;

    return state;
}

void appSecDeinit(struct AppSecState *state)
{
    heapFree(state);
}


//if needed, decrypt and hash incoming data
static AppSecErr appSecBlockRx(struct AppSecState *state)
{
    //if signatures are on, hash it
    if (state->haveSig) {

        //make sure we do not get too much data & account for the data we got
        if (state->haveBytes > state->signedBytesIn)
            return APP_SEC_TOO_MUCH_DATA;
        state->signedBytesIn -= state->haveBytes;

        //make sure we do not produce too much data (discard padding) & make sure we account for it
        if (state->signedBytesOut < state->haveBytes)
            state->haveBytes = state->signedBytesOut;
        state->signedBytesOut -= state->haveBytes;

        //hash the data
        sha2processBytes(&state->sha, state->dataBytes, state->haveBytes);
    }

    //decrypt if encryption is on
    if (state->haveEncr) {

        uint32_t *dataP = state->dataWords;
        uint32_t i, numBlocks = state->haveBytes / APP_DATA_CHUNK_SIZE;

        //we should not be called with partial encr blocks
        if (state->haveBytes % APP_DATA_CHUNK_SIZE)
            return APP_SEC_TOO_LITTLE_DATA;

        //make sure we do not get too much data & account for the data we got
        if (state->haveBytes > state->encryptedBytesIn)
            return APP_SEC_TOO_MUCH_DATA;
        state->encryptedBytesIn -= state->haveBytes;

        //decrypt
        for (i = 0; i < numBlocks; i++, dataP += AES_BLOCK_WORDS)
            aesCbcDecr(&state->cbc, dataP, dataP);

        //make sure we do not produce too much data (discard padding) & make sure we account for it
        if (state->encryptedBytesOut < state->haveBytes)
            state->haveBytes = state->encryptedBytesOut;
        state->encryptedBytesOut -= state->haveBytes;
    }

    return APP_SEC_NO_ERROR;
}

static AppSecErr appSecProcessIncomingHdr(struct AppSecState *state, bool *sendDataToDataHandlerP)
{
    static const char hdrAddEncrKey[] = "EncrKey+";
    static const char hdrDelEncrKey[] = "EncrKey+";
    static const char hdrNanoApp[] = "GoogleNanoApp\x00\xff\xff"; //we check marker is set to 0xFF and version set to 0, as we must as per spec
    static const char hdrEncrHdr[] = "Encr";
    static const char hdrSigHdr[] = "SigndApp";

    //check for signature header
    if (!memcmp(state->dataBytes, hdrSigHdr, sizeof(hdrSigHdr) - 1)) {

        struct AppSecSigHdr *sigHdr = (struct AppSecSigHdr*)state->dataBytes;

        if (state->haveSig)    //we do not allow signing of already-signed data
            return APP_SEC_INVALID_DATA;

        if (state->haveEncr) //we do not allow encryption of signed data, only signing of encrypted data
            return APP_SEC_INVALID_DATA;

        if (!sigHdr->appDataLen || !sigHdr->numSigs) //no data bytes or no sigs?
            return APP_SEC_INVALID_DATA;

        state->signedBytesOut = sigHdr->appDataLen;
        state->signedBytesIn = ((state->signedBytesOut + APP_SEC_SIG_ALIGN - 1) / APP_SEC_SIG_ALIGN) * APP_SEC_SIG_ALIGN;
        state->numSigs = sigHdr->numSigs;
        state->haveSig = 1;
        sha2init(&state->sha);

        return APP_SEC_NO_ERROR;
    }

    //check for encryption header
    if (!memcmp(state->dataBytes, hdrEncrHdr, sizeof(hdrEncrHdr) - 1)) {

        struct AppSecEncrHdr *encrHdr = (struct AppSecEncrHdr*)state->dataBytes;
	uint32_t k[AES_KEY_WORDS];
        AppSecErr ret;

        if (state->haveEncr) //we do not allow encryption of already-encrypted data
            return APP_SEC_INVALID_DATA;

        if (!encrHdr->dataLen || !encrHdr->keyID)
            return APP_SEC_INVALID_DATA;

        ret = state->aesKeyAccessCbk(encrHdr->keyID, k);
        if (ret)
            return ret;

        aesCbcInitForDecr(&state->cbc, k, encrHdr->IV);
        state->encryptedBytesOut = encrHdr->dataLen;
        state->encryptedBytesIn = ((state->encryptedBytesOut + APP_SEC_ENCR_ALIGN - 1) / APP_SEC_ENCR_ALIGN) * APP_SEC_ENCR_ALIGN;
        state->haveEncr = 1;

        return APP_SEC_NO_ERROR;
    }

    //check for valid app or something else that we pass directly to caller
    if (memcmp(state->dataBytes, hdrAddEncrKey, sizeof(hdrAddEncrKey) - 1) && memcmp(state->dataBytes, hdrDelEncrKey, sizeof(hdrDelEncrKey) - 1) && memcmp(state->dataBytes, hdrNanoApp, sizeof(hdrNanoApp) - 1))
        return APP_SEC_HEADER_ERROR;

    //if we are in must-sign mode and no signature was provided, fail
    if (!state->haveSig && state->needSig)
        return APP_SEC_SIG_VERIFY_FAIL;

    //we're now in data-accepting state
    state->curState = STATE_RXING_DATA;

    //send data to caller as is
    *sendDataToDataHandlerP = true;
    return APP_SEC_NO_ERROR;
}

static AppSecErr appSecProcessIncomingData(struct AppSecState *state)
{
    //check for data-ending conditions
    if (state->haveSig && !state->signedBytesIn) {      //we're all done with the signed portion of the data, now come the signatures
        if (state->haveEncr && state->encryptedBytesIn) //somehow we still have more "encrypted" bytes now - this is not valid
            return APP_SEC_INVALID_DATA;
        state->curState = STATE_RXING_SIG_HASH;

        //collect the hash
        memcpy(state->lastHash, sha2finish(&state->sha), SHA2_HASH_SIZE);
    }
    else if (state->haveEncr && !state->encryptedBytesIn) { //we're all done with encrypted bytes
        if (state->haveSig && state->signedBytesIn)           //somehow we still have more "signed" bytes now - this is not valid
            return APP_SEC_INVALID_DATA;
        state->curState = STATE_DONE;
    }

    //pass to caller
    return state->writeCbk(state->dataBytes, state->haveBytes);
}

AppSecErr appSecDoSomeProcessing(struct AppSecState *state)
{
    const uint32_t *result;

    if (!state->doingRsa) {
        //shouldn't be calling us then...
        return APP_SEC_BAD;
    }

    result = rsaPubOpIterative(&state->rsa, state->rsaTmp, state->dataWords, &state->rsaState1, &state->rsaState2, &state->rsaStep);
    if (state->rsaStep)
        return APP_SEC_NEED_MORE_TIME;

    //we just finished the RSA-ing
    state->doingRsa = 0;

    //verify signature padding (and thus likely: correct decryption)
    result = BL.blSigPaddingVerify(result);
    if (!result)
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

static AppSecErr appSecProcessIncomingSigData(struct AppSecState *state)
{
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

    //we now have the pubKey. decrypt over time
    state->doingRsa = 1;
    state->rsaStep = 0;
    return APP_SEC_NEED_MORE_TIME;
}

AppSecErr appSecRxData(struct AppSecState *state, const void *dataP, uint32_t len, uint32_t *lenUnusedP)
{
    const uint8_t *data = (const uint8_t*)dataP;
    AppSecErr ret = APP_SEC_NO_ERROR;
    bool sendToDataHandler = false;

    if (state->curState == STATE_INIT)
        state->curState = STATE_RXING_HEADERS;

    while (len--) {
        state->dataBytes[state->haveBytes++] = *data++;
        switch (state->curState) {

        case STATE_RXING_HEADERS:
            if (state->haveBytes == APP_HDR_SIZE) {

                ret = appSecBlockRx(state);
                if (ret != APP_SEC_NO_ERROR)
                    break;

                ret = appSecProcessIncomingHdr(state, &sendToDataHandler);
                if (ret != APP_SEC_NO_ERROR)
                    break;
                if (!sendToDataHandler) {
                    state->haveBytes = 0;
                    break;
                }
                //fallthrough
            }
            else
                break;

        case STATE_RXING_DATA:
            if (state->haveBytes >= APP_DATA_CHUNK_SIZE) {

                //if data is already processed, do not re-process it
                if (sendToDataHandler)
                    sendToDataHandler = false;
                else {
                    ret = appSecBlockRx(state);
                    if (ret != APP_SEC_NO_ERROR)
                        break;
                }

                ret = appSecProcessIncomingData(state);
                state->haveBytes = 0;
                if (ret != APP_SEC_NO_ERROR)
                    break;
            }
            break;

        case STATE_RXING_SIG_HASH:
        case STATE_RXING_SIG_PUBKEY:

            //no need for calling appSecBlockRx() as sigs are not signed, and encryption cannot be done after signing
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

    *lenUnusedP = len;

    if (ret != APP_SEC_NO_ERROR && ret != APP_SEC_NEED_MORE_TIME)
        state->curState = STATE_BAD;

    return ret;
}

AppSecErr appSecRxDataOver(struct AppSecState *state)
{
    AppSecErr ret;

    //Feed remianing data to data processor, if any
    if (state->haveBytes) {

        //not in data rx stage when the incoming data ends? This is not good (if we had encr or sign we'd not be here)
        if (state->curState != STATE_RXING_DATA) {
            state->curState = STATE_BAD;
            return APP_SEC_TOO_LITTLE_DATA;
        }

        //feed the remaining data to the data processor
        ret = appSecProcessIncomingData(state);
        if (ret != APP_SEC_NO_ERROR) {
            state->curState = STATE_BAD;
            return ret;
        }
    }

    //for unsigned/unencrypted case we have no way to judge length, so we assume it is over when we're told it is
    //this is potentially dangerous, but then again so is allowing unsigned uploads in general.
    if (!state->haveSig && !state->haveEncr && state->curState == STATE_RXING_DATA)
        state->curState = STATE_DONE;

    //Check the state and return our verdict
    if(state->curState == STATE_DONE)
        return APP_SEC_NO_ERROR;

    state->curState = STATE_BAD;
    return APP_SEC_TOO_LITTLE_DATA;
}










