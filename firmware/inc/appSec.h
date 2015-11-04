#ifndef _APP_SEC_H_
#define _APP_SEC_H_

#include <stdbool.h>
#include <stdint.h>

//types
struct AppSecState;
typedef uint32_t AppSecErr;

//callbacks
typedef AppSecErr (*AppSecWriteCbk)(const void *data, uint32_t len);
typedef AppSecErr (*AppSecPubKeyFindCbk)(const uint32_t *gotKey, bool *foundP); // fill in *foundP on result of lookup
typedef AppSecErr (*AppSecGetAesKeyCbk)(uint64_t keyIdx, uint8_t *keyBuf); // return APP_SEC_KEY_NOT_FOUND or APP_SEC_NO_ERROR

//return values
#define APP_SEC_NO_ERROR            0 //all went ok
#define APP_SEC_KEY_NOT_FOUND       1 //we did not find the encr key
#define APP_SEC_HEADER_ERROR        2 //data (decrypted or input) has no recognizable header
#define APP_SEC_TOO_MUCH_DATA       3 //we got more data than expected
#define APP_SEC_TOO_LITTLE_DATA     4 //we got less data than expected
#define APP_SEC_SIG_VERIFY_FAIL     5 //some signature verification failed
#define APP_SEC_SIG_DECODE_FAIL     6 //some signature verification failed
#define APP_SEC_SIG_ROOT_UNKNOWN    7 //signatures all verified but the referenced root of trust is unknown
#define APP_SEC_MEMORY_ERROR        8 //we ran out of memory while doing things
#define APP_SEC_INVALID_DATA        9 //data is invalid in some way not described by other error messages
#define APP_SEC_BAD                10 //something irrecoverably bad happened and we gave up. Sorry...

//init/deinit
struct AppSecState *appSecInit(AppSecWriteCbk writeCbk, AppSecPubKeyFindCbk pubKeyFindCbk, AppSecGetAesKeyCbk aesKeyAccessCbk, bool mandateSigning);
void appSecDeinit (struct AppSecState *state);

//actually doing things
AppSecErr appSecRxData(struct AppSecState *state, const void *data, uint32_t len);
AppSecErr appSecRxDataOver(struct AppSecState *state); //caleed when there is no more data




#endif

