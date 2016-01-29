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

#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#ifdef __cplusplus
}
#endif

#include <stdint.h>

struct Sha2state;
struct RsaState;
struct AesContext;
struct AesSetupTempWorksSpace;
struct AesCbcContext;


//tags functions and data to live in the bootloader
#define BOOTLOADER    __attribute__ ((section (".bltext")))
#define BOOTLOADER_RO __attribute__ ((section (".blrodata")))


#define BL_VERSION_1        1
#define BL_VERSION_CUR      BL_VERSION_1

#define BL_FLASH_KERNEL_ID  0x1
#define BL_FLASH_EEDATA_ID  0x2
#define BL_FLASH_APP_ID     0x4

#define BL_FLASH_KEY1       0x45670123
#define BL_FLASH_KEY2       0xCDEF89AB

struct BlVecTable {
    /* cortex requirements */
    uint32_t    blStackTop;
    void        (*blEntry)(void);
    void        (*blNmiHandler)(void);
    void        (*blHardFaultHandler)(void);
    void        (*blMmuFaultHandler)(void);
    void        (*blBusFaultHandler)(void);
    void        (*blUsageFaultHandler)(void);

    /* bl api */

    //ver 1 bl supports:

    //basics
    uint32_t        (*blGetVersion)(void);
    void            (*blReboot)(void);
    void            (*blGetSnum)(uint32_t *snum, uint8_t length);

    //flash
    int             (*blProgramShared)(uint8_t *dst, uint8_t *src, uint32_t length, uint32_t key1, uint32_t key2);
    int             (*blEraseShared)(uint32_t key1, uint32_t key2);

    //security data
    const uint32_t* (*blGetPubKeysInfo)(uint32_t *numKeys);

    //hashing, encryption, signature apis
    const uint32_t* (*blRsaPubOpIterative)(struct RsaState* state, const uint32_t *a, const uint32_t *c, uint32_t *state1, uint32_t *state2, uint32_t *stepP);
    void            (*blSha2init)(struct Sha2state *state);
    void            (*blSha2processBytes)(struct Sha2state *state, const void *bytes, uint32_t numBytes);
    const uint32_t* (*blSha2finish)(struct Sha2state *state);
    void            (*blAesInitForEncr)(struct AesContext *ctx, const uint32_t *k);
    void            (*blAesInitForDecr)(struct AesContext *ctx, struct AesSetupTempWorksSpace *tmpSpace, const uint32_t *k);
    void            (*blAesEncr)(struct AesContext *ctx, const uint32_t *src, uint32_t *dst);
    void            (*blAesDecr)(struct AesContext *ctx, const uint32_t *src, uint32_t *dst);
    void            (*blAesCbcInitForEncr)(struct AesCbcContext *ctx, const uint32_t *k, const uint32_t *iv);
    void            (*blAesCbcInitForDecr)(struct AesCbcContext *ctx, const uint32_t *k, const uint32_t *iv);
    void            (*blAesCbcEncr)(struct AesCbcContext *ctx, const uint32_t *src, uint32_t *dst);
    void            (*blAesCbcDecr)(struct AesCbcContext *ctx, const uint32_t *src, uint32_t *dst);
    const uint32_t* (*blSigPaddingVerify)(const uint32_t *rsaResult); //return pointer to hash inside the rsaResult or NULL on error
};

//for using outside of bootloader
extern struct BlVecTable BL;

#ifdef __cplusplus
}
#endif

#endif
