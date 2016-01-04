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

#ifndef _STM32_PLAT_H_
#define _STM32_PLAT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <seos.h>


enum PlatSleepDevID
{
    Stm32sleepDevTim2, /* we use this for short sleeps in WFI mode */
    Stm32sleepDevTim4, /* input capture uses this, potentially */
    Stm32sleepDevTim5, /* input capture uses this, potentially */
    Stm32sleepDevTim9, /* input capture uses this, potentially */
    Stm32sleepWakeup,  /* we use this to wakeup from AP */
    Stm32sleepDevSpi2, /* we use this to prevent stop mode during spi2 xfers */
    Stm32sleepDevI2c1, /* we use this to prevent stop mode during i2c1 xfers */

    Stm32sleepDevNum,  //must be last always, and must be <= PLAT_MAX_SLEEP_DEVS
};

struct StmPlatEeDataGeneric {
    uint32_t eeDataType : 20;
    uint32_t eeDataLen  : 12; // not incl header
} __attribute__((packed));

struct StmPlatEeDataEncrKey {
    struct StmPlatEeDataGeneric hdr;
    uint64_t keyID;
    uint8_t key[32];
} __attribute__((packed));


#define PREPOPULATED_ENCR_KEY(name, keyid, ...) \
    const struct StmPlatEeDataEncrKey __attribute__ ((section (".eedata"))) __EE__ ## name = { { EE_DATA_TYPE_ENCR_KEY, sizeof(struct StmPlatEeDataEncrKey) - sizeof(struct StmPlatEeDataGeneric)}, keyid, {__VA_ARGS__}}

#ifdef __cplusplus
}
#endif

#endif

