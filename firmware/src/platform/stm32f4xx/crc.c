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

#include <string.h>

#include <crc.h>
#include <seos.h>

#include <plat/inc/pwr.h>

struct StmCrcRegs {
    volatile uint32_t DR;
    volatile uint32_t IDR;
    volatile uint32_t CR;
};

#define STM_CRC_CR_RESET        1

static struct StmCrcRegs *gCrcRegs = (struct StmCrcRegs *)CRC_BASE;

uint32_t crc32(const void *buf, size_t size)
{
    const uint32_t *words = (const uint32_t *)buf;
    size_t numWords = size / 4;
    unsigned int leftoverBytes = size % 4;

    pwrUnitClock(PERIPH_BUS_AHB1, PERIPH_AHB1_CRC, true);

    gCrcRegs->CR = STM_CRC_CR_RESET;
    size_t i;
    for (i = 0; i < numWords; i++)
        gCrcRegs->DR = words[i];

    if (leftoverBytes) {
        uint32_t word = 0;
        memcpy(&word, words + numWords, leftoverBytes);
        /* n.b.: no shifting is needed, since the CRC block looks at the
         * lowest byte first (i.e., we need the padding in the upper bytes)
         */
        gCrcRegs->DR = word;
    }

    uint32_t crc = gCrcRegs->DR;
    pwrUnitClock(PERIPH_BUS_AHB1, PERIPH_AHB1_CRC, false);
    return crc;
}
