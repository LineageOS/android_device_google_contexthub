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

#ifndef __HOSTINTF_H
#define __HOSTINTF_H

#include <stdint.h>
#include <atomicBitset.h>

/**
 * System-facing hostIntf API
 */

#define HOSTINTF_MAX_INTERRUPTS     256

void hostIntfCopyInterrupts(void *dst, uint32_t numBits);
void hostIntfClearInterrupts();
void hostIntfSetInterrupt(uint32_t bit);
void hostIntfClearInterrupt(uint32_t bit);
void hostIntfSetInterruptMask(uint32_t bit);
void hostIntfClearInterruptMask(uint32_t bit);
void hostIntfPacketFree(void *ptr);
bool hostIntfPacketDequeue(void *ptr, uint32_t *wakeup, uint32_t *nonwakeup);
void hostIntfSetBusy(bool busy);
void hostIntfRxPacket();

#endif /* __HOSTINTF_H */
