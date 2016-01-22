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

#ifndef _RSA_H_
#define _RSA_H_

#ifndef HOST_BUILD
#include <plat/inc/bl.h> //for BOOTLOADER define
#endif
#include <stdint.h>

#define RSA_LEN	    2048
#define RSA_LIMBS   ((RSA_LEN + 31)/ 32)
#define RSA_BYTES   sizeof(uint32_t[RSA_LIMBS])
#define RSA_WORDS   (RSA_BYTES / sizeof(uint32_t)) //limbs may change in size, but words are always same :)

struct RsaState {
    uint32_t tmpA[RSA_LIMBS * 2];
    uint32_t tmpB[RSA_LIMBS + 1];

#if defined(RSA_SUPPORT_PRIV_OP_LOWRAM)
    uint32_t tmpC[RSA_LIMBS + 1];
#elif defined (RSA_SUPPORT_PRIV_OP_BIGRAM)
    uint32_t tmpC[RSA_LIMBS * 2];
#endif
};

//DO NOT CALL THESE DIRECTLY, IT WILL BREAK!
//calculate a ^ 65537 mod c, where a and c are each exactly RSA_LEN bits long, result is only valid as long as state is. state needs no init
const uint32_t* _rsaPubOp(struct RsaState* state, const uint32_t *a, const uint32_t *c);

#if defined(RSA_SUPPORT_PRIV_OP_LOWRAM) || defined (RSA_SUPPORT_PRIV_OP_BIGRAM)
//calculate a ^ b mod c, where a and c are each exactly RSA_LEN bits long, result is only valid as long as state is. state needs no init
const uint32_t* _rsaPrivOp(struct RsaState* state, const uint32_t *a, const uint32_t *b, const uint32_t *c);

#ifdef ARM
#error "RSA private ops should never be compiled into firmware. You *ARE* doing something wrong! Stop!"
#endif

#endif

#ifndef HOST_BUILD
static inline const uint32_t* rsaPubOp(struct RsaState* state, const uint32_t *a, const uint32_t *c)
{
    return BL.blRsaPubOp(state, a, c);
}
#endif //HOST_BUILD


#endif

