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

#ifndef _SYSCALL_H_
#define _SYSCALL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

/*
 * System calls are indexed by an unsigned 32-bit constant path. To avoid having a
 * single gigantic table, we use a system of 4-level tables to dispatch them. In
 * order of use the tables use: 5, 10, 8, 9 bits, from high to low, so for example
 * syscall 0x19918FC8 would use table indices 3, 200, 199, 456 since it is equal to
 * (3 << (10 + 8 + 9)) + (200 << (8 + 9)) + (199 << 9) + 456. Each table may be
 * truncated (with a length field specified), and may be in RAM or ROM, as seen fit.
 * The expected use case is that the first level detemines grand purpose
 * (OS, driver, google app, 3rd party app, etc), second level determines vendor,
 * third level determines subsystem, and the last level determines desired function.
 */

#define SYSCALL_BITS_LEVEL_0      5  /* domain */
#define SYSCALL_BITS_LEVEL_1      10 /* family */
#define SYSCALL_BITS_LEVEL_2      8  /* genus */
#define SYSCALL_BITS_LEVEL_3      9  /* species */

#define SYSCALL_CUT_SCALE_SHIFT(_val, _cut, _shift)   ((((uint32_t)(_val)) & ((1UL << (_cut)) - 1)) << (_shift))
#define SYSCALL_NO(_domain, _family, _genus, _species) (                                                                                    \
            SYSCALL_CUT_SCALE_SHIFT((_domain) , SYSCALL_BITS_LEVEL_0, SYSCALL_BITS_LEVEL_1 + SYSCALL_BITS_LEVEL_2 + SYSCALL_BITS_LEVEL_3) + \
            SYSCALL_CUT_SCALE_SHIFT((_family) , SYSCALL_BITS_LEVEL_1,                        SYSCALL_BITS_LEVEL_2 + SYSCALL_BITS_LEVEL_3) + \
            SYSCALL_CUT_SCALE_SHIFT((_genus)  , SYSCALL_BITS_LEVEL_2,                                               SYSCALL_BITS_LEVEL_3) + \
            SYSCALL_CUT_SCALE_SHIFT((_species), SYSCALL_BITS_LEVEL_3,                                                                  0)   \
                                                       )

//level 0 indices:
#define SYSCALL_DOMAIN_OS         0
#define SYSCALL_DOMAIN_DRIVERS    1 /* hardware drivers for custom hardware */


typedef void (*SyscallFunc)(uintptr_t *retValP, va_list args); /* you better know what args you need */

struct SyscallTable {
    uint32_t numEntries;
    union SyscallTableEntry {
        struct SyscallTable *subtable;
        SyscallFunc func;
    } entry[];
};


void syscallInit(void);

//add a complete table
bool syscallAddTable(uint32_t path, uint32_t level, struct SyscallTable *table);

//this will only work if the backing table exists...this is intentional to avoid auto growth in scary ways
bool syscallAddFunc(uint32_t path, SyscallFunc func);

SyscallFunc syscallGetHandler(uint32_t path); // NULL if none


/*
 * Userspace callbacks are a complicated topic. They allow us to call back to a userspace
 * piece of code in userspace context. This is rather complicated, and so these functions
 * exist to simplify it. The callbacks are allowed up to 4 params, of uintptr_t size. They
 * are also allowed a single uintptr_t return value. First, one must create a callback
 * state. The function syscallUserspaceCallbackAlloc() can be used for that. It takes in
 * the callback function pointer as well as up to four params. It creates a reusable state
 * structure that can be used to call this callback as many times as one wishes. The params
 * are saved in the struct, as is the function pointer. The inverse of this function is
 * syscallUserspaceCallbackFree(). It frees the state. To actually call the callback, use
 * syscallUserspaceCallbackCall(). It takes in the struct, and optionally can override the
 * params with new values. If you wish to do that, pass non-NULL values as overrideParam#P.
 * The override values are NOT saved in the struct, and are only used for this one call.
 */
//used to alloc state to call to userspace
struct UserspaceCallback;
struct UserspaceCallback* syscallUserspaceCallbackAlloc(void *func, uintptr_t param1, uintptr_t param2, uintptr_t param3, uintptr_t param4);
void syscallUserspaceCallbackFree(struct UserspaceCallback *ucbk);

//used to call to userspace
uintptr_t syscallUserspaceCallbackCall(struct UserspaceCallback *ucbk, const uintptr_t *overrideParam1P, const uintptr_t *overrideParam2P, const uintptr_t *overrideParam3P, const uintptr_t *overrideParam4P);

#ifdef __cplusplus
}
#endif

#endif

