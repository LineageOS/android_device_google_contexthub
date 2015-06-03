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

#define SYSCALL_BITS_LEVEL_0      5
#define SYSCALL_BITS_LEVEL_1      10
#define SYSCALL_BITS_LEVEL_2      8
#define SYSCALL_BITS_LEVEL_3      9


typedef uint32_t (*SyscallFunc)(va_list args); /* you better know what args you need */

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
bool syscallAddFunc(uint32_t path, SyscallFunc *func);

SyscallFunc* syscallGetHandler(uint32_t path); // NULL if none


#ifdef __cplusplus
}
#endif

#endif

