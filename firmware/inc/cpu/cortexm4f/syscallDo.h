#ifndef _CM4F_SYSCALL_DO_H_
#define _CM4F_SYSCALL_DO_H_


#ifdef __cplusplus
extern "C" {
#endif

#ifdef _OS_BUILD_
    #error "Syscalls should not be called from OS code"
#endif


#include <stdint.h>
#include <stdarg.h>

static inline uintptr_t cpuSyscallDo(va_list *args, uint32_t syscallNo)
{
    uint32_t retVal;

    asm (
        "mov r0, %1 \n"
        "mov r1, %2 \n"
        "swi 0      \n"
        "mov %0, r0 \n"
        :"=r"(retVal)
        :"r"(args), "r"(syscallNo)
        :"memory", "r0", "r1"
    );

    return retVal;
}


#ifdef __cplusplus
}
#endif

#endif

