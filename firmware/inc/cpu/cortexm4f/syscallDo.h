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

#define cpuSyscallDo(syscallNo, vaListPtr)                     \
    ({                                                         \
        register uint32_t _r0 asm("r0") = syscallNo;           \
        register uint32_t _r1 asm("r1") = (uint32_t)vaListPtr; \
        asm volatile (                                         \
            "swi 0      \n"                                    \
            :                                                  \
            :"r"(_r0), "r"(_r1)                                \
            :"memory"                                          \
        );                                                     \
        _r0;                                                   \
    })

/* these are specific to arm-eabi-32-le-gcc ONLY. YMMV otherwise */

#define cpuSyscallDo0P(syscallNo)                    \
    ({                                               \
        register uint32_t _r0 asm("r0") = syscallNo; \
        asm volatile (                               \
            "swi 0      \n"                          \
            :                                        \
            :"r"(_r0)                                \
            :"memory"                                \
        );                                           \
        _r0;                                         \
    })


#define cpuSyscallDo1P(syscallNo, p1)                  \
    ({                                                 \
        register uint32_t _r0 asm("r0") = syscallNo;   \
        register uint32_t _r1 asm("r1") = (uint32_t)p1;\
        asm volatile (                                 \
            "swi 1      \n"                            \
            :                                          \
            :"r"(_r0), "r"(_r1)                        \
            :"memory"                                  \
        );                                             \
        _r0;                                           \
    })



#define cpuSyscallDo2P(syscallNo, p1, p2)              \
    ({                                                 \
        register uint32_t _r0 asm("r0") = syscallNo;   \
        register uint32_t _r1 asm("r1") = (uint32_t)p1;\
        register uint32_t _r2 asm("r2") = (uint32_t)p2;\
        asm volatile (                                 \
            "swi 1      \n"                            \
            :                                          \
            :"r"(_r0), "r"(_r1), "r"(_r2)              \
            :"memory"                                  \
        );                                             \
        _r0;                                           \
    })

#define cpuSyscallDo3P(syscallNo, p1, p2, p3)          \
    ({                                                 \
        register uint32_t _r0 asm("r0") = syscallNo;   \
        register uint32_t _r1 asm("r1") = (uint32_t)p1;\
        register uint32_t _r2 asm("r2") = (uint32_t)p2;\
        register uint32_t _r3 asm("r3") = (uint32_t)p3;\
        asm volatile (                                 \
            "swi 1      \n"                            \
            :                                          \
            :"r"(_r0), "r"(_r1), "r"(_r2), "r"(_r3)    \
            :"memory"                                  \
        );                                             \
        _r0;                                           \
    })

#define cpuSyscallDo4P(syscallNo, p1, p2, p3, p4)              \
    ({                                                         \
        register uint32_t _r0 asm("r0") = syscallNo;           \
        register uint32_t _r1 asm("r1") = (uint32_t)p1;        \
        register uint32_t _r2 asm("r2") = (uint32_t)p2;        \
        register uint32_t _r3 asm("r3") = (uint32_t)p3;        \
        register uint32_t _r12 asm("r12") = (uint32_t)p4;      \
        asm volatile (                                         \
            "swi 1      \n"                                    \
            :                                                  \
            :"r"(_r0), "r"(_r1), "r"(_r2), "r"(_r3), "r"(_r12) \
            :"memory"                                          \
        );                                                     \
        _r0;                                                   \
    })




#ifdef __cplusplus
}
#endif

#endif

