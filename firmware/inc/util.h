#ifndef __UTIL_H
#define __UTIL_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>

#define ARRAY_SIZE(a)   (sizeof((a)) / sizeof((a)[0]))
#define LIKELY(x)	(__builtin_expect(x, 1))
#define UNLIKELY(x)	(__builtin_expect(x, 0))

#define container_of(addr, struct_name, field_name) \
    ((struct_name *)((char *)(addr) - offsetof(struct_name, field_name)))

static inline bool IS_POWER_OF_TWO(unsigned int n)
{
    return !(n & (n - 1));
}

static inline int LOG2_FLOOR(unsigned int n)
{
    if (UNLIKELY(n == 0))
        return INT_MIN;

    // floor(log2(n)) = MSB(n) = (# of bits) - (# of leading zeros) - 1
    return 8 * sizeof(n) - __builtin_clz(n) - 1;
}

static inline int LOG2_CEIL(unsigned int n)
{
    return IS_POWER_OF_TWO(n) ? LOG2_FLOOR(n) : LOG2_FLOOR(n) + 1;
}

#ifdef SYSCALL_VARARGS_PARAMS_PASSED_AS_PTRS
  #define VA_LIST_TO_INTEGER(__vl)  ((uintptr_t)(&__vl))
  #define INTEGER_TO_VA_LIST(__ptr)  (*(va_list*)(__ptr))
#else
  #define VA_LIST_TO_INTEGER(__vl)  (*(uintptr_t*)(&__vl))
  #define INTEGER_TO_VA_LIST(__ptr)  ({uintptr_t tmp = __ptr; (*(va_list*)(&tmp)); })
#endif

#endif /* __UTIL_H */
