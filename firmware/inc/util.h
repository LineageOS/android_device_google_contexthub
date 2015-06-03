#ifndef __UTIL_H
#define __UTIL_H

#include <limits.h>
#include <stdbool.h>

#define ARRAY_SIZE(a)   (sizeof((a)) / sizeof((a)[0]))
#define LIKELY(x)	(__builtin_expect(x, 1))
#define UNLIKELY(x)	(__builtin_expect(x, 0))

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

#endif /* __UTIL_H */
