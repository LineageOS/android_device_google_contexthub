#ifndef _CPU_MATH_H_
#define _CPU_MATH_H_

#include <stdint.h>

uint32_t cpuMathUint44Div1000ToUint32_slow_path(uint64_t val);

static inline uint32_t cpuMathUint44Div1000ToUint32(uint64_t val)
{
    if (val >> 32)
        return cpuMathUint44Div1000ToUint32_slow_path(val);
    else
        return (uint32_t)val / 1000;
}


//correctly handles 0, 1, powers of 2, and all else to calculate "(1 << 64) / val"
//do not even think of using this on non-compile-time-constant values!
#define U64_RECIPROCAL_CALCULATE(val)  ((val) & ((val) - 1)) ? (0xffffffffffffffffull / (val)) : (((val) <= 1) ? 0xffffffffffffffffull : (0x8000000000000000ull / ((val) >> 1)))

uint64_t cpuMathRecipAssistedUdiv64by64(uint64_t num, uint64_t denom, uint64_t denomRecip);
uint64_t cpuMathRecipAssistedUdiv64by32(uint64_t num, uint32_t denom, uint64_t denomRecip);


#endif

