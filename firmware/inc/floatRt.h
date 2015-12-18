#ifndef _FLOAT_RT_H_
#define _FLOAT_RT_H_

#include <stdint.h>

#ifdef MY_FLOAT_RUNTIME_SUCKS

uint64_t floatToUint64(float f);
int64_t floatToInt64(float f);
float floatFromUint64(uint64_t v);
float floatFromInt64(int64_t v);


#else //MY_FLOAT_RUNTIME_SUCKS

static inline uint64_t floatToUint64(float f)
{
	return f;
}

static inline int64_t floatToInt64(float f)
{
	return f;
}

static inline float floatFromUint64(uint64_t v)
{
	return v;
}

static inline float floatFromInt64(int64_t v)
{
	return v;
}

#endif //MY_FLOAT_RUNTIME_SUCKS

#endif


