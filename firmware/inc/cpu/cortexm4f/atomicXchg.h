#ifndef _CM4F_ATOMIC_XCHG_H_
#define _CM4F_ATOMIC_XCHG_H_

#include <stdint.h>
#include <stdbool.h>

/* almsot all platforms support byte and 32-bit operations fo this sort. please do not add other sizes here */
uint32_t atomicXchgByte(volatile uint8_t *byte, uint32_t newVal);
uint32_t atomicXchg32bits(volatile uint32_t *byte, uint32_t newVal);

#endif

