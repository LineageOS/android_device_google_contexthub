#ifndef _ATOMIC_H_
#define _ATOMIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* almost all platforms support byte and 32-bit operations of this sort. please do not add other sizes here */
uint32_t atomicXchgByte(volatile uint8_t *byte, uint32_t newVal);
uint32_t atomicXchg32bits(volatile uint32_t *byte, uint32_t newVal);
bool atomicCmpXchgByte(volatile uint8_t *byte, uint32_t prevVal, uint32_t newVal);
bool atomicCmpXchg32bits(volatile uint32_t *word, uint32_t prevVal, uint32_t newVal);

//returns old value
uint32_t atomicAdd(volatile uint32_t *val, uint32_t addend);

#ifdef __cplusplus
}
#endif

#endif
