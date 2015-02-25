#ifndef _ATOMIC_BITSET_H_
#define _ATOMIC_BITSET_H_

#include <stdint.h>
#include <stdbool.h>
#include <cpu/inc/atomicBitset.h>

struct AtomicBitset;

//static size calc:
//	ATOMIC_BITSET_SZ(numbits)
//dynamic init:
//	uint32_t sz = atomicBitsetSize(uint32_t numBits);
//	struct AtomicBitset *set = (struct AtomicBitset*)heapAlloc(sz);
//	atomicBitsetInit(set, numBits);


void atomicBitsetInit(struct AtomicBitset *set, uint32_t numBits); //inited state is all zeroes
uint32_t atomicBitsetGetNumBits(const struct AtomicBitset *set);
bool atomicBitsetGetBit(const struct AtomicBitset *set, uint32_t num);
void atomicBitsetSetBit(struct AtomicBitset *set, uint32_t num, bool val);

//find a clear bit and set it atomically.
// returns bit number or negative if none.
// only one pass is attempted so if index 0 is cleared after we've looked at it, too bad
int32_t atomicBitsetFindClearAndSet(struct AtomicBitset *set);




#endif

