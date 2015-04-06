#ifndef _CM4F_ATOMIC_BITSET_H_
#define _CM4F_ATOMIC_BITSET_H_

#include <stdint.h>
#include <stdbool.h>

struct AtomicBitset {
    uint32_t numBits;
    uint32_t words[];
};

#define ATOMIC_BITSET_SZ(numbits)	(sizeof(struct AtomicBitset) + ((numbits) + 31) / 8)


void atomicBitsetInit(struct AtomicBitset *set, uint32_t numBits);
uint32_t atomicBitsetGetNumBits(const struct AtomicBitset *set);
bool atomicBitsetGetBit(const struct AtomicBitset *set, uint32_t num);
void atomicBitsetClearBit(struct AtomicBitset *set, uint32_t num);
int32_t atomicBitsetFindClearAndSet(struct AtomicBitset *set);

#endif


