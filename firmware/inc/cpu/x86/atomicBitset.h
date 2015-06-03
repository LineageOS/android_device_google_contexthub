#ifndef _X86_ATOMIC_BITSET_H_
#define _X86_ATOMIC_BITSET_H_

#include <stdint.h>
#include <stdbool.h>

struct AtomicBitset {
    uint32_t numBits;
    uint32_t words[];
};

#define ATOMIC_BITSET_SZ(numbits)	(sizeof(struct AtomicBitset) + ((numbits) + 31) / 8)
#define ATOMIC_BITSET_DECL(nam, numbits, extra_keyword)    extra_keyword uint8_t _##nam##_store [ATOMIC_BITSET_SZ(numbits)] __attribute__((aligned(4))); extra_keyword struct AtomicBitset *nam = (struct AtomicBitset*)_##nam##_store


void atomicBitsetInit(struct AtomicBitset *set, uint32_t numBits);
uint32_t atomicBitsetGetNumBits(const struct AtomicBitset *set);
bool atomicBitsetGetBit(const struct AtomicBitset *set, uint32_t num);
void atomicBitsetClearBit(struct AtomicBitset *set, uint32_t num);
int32_t atomicBitsetFindClearAndSet(struct AtomicBitset *set);

#endif


