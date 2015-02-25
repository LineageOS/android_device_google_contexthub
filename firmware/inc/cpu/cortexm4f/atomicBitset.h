#ifndef _CM4F_ATOMIC_BITSET_H_
#define _CM4F_ATOMIC_BITSET_H_


struct AtomicBitset {
    uint32_t numBits;
    uint32_t words[];
};

#define ATOMIC_BITSET_SZ(numbits)	(sizeof(struct AtomicBitset) + ((numbits) + 31) / 8)

#endif


