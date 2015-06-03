#include <stdint.h>
#include <string.h>
#include <atomicBitset.h>
#include <atomic.h>



void atomicBitsetInit(struct AtomicBitset *set, uint32_t numBits)
{
    set->numBits = numBits;
    memset(set->words, 0, (numBits + 31) / 8);
    if (numBits & 31) //mark all high bits so that atomicBitsetFindClearAndSet() is simpler
        set->words[numBits / 32] = ((uint32_t)((int32_t)-1LL)) << (numBits & 31);
}

uint32_t atomicBitsetGetNumBits(const struct AtomicBitset *set)
{
    return set->numBits;
}

bool atomicBitsetGetBit(const struct AtomicBitset *set, uint32_t num)
{
    if (num >= set->numBits) /* any value is as good as the next */
        return false;

    return !!((set->words[num / 32]) & (1UL << (num & 31)));
}

void atomicBitsetClearBit(struct AtomicBitset *set, uint32_t num)
{
    uint32_t idx = num / 32, mask = 1UL << (num & 31);
    uint32_t *wordPtr = set->words + idx;
    uint32_t old, new;

    if (num >= set->numBits)
        return;


    do {
        old = *wordPtr;
        new = old &~ mask;
    } while (!atomicCmpXchg32bits(wordPtr, old, new));
}

int32_t atomicBitsetFindClearAndSet(struct AtomicBitset *set)
{
    uint32_t pos, i, numWords = (set->numBits + 31) / 32;
    uint32_t *wordPtr = set->words;

    for (i = 0; i < numWords; i++, wordPtr++) {
        uint32_t old, new;

        while (1) {
            old = *wordPtr;
            if (!(old + 1)) /* no work for words with no clear bits */
                break;

            pos = __builtin_ctz(~old); /* This will allocate in diff order than ARM. Since we never made any promises on order of bits returned, this is ok */
            new = old | (1 << pos);

            if (atomicCmpXchg32bits(wordPtr, old, new))
                return 32 * i + pos;
        }
    }

    return false;
}










