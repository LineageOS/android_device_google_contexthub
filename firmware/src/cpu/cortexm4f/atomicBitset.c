#include <stdint.h>
#include <string.h>
#include <atomicBitset.h>



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

void atomicBitsetSetBit(struct AtomicBitset *set, uint32_t num, bool val)
{
    uint32_t idx = num / 32, mask = 1UL << (num & 31), status, tmp;

    if (num >= set->numBits)
        return;

    do {
        asm volatile(
            "    ldrex %0, [%2]       \n"
            "    bic   %0, %3         \n"
            "    strex %1, %0, [%2]   \n"
            :"=r"(tmp), "=r"(status)
            :"r"(set->words + idx), "r"(mask)
            :"cc","memory"
        );
    } while (status);
}

int32_t atomicBitsetFindClearAndSet(struct AtomicBitset *set)
{
    uint32_t idx, numWords = (set->numBits + 31) / 32;
    uint32_t scratch1, scratch2, scratch3, bit = 32;

    for (idx = 0; idx < numWords && bit == 32; idx++) {
        asm volatile(
            "1:                       \n"
            "    ldrex %0, [%4]       \n"
            "    mvns  %1, %0         \n"
            "    beq   1f             \n"
            "    clz   %1, %1         \n"
            "    rsb   %1, #31        \n"
            "    lsl   %3, %2, %1     \n"
            "    orr   %0, %3         \n"
            "    strex %3, %0, [%4]   \n"
            "    cmp   %3, #0         \n"
            "    bne   1b             \n"
            "1:                       \n"
            :"=r"(scratch1), "=r"(bit), "=r"(scratch2), "=r"(scratch3)
            :"r"(set->words + idx), "1"(32), "2"(1)
            :"cc", "memory"
        );
    }

    return (idx == numWords) ? -1 : (idx * 32) + bit;
}










