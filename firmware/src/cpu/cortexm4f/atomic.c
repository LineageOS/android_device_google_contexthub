#include <atomic.h>

uint32_t atomicAdd(volatile uint32_t *val, uint32_t addend)
{
    uint32_t prevVal, storeFailed, tmp;

    do {
        asm volatile(
            "ldrexb %0,     [%4] \n"
            "add    %2, %0, %3   \n"
            "strexb %1, %2, [%4] \n"
            :"=r"(prevVal), "=r"(storeFailed), "=r"(tmp), "=r"(addend), "=r"(val)
            :"3"(addend), "4"(val)
            :"memory"
        );
    } while (storeFailed);

    return prevVal;
}

uint32_t atomicXchgByte(volatile uint8_t *byte, uint32_t newVal)
{
    uint32_t prevVal, storeFailed;

    do {
        asm volatile(
            "ldrexb %0,     [%3] \n"
            "strexb %1, %2, [%3] \n"
            :"=r"(prevVal), "=r"(storeFailed), "=r"(newVal), "=r"(byte)
            :"2"(newVal), "3"(byte)
            :"memory"
        );
    } while (storeFailed);

    return prevVal;
}

uint32_t atomicXchg32bits(volatile uint32_t *word, uint32_t newVal)
{
    uint32_t prevVal, storeFailed;

    do {
        asm volatile(
            "ldrex %0,     [%3] \n"
            "strex %1, %2, [%3] \n"
            :"=r"(prevVal), "=r"(storeFailed), "=r"(newVal), "=r"(word)
            :"2"(newVal), "3"(word)
            :"memory"
        );
    } while (storeFailed);

    return prevVal;
}

bool atomicCmpXchgByte(volatile uint8_t *byte, uint32_t prevVal, uint32_t newVal)
{
    uint32_t currVal, storeFailed;

    do {
        asm volatile(
            "ldrexb %0,     [%1] \n"
            :"=r"(currVal), "=r"(byte)
            :"1"(byte)
            :"memory"
        );

        if (currVal != prevVal)
            return false;

        asm volatile(
            "strexb %0, %1, [%2] \n"
            :"=r"(storeFailed), "=r"(newVal), "=r"(byte)
            :"1"(newVal), "2"(byte)
            :"memory"
        );
    } while (storeFailed);

    return true;
}

bool atomicCmpXchg32bits(volatile uint32_t *word, uint32_t prevVal, uint32_t newVal)
{
    uint32_t currVal, storeFailed;

    do {
        asm volatile(
            "ldrex %0,     [%1] \n"
            :"=r"(currVal), "=r"(word)
            :"1"(word)
            :"memory"
        );

        if (currVal != prevVal)
            return false;

        asm volatile(
            "strex %0, %1, [%2] \n"
            :"=r"(storeFailed), "=r"(newVal), "=r"(word)
            :"1"(newVal), "2"(word)
            :"memory"
        );
    } while (storeFailed);

    return true;
}
