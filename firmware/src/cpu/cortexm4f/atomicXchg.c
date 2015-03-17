#include <cpu/inc/atomicXchg.h>


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


