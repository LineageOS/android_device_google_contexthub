#include <cpu/inc/atomicXchg.h>


uint8_t atomicXchgByte(volatile uint8_t *byte, uint8_t newVal)
{
    uint32_t prevVal, storeFailed;

    do {
        asm volatile(
            "ldrexb %0,     [%3] \n"
            "strexb %1, %2, [%3] \n"
            :"=r"(prevVal), "=r"(storeFailed)
            :"r"(newVal), "r"(byte)
            :"memory"
        );
    } while (storeFailed);

    return prevVal;
}

uint32_t atomicXchg32bits(volatile uint32_t *byte, uint32_t newVal)
{
    uint32_t prevVal, storeFailed;

    do {
        asm volatile(
            "ldrex %0,     [%3] \n"
            "strex %1, %2, [%3] \n"
            :"=r"(prevVal), "=r"(storeFailed)
            :"r"(newVal), "r"(byte)
            :"memory"
        );
    } while (storeFailed);

    return prevVal;
}


