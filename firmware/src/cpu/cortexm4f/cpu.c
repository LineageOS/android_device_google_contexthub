#include <inc/cpu.h>
#include <plat/inc/cmsis.h>


void cpuInit(void)
{
    /* set pendsv to be lowest priority possible */
    SCB->SHP[3] = (SCB->SHP[3] & 0xFF00FFFF) | 0x00010000;

    /* FPU on */
    SCB->CPACR |= 0x00F00000;
}

uint64_t cpuIntsOff(void)
{
    uint32_t state;

    asm volatile (
        "mrs %0, PRIMASK    \n"
        "cpsid i            \n"
        :"=r"(state)
    );

    return state;
}

uint64_t cpuIntsOn(void)
{
    uint32_t state;

    asm volatile (
        "mrs %0, PRIMASK    \n"
        "cpsie i            \n"
        :"=r"(state)
    );

    return state;
}

void cpuIntsRestore(uint64_t state)
{

    asm volatile(
        "msr PRIMASK, %0   \n"
        ::"r"((uint32_t)state)
    );   
}

