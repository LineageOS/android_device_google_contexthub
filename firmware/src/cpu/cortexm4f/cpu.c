#include <inc/cpu.h>
#include <cpu/inc/cmsis.h>


void cpuInit(void)
{
    /* set pendsv to be lowest priority possible */
    SCB->SHPR3 = (SCB->SHPR3 & 0xFF00FFFF) | 0x00010000;
}
