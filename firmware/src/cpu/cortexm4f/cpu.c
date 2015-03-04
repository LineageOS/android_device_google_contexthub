#include <inc/cpu.h>
#include <plat/inc/cmsis.h>


void cpuInit(void)
{
    /* set pendsv to be lowest priority possible */
    SCB->SHP[3] = (SCB->SHP[3] & 0xFF00FFFF) | 0x00010000;
}
