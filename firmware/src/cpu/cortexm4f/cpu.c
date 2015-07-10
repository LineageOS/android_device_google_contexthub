#include <cpu.h>
#include <plat/inc/cmsis.h>
#include <seos.h>
#include <syscall.h>


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

static void __attribute__((used)) syscallHandler(uint32_t syscallNr, va_list *args, uintptr_t *excRegs)
{
    uint16_t *svcPC = ((uint16_t *)(excRegs[6])) - 1;
    uint32_t svcNo = (*svcPC) & 0xFF;
    SyscallFunc handler;

    if (svcNo)
        osLog(LOG_WARN, "Unknown SVC 0x%02lX called at 0x%08lX\n", svcNo, (unsigned long)svcPC);
    else if (!(handler = syscallGetHandler(syscallNr)))
        osLog(LOG_WARN, "Unknown syscall 0x%08lX called at 0x%08lX\n", (unsigned long)syscallNr, (unsigned long)svcPC);
    else
        handler(excRegs, args);
}

void SVC_Handler(uint32_t syscallNr, va_list *args);
void __attribute__((naked)) SVC_Handler(uint32_t syscallNr, va_list *args)
{
    asm volatile(
        "tst lr, #4         \n"
        "ite eq             \n"
        "mrseq r2, msp      \n"
        "mrsne r2, psp      \n"
        "b syscallHandler   \n"
    );
}

static void __attribute__((used)) logHardFault(uintptr_t *excRegs, uintptr_t* otherRegs)
{
    osLog(LOG_ERROR, "*HARD FAULT* SR  = %08lX\n", (unsigned long)excRegs[7]);
    osLog(LOG_ERROR, "R0  = %08lX   R8  = %08lX\n", (unsigned long)excRegs[0], (unsigned long)otherRegs[4]);
    osLog(LOG_ERROR, "R1  = %08lX   R9  = %08lX\n", (unsigned long)excRegs[1], (unsigned long)otherRegs[5]);
    osLog(LOG_ERROR, "R2  = %08lX   R10 = %08lX\n", (unsigned long)excRegs[2], (unsigned long)otherRegs[6]);
    osLog(LOG_ERROR, "R3  = %08lX   R11 = %08lX\n", (unsigned long)excRegs[3], (unsigned long)otherRegs[7]);
    osLog(LOG_ERROR, "R4  = %08lX   R12 = %08lX\n", (unsigned long)otherRegs[0], (unsigned long)excRegs[4]);
    osLog(LOG_ERROR, "R5  = %08lX   SP  = %08lX\n", (unsigned long)otherRegs[1], (unsigned long)(excRegs + 8));
    osLog(LOG_ERROR, "R6  = %08lX   LR  = %08lX\n", (unsigned long)otherRegs[2], (unsigned long)excRegs[5]);
    osLog(LOG_ERROR, "R7  = %08lX   PC  = %08lX\n", (unsigned long)otherRegs[3], (unsigned long)excRegs[6]);
    osLog(LOG_ERROR, "HFSR= %08lX   CFSR= %08lX\n", (unsigned long)SCB->HFSR, (unsigned long)SCB->CFSR);
    while(1);
}

void HardFault_Handler(void);
void __attribute__((naked)) HardFault_Handler(void)
{
    asm volatile(
        "tst lr, #4         \n"
        "ite eq             \n"
        "mrseq r0, msp      \n"
        "mrsne r0, psp      \n"
        "push  {r4-r11}     \n"
        "mov   r1, sp       \n"
        "b     logHardFault \n"
    );
}



