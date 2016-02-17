/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <plat/inc/cmsis.h>
#include <plat/inc/plat.h>
#include <syscall.h>
#include <string.h>
#include <seos.h>
#include <heap.h>
#include <cpu.h>


#define HARD_FAULT_DROPBOX_MAGIC  0x31415926
struct HardFaultDropbox {
    uint32_t magic;
    uint32_t r[16];
    uint32_t sr, hfsr, cfsr;
};

/* //if your device persists ram, you can use this instead:
 * static struct HardFaultDropbox* getDropbox(void)
 * {
 *     static struct HardFaultDropbox __attribute__((section(".neverinit"))) dbx;
 *     return &dbx;
 * }
 */

static struct HardFaultDropbox* getDropbox(void)
{
    uint32_t bytes = 0;
    void *loc = platGetPersistentRamStore(&bytes);

    return bytes >= sizeof(struct HardFaultDropbox) ? (struct HardFaultDropbox*)loc : NULL;
}

void cpuInit(void)
{
    /* set pendsv to be lowest priority possible */
    NVIC_SetPriority(PendSV_IRQn, 1 << (8 - __NVIC_PRIO_BITS));

    /* set SVC to be highest possible priority */
    NVIC_SetPriority(SVCall_IRQn, 0xff);

    /* FPU on */
    SCB->CPACR |= 0x00F00000;
}


void cpuInitLate(void)
{
    struct HardFaultDropbox *dbx = getDropbox();

    /* print and clear dropbox */
    if (dbx->magic == HARD_FAULT_DROPBOX_MAGIC) {
        uint32_t i;
	osLog(LOG_INFO, "Hard Fault Dropbox not empty. Contents:\n");
	for (i = 0; i < 16; i++)
		osLog(LOG_INFO, "  R%02lu  = 0x%08lX\n", i, dbx->r[i]);
        osLog(LOG_INFO, "  SR   = %08lX\n", dbx->sr);
        osLog(LOG_INFO, "  HFSR = %08lX\n", dbx->hfsr);
        osLog(LOG_INFO, "  CFSR = %08lX\n", dbx->cfsr);
    }
    dbx->magic = 0;
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

static void __attribute__((used)) syscallHandler(uintptr_t *excRegs)
{
    uint16_t *svcPC = ((uint16_t *)(excRegs[6])) - 1;
    uint32_t svcNo = (*svcPC) & 0xFF;
    uint32_t syscallNr = excRegs[0];
    SyscallFunc handler;
    va_list args_long = *(va_list*)(excRegs + 1);
    uintptr_t *fastParams = excRegs + 1;
    va_list args_fast = *(va_list*)(&fastParams);

    if (svcNo > 1)
        osLog(LOG_WARN, "Unknown SVC 0x%02lX called at 0x%08lX\n", svcNo, (unsigned long)svcPC);
    else if (!(handler = syscallGetHandler(syscallNr)))
        osLog(LOG_WARN, "Unknown syscall 0x%08lX called at 0x%08lX\n", (unsigned long)syscallNr, (unsigned long)svcPC);
    else
        handler(excRegs, svcNo ? args_fast : args_long);
}

void SVC_Handler(void);
void __attribute__((naked)) SVC_Handler(void)
{
    asm volatile(
        "tst lr, #4         \n"
        "ite eq             \n"
        "mrseq r0, msp      \n"
        "mrsne r0, psp      \n"
        "b syscallHandler   \n"
    );
}

static void __attribute__((used)) logHardFault(uintptr_t *excRegs, uintptr_t* otherRegs)
{
    struct HardFaultDropbox *dbx = getDropbox();
    uint32_t i;

    for (i = 0; i < 4; i++)
        dbx->r[i] = excRegs[i];
    for (i = 0; i < 8; i++)
        dbx->r[i + 4] = otherRegs[i];
    dbx->r[12] = excRegs[4];
    dbx->r[13] = (uint32_t)(excRegs + 8);
    dbx->r[14] = excRegs[5];
    dbx->r[15] = excRegs[6];
    dbx->sr = excRegs[7];
    dbx->hfsr = SCB->HFSR;
    dbx->cfsr = SCB->CFSR;
    dbx->magic = HARD_FAULT_DROPBOX_MAGIC;

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

    //reset
    SCB->AIRCR = 0x05FA0004;

    //and in case somehow we do not, loop
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


