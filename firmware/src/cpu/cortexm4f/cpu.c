#include <cpu.h>
#include <plat/inc/cmsis.h>
#include <seos.h>
#include <heap.h>
#include <syscall.h>
#include <string.h>


#define HARD_FAULT_DROPBOX_MAGIC  0x31415926
struct HardFaultDropbox {
    uint32_t magic;
    uint32_t r[16];
    uint32_t sr, hfsr, cfsr;
};

static struct HardFaultDropbox __attribute__((section(".neverinit"))) mDropbox;




//reloc types for this cpu type
#define NANO_RELOC_TYPE_RAM	0
#define NANO_RELOC_TYPE_FLASH	1


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
    /* print and clear dropbox */
    if (mDropbox.magic == HARD_FAULT_DROPBOX_MAGIC) {
        uint32_t i;
	osLog(LOG_INFO, "Hard Fault Dropbox not empty. Contents:\n");
	for (i = 0; i < 16; i++)
		osLog(LOG_INFO, "  R%02lu  = 0x%08lX\n", i, mDropbox.r[i]);
        osLog(LOG_INFO, "  SR   = %08lX\n", mDropbox.sr);
        osLog(LOG_INFO, "  HFSR = %08lX\n", mDropbox.hfsr);
        osLog(LOG_INFO, "  CFSR = %08lX\n", mDropbox.cfsr);
    }
    mDropbox.magic = 0;
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
    uint32_t i;

    for (i = 0; i < 4; i++)
        mDropbox.r[i] = excRegs[i];
    for (i = 0; i < 8; i++)
        mDropbox.r[i + 4] = otherRegs[i];
    mDropbox.r[12] = excRegs[4];
    mDropbox.r[13] = (uint32_t)(excRegs + 8);
    mDropbox.r[14] = excRegs[5];
    mDropbox.r[15] = excRegs[6];
    mDropbox.sr = excRegs[7];
    mDropbox.hfsr = SCB->HFSR;
    mDropbox.cfsr = SCB->CFSR;

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


bool cpuInternalAppLoad(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    platInfo->got = 0x00000000;

    return true;
}

bool cpuAppLoad(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    const uint32_t *relocsStart = (const uint32_t*)(((uint8_t*)appHdr) + appHdr->rel_start);
    const uint32_t *relocsEnd = (const uint32_t*)(((uint8_t*)appHdr) + appHdr->rel_end);
    uint8_t *mem = heapAlloc(appHdr->bss_end);

    if (!mem)
        return false;

    //calcualte and assign got
    platInfo->got = mem + appHdr->got_start;

    //clear bss
    memset(mem + appHdr->bss_start, 0, appHdr->bss_end - appHdr->bss_start);

    //copy initialized data and initialized got
    memcpy(mem + appHdr->data_start, ((uint8_t*)appHdr) + appHdr->data_data, appHdr->got_end - appHdr->data_start);

    //perform relocs
    while (relocsStart != relocsEnd) {
        const uint32_t rel = *relocsStart++;
        const uint32_t relType = rel >> 28;
        const uint32_t relOfst = rel & 0x0ffffffful;
        uint32_t *relWhere = (uint32_t*)(mem + relOfst);

        switch (relType) {
        case NANO_RELOC_TYPE_RAM:
            (*relWhere) += (uintptr_t)mem;
            break;
        case NANO_RELOC_TYPE_FLASH:
            (*relWhere) += (uintptr_t)appHdr;
            break;
        default:
            heapFree(mem);
            return false;
        }
    }

    return true;
}

void cpuAppUnload(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    heapFree((uint8_t*)platInfo->got - appHdr->got_start);
}

static uintptr_t __attribute__((naked)) callWithR9(const struct AppHdr *appHdr, void *funcOfst, void *got, uintptr_t arg1, uintptr_t arg2)
{
    asm volatile (
        "add  r12, r0, r1  \n"
        "mov  r0,  r3      \n"
        "ldr  r1,  [sp]    \n"
        "push {r9, lr}     \n"
        "mov  r9, r2       \n"
        "blx  r12          \n"
        "pop  {r9, pc}     \n"
    );

    return 0; //dummy to fool gcc
}

bool cpuAppInit(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo, uint32_t tid)
{
    if (platInfo->got)
        return callWithR9(appHdr, appHdr->funcs.init, platInfo->got, tid, 0);
    else
        return appHdr->funcs.init(tid);
}

void cpuAppEnd(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    if (platInfo->got)
        (void)callWithR9(appHdr, appHdr->funcs.end, platInfo->got, 0, 0);
    else
        appHdr->funcs.end();
}

void cpuAppHandle(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo, uint32_t evtType, const void* evtData)
{
    if (platInfo->got)
        (void)callWithR9(appHdr, appHdr->funcs.handle, platInfo->got, evtType, (uintptr_t)evtData);
    else
        appHdr->funcs.handle(evtType, evtData);
}


