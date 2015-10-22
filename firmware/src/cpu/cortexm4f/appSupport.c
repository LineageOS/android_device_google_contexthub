#include <cpu/inc/appRelocFormat.h>
#include <string.h>
#include <stdint.h>
#include <heap.h>
#include <seos.h>
#include <cpu.h>



//reloc types for this cpu type
#define NANO_RELOC_TYPE_RAM	0
#define NANO_RELOC_TYPE_FLASH	1


static bool handleRelNumber(uint32_t *ofstP, uint32_t type, uint32_t flashAddr, uint32_t ramAddr, uint32_t *mem, uint32_t value)
{
    uint32_t base, where;

    switch (type) {

    case NANO_RELOC_TYPE_RAM:
        base = ramAddr;
        break;

    case NANO_RELOC_TYPE_FLASH:
        base = flashAddr;
        break;

    default:
        return false;
    }

    where = *ofstP + value;
    *ofstP = where + 1;

    mem[where] += base;

    return true;
}

static bool handleRelocs(const uint8_t *relStart, const uint8_t *relEnd, uint32_t flashStart, uint32_t ramStart, void *mem)
{
    uint32_t ofst = 0;
    uint32_t type = 0;

    while (relStart != relEnd) {

        uint32_t rel = *relStart++;

        if (rel <= MAX_8_BIT_NUM) {

            if (!handleRelNumber(&ofst, type, flashStart, ramStart, mem, rel))
                return false;
        }
        else switch (rel) {

        case TOKEN_32BIT_OFST:
            if (relEnd - relStart < 4)
                return false;
            rel = *(uint32_t*)relStart;
            relStart += sizeof(uint32_t);
            if (!handleRelNumber(&ofst, type, flashStart, ramStart, mem, rel))
                return false;
            break;

        case TOKEN_24BIT_OFST:
            if (relEnd - relStart < 3)
                return false;
            rel = *(uint16_t*)relStart;
            relStart += sizeof(uint16_t);
            rel += ((uint32_t)(*relStart++)) << 16;
            if (!handleRelNumber(&ofst, type, flashStart, ramStart, mem, rel + MAX_16_BIT_NUM))
                return false;
            break;

        case TOKEN_16BIT_OFST:
            if (relEnd - relStart < 2)
                return false;
            rel = *(uint16_t*)relStart;
            relStart += sizeof(uint16_t);
            if (!handleRelNumber(&ofst, type, flashStart, ramStart, mem, rel + MAX_8_BIT_NUM))
                return false;
            break;

        case TOKEN_CONSECUTIVE:
            if (relEnd - relStart < 1)
                return false;
            rel = *relStart++;
            rel += MIN_RUN_LEN;
            while (rel--)
                if (!handleRelNumber(&ofst, type, flashStart, ramStart, mem, 0))
                	return false;
            break;

        case TOKEN_RELOC_TYPE_CHG:
            if (relEnd - relStart < 1)
                return false;
            rel = *relStart++;
            rel++;
            type += rel;
            ofst = 0;
            break;

        case TOKEN_RELOC_TYPE_NEXT:
            type++;
            ofst = 0;
            break;
        }
    }

    return true;
}

bool cpuInternalAppLoad(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    platInfo->got = 0x00000000;

    return true;
}

bool cpuAppLoad(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    const uint8_t *relocsStart = (const uint8_t*)(((uint8_t*)appHdr) + appHdr->rel_start);
    const uint8_t *relocsEnd = (const uint8_t*)(((uint8_t*)appHdr) + appHdr->rel_end);
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
    if (!handleRelocs(relocsStart, relocsEnd, (uintptr_t)appHdr, (uintptr_t)mem, (void*)mem)) {
        osLog(LOG_ERROR, "Relocs are invalid in this app. Aborting app load\n");
        heapFree(mem);
        return false;
    }

    return true;
}

void cpuAppUnload(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo)
{
    if (platInfo->got)
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


