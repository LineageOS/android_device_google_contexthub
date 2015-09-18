#ifndef _CPU_H_
#define _CPU_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <seos.h>
#include <stdint.h>
#include <plat/inc/app.h>


void cpuInit(void);
void cpuInitLate(void);  //console guaranted to be up by now

uint64_t cpuIntsOff(void);
uint64_t cpuIntsOn(void);
void cpuIntsRestore(uint64_t state);

/* app loading, unloading & calling */
bool cpuInternalAppLoad(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo);
bool cpuAppLoad(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo);
void cpuAppUnload(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo);
bool cpuAppInit(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo, uint32_t tid);
void cpuAppEnd(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo);
void cpuAppHandle(const struct AppHdr *appHdr, struct PlatAppInfo *platInfo, uint32_t evtType, const void* evtData);

#ifdef __cplusplus
}
#endif

#endif

