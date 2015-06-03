#ifndef _CPU_H_
#define _CPU_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


void cpuInit(void);

uint64_t cpuIntsOff(void);
uint64_t cpuIntsOn(void);
void cpuIntsRestore(uint64_t state);


#ifdef __cplusplus
}
#endif

#endif

