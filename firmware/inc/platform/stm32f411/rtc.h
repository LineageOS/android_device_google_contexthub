#ifndef _STM32F411_RTC_H_
#define _STM32F411_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <inc/seos.h>

void rtcInit(void);
int rtcSetWakeupTimer(struct nanotime_t delay, int ppm);

#ifdef __cplusplus
}
#endif

#endif
