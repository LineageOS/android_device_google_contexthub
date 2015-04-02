#ifndef _STM32F411_RTC_H_
#define _STM32F411_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <inc/seos.h>

#define RTC_ERR_TOO_BIG         -1
#define RTC_ERR_TOO_SMALL       -2
#define RTC_ERR_INTERNAL        -3
#define RTC_ERR_ACCURACY_UNMET  -4

void rtcInit(void);
int rtcSetWakeupTimer(uint64_t delay, int ppm);

#ifdef __cplusplus
}
#endif

#endif
