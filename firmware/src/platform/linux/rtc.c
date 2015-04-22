#include <cpu/inc/barrier.h>
#include <plat/inc/rtc.h>
#include <inc/timer.h>
#include <inc/platform.h>


void rtcInit(void)
{
    /* nope */
}

/* Set calendar alarm to go off after delay has expired. uint64_t delay must
 * be in valid uint64_t format and must be less than 32 s.  A negative value
 * for the 'ppm' param indicates the alarm has no accuracy requirements. */
int rtcSetWakeupTimer(uint64_t delay, int ppm)
{
    //TODO

    return 0;
}

