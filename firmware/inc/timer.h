#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>


#define MAX_TIMERS	8	/* we *REALLY* do not want these proliferating endlessly, hence a small limit */

struct TimerEvent {
    uint32_t timerId;
    void *data;
};


typedef void (*TimTimerCbkF)(uint32_t timerId, void* data);



uint64_t timGetTime(void);   /* Time since some stable reference point in nanoseconds */

uint32_t timTimerSet(uint64_t length, uint32_t jitterPpm, uint32_t driftPpm, TimTimerCbkF cbk, void* data, bool oneShot); /* return timer id or 0 if failed */
uint32_t timTimerSetAsApp(uint64_t length, uint32_t jitterPpm, uint32_t driftPpm, uint32_t tid, void* data, bool oneShot); /* return timer id or 0 if failed */
bool timTimerCancel(uint32_t timerId);


//called by interrupt routine. ->true if any timers were fired
bool timIntHandler(void);


//init subsystem
void timInit(void);




#ifdef __cplusplus
}
#endif

#endif

