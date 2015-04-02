#ifndef _SEOS_H_
#define _SEOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "eventQ.h"



#define MAX_TASKS                        16
#define MAX_EMBEDDED_EVT_SUBS            6 /*tradeoff, no wrong answer */

#define EVT_NO_FIRST_USER_EVENT          0x00000100    //all events lower than this are reserved for the OS. all of them are nondiscardable necessarily!


struct AppEntry {
    /* lifescycle */
    void (*start)(uint32_t yourTid);
    void (*end)(void);
    /* events */
    void (*handle)(uint32_t evtType, const void* evtData);
};


void osMain(void);
bool osEventSubscribe(uint32_t tid, uint32_t evtType); /* async */
bool osEventUnsubscribe(uint32_t tid, uint32_t evtType);  /* async */
bool osEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external);

/* Logging */
enum LogLevel {
    LOG_ERROR = 'E',
    LOG_WARN = 'W',
    LOG_INFO = 'I',
    LOG_DEBUG = 'D',
};

void osLog(enum LogLevel level, const char *str, ...);

#define APP_INIT(_start, _end, _event) \
static const struct AppEntry __attribute__((used,section (".app_init"))) mAppEntry = {\
    .start = (_start),\
    .end = (_end),\
    .handle = (_event)\
}

#ifdef __cplusplus
}
#endif

#endif
