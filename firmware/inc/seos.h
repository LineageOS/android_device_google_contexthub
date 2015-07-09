#ifndef _SEOS_H_
#define _SEOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "eventQ.h"



#define MAX_TASKS                        16
#define MAX_EMBEDDED_EVT_SUBS            6 /*tradeoff, no wrong answer */

#define EVT_NO_FIRST_USER_EVENT          0x00000100    //all events lower than this are reserved for the OS. all of them are nondiscardable necessarily!
#define EVT_NO_FIRST_SENSOR_EVENT        0x00000200    //sensor type SENSOR_TYPE_x produces events of type EVT_NO_FIRST_SENSOR_EVENT + SENSOR_TYPE_x for all Google-defined sensors
#define EVT_BOOT_COMPLETED               0x00000300    //sent when boot completes

#define OS_VER                           0x0000

struct AppEntry { /* do not rearrange */
    /* lifescycle */
    void (*start)(uint32_t yourTid);
    void (*end)(void);
    /* events */
    void (*handle)(uint32_t evtType, const void* evtData);
};

typedef void (*OsDeferCbkF)(void *);

void osMain(void);
bool osEventSubscribe(uint32_t tid, uint32_t evtType); /* async */
bool osEventUnsubscribe(uint32_t tid, uint32_t evtType);  /* async */
bool osEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external);
bool osDefer(OsDeferCbkF callback, void *cookie);

/* Logging */
enum LogLevel {
    LOG_ERROR = 'E',
    LOG_WARN = 'W',
    LOG_INFO = 'I',
    LOG_DEBUG = 'D',
};

void osLogv(enum LogLevel level, const char *str, va_list vl);
void osLog(enum LogLevel level, const char *str, ...)
    __attribute__((format(printf, 2, 3)));

#define APP_INIT(_start, _end, _event)                                           \
extern const struct AppEntry _mAppEntry;                                         \
const struct AppEntry __attribute__((used,section (".app_init"))) _mAppEntry = { \
    .start = (_start),                                                           \
    .end = (_end),                                                               \
    .handle = (_event)                                                           \
}




//EXTERNAL API
//level 1 indices in the OS table
#define SYSCALL_OS_MAIN                   0
#define SYSCALL_OS_LAST                   1 // always last. holes are allowed, but nto immediately before this

//level 2 indices in the OS.main table
#define SYSCALL_OS_MAIN_EVENTQ            0
#define SYSCALL_OS_MAIN_LOGGING           1
#define SYSCALL_OS_MAIN_LAST              2 // always last. holes are allowed, but nto immediately before this

//level 3 indices in the OS.main.event_queue table
#define SYSCALL_OS_MAIN_EVTQ_SUBCRIBE    0 // (uint32_t tid, uint32_t evtType) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_UNSUBCRIBE  1 // ((uint32_t tid, uint32_t evtType) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_ENQUEUE     2 // (uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_FUNC_DEFER  3 // (OsDeferCbkF callback, void *data) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_LAST        4 // always last. holes are allowed, but nto immediately before this

//level 3 indices in the OS.main.logging table
#define SYSCALL_OS_MAIN_LOG_LOG          0 // (enum LogLevel level, const char *str, ...) -> void
#define SYSCALL_OS_MAIN_LOG_LAST         1 // always last. holes are allowed, but nto immediately before this


#ifdef __cplusplus
}
#endif

#endif
