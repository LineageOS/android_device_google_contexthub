#ifndef _SEOS_H_
#define _SEOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <eventQ.h>
#include <plat/inc/app.h>



#define MAX_TASKS                        16
#define MAX_EMBEDDED_EVT_SUBS            6 /*tradeoff, no wrong answer */

#define EVT_NO_FIRST_USER_EVENT          0x00000100    //all events lower than this are reserved for the OS. all of them are nondiscardable necessarily!
#define EVT_NO_FIRST_SENSOR_EVENT        0x00000200    //sensor type SENSOR_TYPE_x produces events of type EVT_NO_FIRST_SENSOR_EVENT + SENSOR_TYPE_x for all Google-defined sensors
#define EVT_APP_START                    0x00000300    //sent when an app can actually start

#define OS_VER                           0x0000

struct AppFuncs { /* do not rearrange */
    /* lifescycle */
    bool (*init)(uint32_t yourTid);   //simple init only - no ints on at this time
    void (*end)(void);                //die quickly please
    /* events */
    void (*handle)(uint32_t evtType, const void* evtData);
};

#define APP_HDR_MAGIC              "GoogleNanoApp"
#define APP_HDR_VER_CUR            0
#define APP_HDR_MARKER_UPLOADING   0xFFFF
#define APP_HDR_MARKER_VERIFYING   0xFFFE
#define APP_HDR_MARKER_VALID       0xFF00
#define APP_HDR_MARKER_DELETED     0x0000


struct AppHdr {
    char magic[13];
    uint8_t version;
    uint16_t marker;

    uint64_t appId;

    uint32_t data_start;
    uint32_t data_end;
    uint32_t data_data;

    uint32_t bss_start;
    uint32_t bss_end;

    uint32_t got_start;
    uint32_t got_end;
    uint32_t rel_start;
    uint32_t rel_end;

    struct AppFuncs funcs;
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

#define APP_INIT(_init, _end, _event)                                            \
extern const struct AppFuncs _mAppFuncs;                                         \
const struct AppFuncs __attribute__((used,section (".app_init"))) _mAppFuncs = { \
    .init = (_init),                                                             \
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
#define SYSCALL_OS_MAIN_LOG_LOGV         0 // (enum LogLevel level, const char *str, va_list *) -> void
#define SYSCALL_OS_MAIN_LOG_LAST         1 // always last. holes are allowed, but nto immediately before this


#ifdef __cplusplus
}
#endif

#endif
