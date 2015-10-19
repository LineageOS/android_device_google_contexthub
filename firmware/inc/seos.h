#ifndef _SEOS_H_
#define _SEOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <plat/inc/taggedPtr.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <eventQ.h>
#include <aes.h>
#include <plat/inc/app.h>
#include <eventnums.h>



#define MAX_TASKS                        16
#define MAX_EMBEDDED_EVT_SUBS            6 /*tradeoff, no wrong answer */



#define OS_VER                           0x0000

#define EE_DATA_TYPE_ENCR_KEY            1

#define ENCR_KEY_GOOGLE_PREPOPULATED     1 // our key ID is 1

#define FIRST_VALID_TID                  0x00000001
#define LAST_VALID_TID                   0x0fffffff


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
#define APP_HDR_MARKER_INTERNAL    0xFF01
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

struct AppEventFreeData { //goes with EVT_APP_FREE_EVT_DATA
    uint32_t evtType;
    void* evtData;
};

typedef void (*OsDeferCbkF)(void *);

typedef void (*EventFreeF)(void* event);

void osMain(void);
bool osEventSubscribe(uint32_t tid, uint32_t evtType); /* async */
bool osEventUnsubscribe(uint32_t tid, uint32_t evtType);  /* async */

bool osEnqueuePrivateEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, uint32_t toTid);
bool osEnqueuePrivateEvtAsApp(uint32_t evtType, void *evtData, uint32_t fromApp, uint32_t toTid);

bool osEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external);
bool osEnqueueEvtAsApp(uint32_t evtType, void *evtData, uint32_t fromApp, bool external);

bool osDequeueExtEvt(uint32_t *evtType, void **evtData, TaggedPtr *evtFreeInfoP); // THIS FUNCTION VIOLATES MANY THINGS, IT WILL GO AWAY SOON, fo rnow it just gets weird "free info" data till it runs out of memory
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

#define INTERNAL_APP_INIT(_id, _init, _end, _event)                                         \
static const struct AppHdr __attribute__((used,section (".internal_app_init"))) mAppHdr = { \
    .magic = APP_HDR_MAGIC,                                                                 \
    .version = APP_HDR_VER_CUR,                                                             \
    .marker = APP_HDR_MARKER_INTERNAL,                                                      \
    .appId = (_id),                                                                         \
    .funcs.init = (_init),                                                                  \
    .funcs.end = (_end),                                                                    \
    .funcs.handle = (_event)                                                                \
}

#define APP_INIT(_init, _end, _event)                                            \
extern const struct AppFuncs _mAppFuncs;                                         \
const struct AppFuncs __attribute__((used,section (".app_init"),visibility("default"))) _mAppFuncs = { \
    .init = (_init),                                                             \
    .end = (_end),                                                               \
    .handle = (_event)                                                           \
}




//EXTERNAL API
//level 1 indices in the OS table
#define SYSCALL_OS_MAIN                   0
#define SYSCALL_OS_LAST                   1 // always last. holes are allowed, but not immediately before this

//level 2 indices in the OS.main table
#define SYSCALL_OS_MAIN_EVENTQ            0
#define SYSCALL_OS_MAIN_LOGGING           1
#define SYSCALL_OS_MAIN_SENSOR            2
#define SYSCALL_OS_MAIN_LAST              3 // always last. holes are allowed, but not immediately before this

//level 3 indices in the OS.main.event_queue table
#define SYSCALL_OS_MAIN_EVTQ_SUBCRIBE    0 // (uint32_t tid, uint32_t evtType) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_UNSUBCRIBE  1 // ((uint32_t tid, uint32_t evtType) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_ENQUEUE     2 // (uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external) -> bool success
#define SYSCALL_OS_MAIN_EVTQ_LAST        3 // always last. holes are allowed, but not immediately before this

//level 3 indices in the OS.main.logging table
#define SYSCALL_OS_MAIN_LOG_LOGV         0 // (enum LogLevel level, const char *str, va_list *) -> void
#define SYSCALL_OS_MAIN_LOG_LAST         1 // always last. holes are allowed, but not immediately before this

//level 3 indices in the OS.main.sensors table
#define SYSCALL_OS_MAIN_SENSOR_SIGNAL    0 // (uint32_t handle, uint32_t intEvtNum, uint32_t value1, uint32_t value2_lo, uint32_t value2_hi) -> bool /* used by sensor-provding apps */
#define SYSCALL_OS_MAIN_SENSOR_FIND      1 // (uint32_t sensorType, uint32_t idx, uint32_t *handleP) -> const struct SensorInfo* or NULL
#define SYSCALL_OS_MAIN_SENSOR_REQUEST   2 // (uint32_t clientId, uint32_t sensorHandle, uint32_t rate) -> bool success
#define SYSCALL_OS_MAIN_SENSOR_RATE_CHG  3 // (uint32_t clientId, uint32_t sensorHandle, uint32_t newRate) -> bool success
#define SYSCALL_OS_MAIN_SENSOR_RELEASE   4 // (uint32_t clientId, uint32_t sensorHandle) -> bool success
#define SYSCALL_OS_MAIN_SENSOR_TRIGGER   5 // (uint32_t clientId, uint32_t sensorHandle) -> bool success
#define SYSCALL_OS_MAIN_SENSOR_GET_RATE  6 // (uint32_t sensorHandle) -> uint32_t rate
#define SYSCALL_OS_MAIN_SENSOR_LAST      7 // always last. holes are allowed, but not immediately before this



#ifdef __cplusplus
}
#endif

#endif
