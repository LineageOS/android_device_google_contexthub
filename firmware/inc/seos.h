/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
#include <plat/inc/app.h>
#include <eventnums.h>



#define UNROLLED   __attribute__((optimize("unroll-loops")))


#define MAX_TASKS                        16
#define MAX_EMBEDDED_EVT_SUBS            6 /*tradeoff, no wrong answer */



#define OS_VER                           0x0000

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
#define APP_HDR_MARKER_INTERNAL    0xFF01 //no external app should at any point have this marker value!
#define APP_HDR_MARKER_DELETED     0x0000

/* app ids are split into vendor and app parts. vendor parts are assigned by google. App parts are free for each vendor to assign at will */
#define APP_ID_FIRST_USABLE        0x0100000000000000ULL //all app ids lower than this are reserved for google's internal use
#define APP_ID_GET_VENDOR(appid)   ((appid) >> 24)
#define APP_ID_MAKE(vendor, app)   ((((uint64_t)(vendor)) << 24) | ((app) & 0x00FFFFFF))
#define APP_ID_VENDOR_GOOGLE       0x476f6f676cULL // "Googl"

struct AppHdr {
    char magic[13];
    uint8_t fmtVer;  //app header format version
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

    uint32_t appVer; //version of actual app
    uint32_t rfu;

    struct AppFuncs funcs;
};

struct AppEventFreeData { //goes with EVT_APP_FREE_EVT_DATA
    uint32_t evtType;
    void* evtData;
};

typedef void (*OsDeferCbkF)(void *);

typedef void (*EventFreeF)(void* event);

struct SeosEedataEncrKeyData {
    uint64_t keyID;
    uint8_t key[32];
} __attribute__((packed));

/* ==== ABOUT THE "urgent" FLAG ====
 *
 * Do not set "urgent" unless you understand all the repercussions! What repercussions you might ask?
 * Setting this flag will place your defer request at the front of the queue. This is useful for enqueueing work
 * from interrupt context that needs to be done "very very soon"(tm). Doing this will delay all other work requests
 * that have heretofore been peacefully queueing in full faith and with complete belief in fairness of our "FIFO"-ness.
 * Please be appreciative of this fact and do not abuse this! Example: if you are setting "urgent" flag outside of interrupt
 * context, you're very very likely wrong. That is not to say that being in interrupt context is a free pass to set this!
 */

// osMainInit is exposed for testing only, it must never be called for any reason at all by anyone
void osMainInit(void);
// osMainDequeueLoop is exposed for testing only, it must never be called for any reason at all by anyone
void osMainDequeueLoop(void);
void osMain(void);

bool osEventSubscribe(uint32_t tid, uint32_t evtType); /* async */
bool osEventUnsubscribe(uint32_t tid, uint32_t evtType);  /* async */

bool osEnqueuePrivateEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, uint32_t toTid);
bool osEnqueuePrivateEvtAsApp(uint32_t evtType, void *evtData, uint32_t fromApp, uint32_t toTid);

bool osEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF);
bool osEnqueueEvtAsApp(uint32_t evtType, void *evtData, uint32_t fromApp);

bool osDefer(OsDeferCbkF callback, void *cookie, bool urgent);

bool osTidById(uint64_t appId, uint32_t *tid);
bool osAppInfoById(uint64_t appId, uint32_t *appIdx, uint32_t *appVer, uint32_t *appSize);
bool osAppInfoByIndex(uint32_t appIdx, uint64_t *appId, uint32_t *appVer, uint32_t *appSize);

//event retaining support
bool osRetainCurrentEvent(TaggedPtr *evtFreeingInfoP); //called from any apps' event handling to retain current event. Only valid for first app that tries. evtFreeingInfoP filled by call and used to free evt later
void osFreeRetainedEvent(uint32_t evtType, void *evtData, TaggedPtr *evtFreeingInfoP);


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

#ifndef INTERNAL_APP_INIT
#define INTERNAL_APP_INIT(_id, _ver, _init, _end, _event)                                   \
static const struct AppHdr __attribute__((used,section (".internal_app_init"))) mAppHdr = { \
    .magic = APP_HDR_MAGIC,                                                                 \
    .fmtVer = APP_HDR_VER_CUR,                                                              \
    .marker = APP_HDR_MARKER_INTERNAL,                                                      \
    .appId = (_id),                                                                         \
    .appVer = (_ver),                                                                       \
    .funcs.init = (_init),                                                                  \
    .funcs.end = (_end),                                                                    \
    .funcs.handle = (_event)                                                                \
}
#endif

#ifndef APP_INIT
#define APP_INIT(_ver, _init, _end, _event)                                            \
extern const struct AppFuncs _mAppFuncs;                                         \
const struct AppFuncs __attribute__((used,section (".app_init"),visibility("default"))) _mAppFuncs = { \
    .init = (_init),                                                             \
    .end = (_end),                                                               \
    .handle = (_event)                                                           \
};                                                                                \
const uint32_t __attribute__((used,section (".app_version"),visibility("default"))) _mAppVer = _ver
#endif


#ifdef __cplusplus
}
#endif

#endif
