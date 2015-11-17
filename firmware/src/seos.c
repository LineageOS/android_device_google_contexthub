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

#include <plat/inc/plat.h>
#include <plat/inc/bl.h>
#include <platform.h>
#include <hostIntf.h>
#include <syscall.h>
#include <sensors.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <printf.h>
#include <eventQ.h>
#include <apInt.h>
#include <timer.h>
#include <osApi.h>
#include <seos.h>
#include <heap.h>
#include <slab.h>
#include <cpu.h>
#include <crc.h>


/*
 * Since locking is difficult to do right for adding/removing listeners and such
 * since it can happen in interrupt context and not, and one such operation can
 * interrupt another, and we do have a working event queue, we enqueue all the
 * requests and then deal with them in the main code only when the event bubbles
 * up to the front of the quque. This allows us to not need locks around the
 * data structures.
 */

struct Task {
    /* pointers may become invalid. Tids do not. Zero tid -> not a valid task */
    uint32_t tid;

    uint16_t subbedEvtCount;
    uint16_t subbedEvtListSz;
    uint32_t *subbedEvents; /* NULL for invalid tasks */

    /* App entry points */
    const struct AppHdr *appHdr;

    /* per-platform app info */
    struct PlatAppInfo platInfo;

    /* for some basic number of subbed events, the array is stored directly here. after that, a heap chunk is used */
    uint32_t subbedEventsInt[MAX_EMBEDDED_EVT_SUBS];
};

union InternalThing {
    struct {
        uint32_t tid;
        uint32_t evt;
    } evtSub;
    struct {
        OsDeferCbkF callback;
        void *cookie;
    } deferred;
    struct {
        uint32_t evtType;
        void *evtData;
        TaggedPtr evtFreeInfo;
        uint32_t toTid;
    } privateEvt;
    union OsApiSlabItem osApiItem;
};

#define EVT_SUBSCRIBE_TO_EVT         0x00000000
#define EVT_UNSUBSCRIBE_TO_EVT       0x00000001
#define EVT_DEFERRED_CALLBACK        0x00000002
#define EVT_PRIVATE_EVT              0x00000003


static struct EvtQueue *mEvtsInternal;
static struct SlabAllocator* mMiscInternalThingsSlab;
static struct Task mTasks[MAX_TASKS];
static uint32_t mNextTidInfo = FIRST_VALID_TID;

static struct Task* osTaskFindByTid(uint32_t tid)
{
    uint32_t i;

    for(i = 0; i < MAX_TASKS; i++)
        if (mTasks[i].tid && mTasks[i].tid == tid)
            return mTasks + i;

    return NULL;
}

static void handleEventFreeing(uint32_t evtType, void *evtData, uintptr_t evtFreeData) // watch out, this is synchronous
{
    if ((taggedPtrIsPtr(evtFreeData) && !taggedPtrToPtr(evtFreeData)) ||
        (taggedPtrIsUint(evtFreeData) && !taggedPtrToUint(evtFreeData)))
        return;

    if (taggedPtrIsPtr(evtFreeData))
        ((EventFreeF)taggedPtrToPtr(evtFreeData))(evtData);
    else {
        struct AppEventFreeData fd = {evtType: evtType, evtData: evtData};
        struct Task* task = osTaskFindByTid(taggedPtrToUint(evtFreeData));

        if (!task)
            osLog(LOG_ERROR, "EINCEPTION: Failed to find app to call app to free event sent to app(s).\n");
        else
            cpuAppHandle(task->appHdr, &task->platInfo, EVT_APP_FREE_EVT_DATA, &fd);
    }
}

static void osInit(void)
{
    cpuInit();
    heapInit();
    platInitialize();

    osLog(LOG_INFO, "SEOS Initializing\n");
    cpuInitLate();

    /* init task list */
    memset(mTasks, 0, sizeof(mTasks));

    /* create the queues */
    if (!(mEvtsInternal = evtQueueAlloc(512, handleEventFreeing))) {
        osLog(LOG_INFO, "events failed to init\n");
        return;
    }

    mMiscInternalThingsSlab = slabAllocatorNew(sizeof(union InternalThing), 4, 64 /* for now? */);
    if (!mMiscInternalThingsSlab) {
        osLog(LOG_INFO, "deferred actions list failed to init\n");
        return;
    }
}

static struct Task* osTaskFindByAppID(uint64_t appID)
{
    uint32_t i;

    for (i = 0; i < MAX_TASKS; i++)
        if (mTasks[i].appHdr && mTasks[i].appHdr->appId == appID)
            return mTasks + i;

    return NULL;
}

static uint32_t osGetFreeTid(void)
{
    do {
        if (mNextTidInfo == LAST_VALID_TID)
            mNextTidInfo = FIRST_VALID_TID;
        else
            mNextTidInfo++;
    } while (osTaskFindByTid(mNextTidInfo));

    return mNextTidInfo;
}

static void osStartTasks(void)
{
    extern char __shared_start[];
    extern char __shared_end[];
    extern const struct AppHdr __internal_app_start, __internal_app_end;
    static const char magic[] = APP_HDR_MAGIC;
    const struct AppHdr *app;
    uint32_t i, nTasks = 0;
    struct Task* task;
    uint8_t *shared_start = (uint8_t *)&__shared_start;
    uint8_t *shared_end = (uint8_t *)&__shared_end;
    uint8_t *shared;
    int len, total_len;
    uint8_t id1, id2;

    /* first enum all internal apps, making sure to check for dupes */
    osLog(LOG_DEBUG, "Reading internal app list...\n");
    for (app = &__internal_app_start; app != &__internal_app_end && nTasks < MAX_TASKS  && app->fmtVer == APP_HDR_VER_CUR; app++) {

        if (app->marker != APP_HDR_MARKER_INTERNAL) {
            osLog(LOG_WARN, "Weird marker on internal app: [%p]=0x%04X\n", app, app->marker);
            continue;
        }
        if ((task = osTaskFindByAppID(app->appId))) {
            osLog(LOG_WARN, "Internal app id %016llx @ %p attempting to update internal app @ %p. Ignored.\n", app->appId, app, task->appHdr);
            continue;
        }
        mTasks[nTasks++].appHdr = app;
    }

    /* then enum all external apps, making sure to find the latest (by position in flash) and checking for conflicts with internal apps */
    osLog(LOG_DEBUG, "Reading external app list...\n");
    for (shared = shared_start;
         shared < shared_end && shared[0] != 0xFF;
         shared += total_len) {
        id1 = shared[0] & 0x0F;
        id2 = (shared[0] >> 4) & 0x0F;
        len = (shared[1] << 16) | (shared[2] << 8) | shared[3];
        total_len = sizeof(uint32_t) + ((len + 3) & ~3) + sizeof(uint32_t);

        if (shared + total_len > shared_end)
            break;

        //skip over erased sections
        if (id1 != id2 || id1 != BL_FLASH_APP_ID)
            continue;

        if (crc32(shared, total_len, ~0) == CRC_RESIDUE) {
            app = (const struct AppHdr *)&shared[4];
            if (len >= sizeof(struct AppHdr) && !memcmp(magic, app->magic, sizeof(magic) - 1) && app->fmtVer == APP_HDR_VER_CUR) {

                if (app->marker != APP_HDR_MARKER_VALID)  //this may need more logic to handle partially-uploaded things
                    osLog(LOG_WARN, "Weird marker on external app: [%p]=0x%04X\n", app, app->marker);
                else if ((task = osTaskFindByAppID(app->appId))) {
                    if (task->appHdr->marker == APP_HDR_MARKER_INTERNAL)
                        osLog(LOG_WARN, "External app id %016llx @ %p attempting to update internal app @ %p. This is not allowed.\n", app->appId, app, task->appHdr);
                    else {
                        osLog(LOG_DEBUG, "External app id %016llx @ %p updating app @ %p\n", app->appId, app, task->appHdr);
                        task->appHdr = app;
                    }
                }
                else if (nTasks == MAX_TASKS)
                    osLog(LOG_WARN, "External app id %016llx @ %p cannot be used as too many apps already exist.\n", app->appId, app);
                else
                    mTasks[nTasks++].appHdr = app;
            }
        }
    }

    osLog(LOG_DEBUG, "Enumerated %lu apps\n", nTasks);

    /* Now that we have pointers to all the latest app headers, let's try loading then. */
    /* Note that if a new version fails to init we will NOT try the old (no reason to assume this is safe) */
    osLog(LOG_DEBUG, "Loading apps...\n");
    for (i = 0; i < nTasks;) {

        if (mTasks[i].appHdr->marker == APP_HDR_MARKER_INTERNAL) {
            if (cpuInternalAppLoad(mTasks[i].appHdr, &mTasks[i].platInfo)) {
                i++;
                continue;
            }
        }
        else {
            if (cpuAppLoad(mTasks[i].appHdr, &mTasks[i].platInfo)) {
                i++;
                continue;
            }
        }

        //if we're here, an app failed to load - remove it from the list
        osLog(LOG_WARN, "App @ %p failed to load\n", mTasks[i].appHdr);
        memcpy(mTasks + i, mTasks + --nTasks, sizeof(struct Task));
    }

    osLog(LOG_DEBUG, "Loaded %lu apps\n", nTasks);

    /* now finish initing structs, assign tids, call init funcs */
    osLog(LOG_DEBUG, "Starting apps...\n");
    for (i = 0; i < nTasks;) {

        mTasks[i].subbedEvtListSz = MAX_EMBEDDED_EVT_SUBS;
        mTasks[i].subbedEvents = mTasks[i].subbedEventsInt;
        mTasks[i].tid = osGetFreeTid();

        if (cpuAppInit(mTasks[i].appHdr, &mTasks[i].platInfo, mTasks[i].tid))
            i++;
        else {
            //if we're here, an app failed to init - unload & remove it from the list
            osLog(LOG_WARN, "App @ %p failed to init\n", mTasks[i].appHdr);
            cpuAppUnload(mTasks[i].appHdr, &mTasks[i].platInfo);
            memcpy(mTasks + i, mTasks + --nTasks, sizeof(struct Task));
        }
    }

    osLog(LOG_DEBUG, "Started %lu apps\n", nTasks);
}

static void osInternalEvtHandle(uint32_t evtType, void *evtData)
{
    union InternalThing *da = (union InternalThing*)evtData;
    struct Task *task;
    uint32_t i;

    switch (evtType) {
    case EVT_SUBSCRIBE_TO_EVT:
    case EVT_UNSUBSCRIBE_TO_EVT:
        /* get task */
        task = osTaskFindByTid(da->evtSub.tid);
        if (!task)
            break;

        /* find if subscribed to this evt */
        for (i = 0; i < task->subbedEvtCount && task->subbedEvents[i] != da->evtSub.evt; i++);

        /* if unsub & found -> unsub */
        if (evtType == EVT_UNSUBSCRIBE_TO_EVT && i != task->subbedEvtCount)
            task->subbedEvents[i] = task->subbedEvents[--task->subbedEvtCount];
        /* if sub & not found -> sub */
        else if (evtType == EVT_SUBSCRIBE_TO_EVT && i == task->subbedEvtCount) {
            if (task->subbedEvtListSz == task->subbedEvtCount) { /* enlarge the list */
                uint32_t newSz = (task->subbedEvtListSz * 3 + 1) / 2;
                uint32_t *newList = heapAlloc(sizeof(uint32_t[newSz])); /* grow by 50% */
                if (newList) {
                    memcpy(newList, task->subbedEvents, sizeof(uint32_t[task->subbedEvtListSz]));
                    if (task->subbedEvents != task->subbedEventsInt)
                        heapFree(task->subbedEvents);
                    task->subbedEvents = newList;
                    task->subbedEvtListSz = newSz;
                }
            }
            if (task->subbedEvtListSz > task->subbedEvtCount) { /* have space ? */
                task->subbedEvents[task->subbedEvtCount++] = da->evtSub.evt;
            }
        }
        break;

    case EVT_DEFERRED_CALLBACK:
        da->deferred.callback(da->deferred.cookie);
        break;

    case EVT_PRIVATE_EVT:
        task = osTaskFindByTid(da->privateEvt.toTid);
        if (task) {
            cpuAppHandle(task->appHdr, &task->platInfo, da->privateEvt.evtType, da->privateEvt.evtData);
        }

        handleEventFreeing(da->privateEvt.evtType, da->privateEvt.evtData, da->privateEvt.evtFreeInfo);
        break;
    }
}

void abort(void)
{
    /* this is necessary for va_* funcs... */
    osLog(LOG_ERROR, "Abort called");
    while(1);
}

void __attribute__((noreturn)) osMain(void)
{
    TaggedPtr evtFreeingInfo;
    uint32_t evtType, i, j;
    void *evtData;

    cpuIntsOff();
    osInit();
    timInit();
    sensorsInit();
    syscallInit();
    osApiExport(mMiscInternalThingsSlab);
    apIntInit();
    cpuIntsOn();
    osStartTasks();

    //broadcast app start to all already-loaded apps
    (void)osEnqueueEvt(EVT_APP_START, NULL, NULL);

    while (true) {

        /* get an event */
        if (!evtQueueDequeue(mEvtsInternal, &evtType, &evtData, &evtFreeingInfo, true))
            continue;

        if (evtType < EVT_NO_FIRST_USER_EVENT) { /* no need for discardable check. all internal events arent discardable */
            /* handle deferred actions and other reserved events here */
            osInternalEvtHandle(evtType, evtData);
        }
        else {
            /* send this event to all tasks who want it (decimation could happen here) */
            for (i = 0; i < MAX_TASKS; i++) {
                if (!mTasks[i].subbedEvents) /* only check real tasks */
                    continue;
                for (j = 0; j < mTasks[i].subbedEvtCount; j++) {
                    if (mTasks[i].subbedEvents[j] == (evtType & ~EVENT_TYPE_BIT_DISCARDABLE)) {
                        cpuAppHandle(mTasks[i].appHdr, &mTasks[i].platInfo, evtType & ~EVENT_TYPE_BIT_DISCARDABLE, evtData);
                        break;
                    }
                }
            }
        }

        /* free it */
        handleEventFreeing(evtType, evtData, evtFreeingInfo);
    }
}

static void osDeferredActionFreeF(void* event)
{
    slabAllocatorFree(mMiscInternalThingsSlab, event);
}

static bool osEventSubscribeUnsubscribe(uint32_t tid, uint32_t evtType, bool sub)
{
    union InternalThing *act = slabAllocatorAlloc(mMiscInternalThingsSlab);

    if (!act)
        return false;
    act->evtSub.evt = evtType;
    act->evtSub.tid = tid;

    if (osEnqueueEvt(sub ? EVT_SUBSCRIBE_TO_EVT : EVT_UNSUBSCRIBE_TO_EVT, act, osDeferredActionFreeF))
        return true;

    slabAllocatorFree(mMiscInternalThingsSlab, act);
    return false;
}

bool osEventSubscribe(uint32_t tid, uint32_t evtType)
{
    return osEventSubscribeUnsubscribe(tid, evtType, true);
}

bool osEventUnsubscribe(uint32_t tid, uint32_t evtType)
{
    return osEventSubscribeUnsubscribe(tid, evtType, false);
}

bool osEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF)
{
    return evtQueueEnqueue(mEvtsInternal, evtType, evtData, taggedPtrMakeFromPtr(evtFreeF), false);
}

bool osEnqueueEvtAsApp(uint32_t evtType, void *evtData, uint32_t fromAppTid)
{
    return evtQueueEnqueue(mEvtsInternal, evtType, evtData, taggedPtrMakeFromUint(fromAppTid), false);
}

bool osDefer(OsDeferCbkF callback, void *cookie, bool urgent)
{
    union InternalThing *act = slabAllocatorAlloc(mMiscInternalThingsSlab);
    if (!act)
            return false;

    act->deferred.callback = callback;
    act->deferred.cookie = cookie;

    if (evtQueueEnqueue(mEvtsInternal, EVT_DEFERRED_CALLBACK, act, taggedPtrMakeFromPtr(osDeferredActionFreeF), urgent))
        return true;

    slabAllocatorFree(mMiscInternalThingsSlab, act);
    return false;
}

static bool osEnqueuePrivateEvtEx(uint32_t evtType, void *evtData, TaggedPtr evtFreeInfo, uint32_t toTid)
{
    union InternalThing *act = slabAllocatorAlloc(mMiscInternalThingsSlab);
    if (!act)
            return false;

    act->privateEvt.evtType = evtType;
    act->privateEvt.evtData = evtData;
    act->privateEvt.evtFreeInfo = evtFreeInfo;
    act->privateEvt.toTid = toTid;

    if (osEnqueueEvt(EVT_PRIVATE_EVT, act, osDeferredActionFreeF))
        return true;

    slabAllocatorFree(mMiscInternalThingsSlab, act);
    return false;
}

bool osEnqueuePrivateEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, uint32_t toTid)
{
    return osEnqueuePrivateEvtEx(evtType, evtData, taggedPtrMakeFromPtr(evtFreeF), toTid);
}

bool osEnqueuePrivateEvtAsApp(uint32_t evtType, void *evtData, uint32_t fromAppTid, uint32_t toTid)
{
    return osEnqueuePrivateEvtEx(evtType, evtData, taggedPtrMakeFromUint(fromAppTid), toTid);
}

bool osAppInfoById(uint64_t appId, uint32_t *appIdx, uint32_t *appVer, uint32_t *appSize)
{
    uint32_t i;

    for (i = 0; i < MAX_TASKS; i++) {
        if (mTasks[i].appHdr && mTasks[i].appHdr->appId == appId) {
            *appIdx = i;
            *appVer = mTasks[i].appHdr->appVer;
            *appSize = mTasks[i].appHdr->rel_end;
            return true;
        }
    }

    return false;
}

bool osAppInfoByIndex(uint32_t appIdx, uint64_t *appId, uint32_t *appVer, uint32_t *appSize)
{
    if (appIdx < MAX_TASKS && mTasks[appIdx].appHdr) {
        *appId = mTasks[appIdx].appHdr->appId;
        *appVer = mTasks[appIdx].appHdr->appVer;
        *appSize = mTasks[appIdx].appHdr->rel_end;
        return true;
    }

    return false;
}

void osLogv(enum LogLevel level, const char *str, va_list vl)
{
    void *userData = platLogAllocUserData();

    platLogPutcharF(userData, level);
    cvprintf(platLogPutcharF, userData, str, vl);

    platLogFlush(userData);
}

void osLog(enum LogLevel level, const char *str, ...)
{
    va_list vl;

    va_start(vl, str);
    osLogv(level, str, vl);
    va_end(vl);
}




//Google's public key for Google's apps' signing
const uint8_t __attribute__ ((section (".pubkeys"))) _RSA_KEY_GOOGLE[] = {
    0xd9, 0xcd, 0x83, 0xae, 0xb5, 0x9e, 0xe4, 0x63, 0xf1, 0x4c, 0x26, 0x6a, 0x1c, 0xeb, 0x4c, 0x12,
    0x5b, 0xa6, 0x71, 0x7f, 0xa2, 0x4e, 0x7b, 0xa2, 0xee, 0x02, 0x86, 0xfc, 0x0d, 0x31, 0x26, 0x74,
    0x1e, 0x9c, 0x41, 0x43, 0xba, 0x16, 0xe9, 0x23, 0x4d, 0xfc, 0xc4, 0xca, 0xcc, 0xd5, 0x27, 0x2f,
    0x16, 0x4c, 0xe2, 0x85, 0x39, 0xb3, 0x0b, 0xcb, 0x73, 0xb6, 0x56, 0xc2, 0x98, 0x83, 0xf6, 0xfa,
    0x7a, 0x6e, 0xa0, 0x9a, 0xcc, 0x83, 0x97, 0x9d, 0xde, 0x89, 0xb2, 0xa3, 0x05, 0x46, 0x0c, 0x12,
    0xae, 0x01, 0xf8, 0x0c, 0xf5, 0x39, 0x32, 0xe5, 0x94, 0xb9, 0xa0, 0x8f, 0x19, 0xe4, 0x39, 0x54,
    0xad, 0xdb, 0x81, 0x60, 0x74, 0x63, 0xd5, 0x80, 0x3b, 0xd2, 0x88, 0xf4, 0xcb, 0x6b, 0x47, 0x28,
    0x80, 0xb0, 0xd1, 0x89, 0x6d, 0xd9, 0x62, 0x88, 0x81, 0xd6, 0xc0, 0x13, 0x88, 0x91, 0xfb, 0x7d,
    0xa3, 0x7f, 0xa5, 0x40, 0x12, 0xfb, 0x77, 0x77, 0x4c, 0x98, 0xe4, 0xd3, 0x62, 0x39, 0xcc, 0x63,
    0x34, 0x76, 0xb9, 0x12, 0x67, 0xfe, 0x83, 0x23, 0x5d, 0x40, 0x6b, 0x77, 0x93, 0xd6, 0xc0, 0x86,
    0x6c, 0x03, 0x14, 0xdf, 0x78, 0x2d, 0xe0, 0x9b, 0x5e, 0x05, 0xf0, 0x93, 0xbd, 0x03, 0x1d, 0x17,
    0x56, 0x88, 0x58, 0x25, 0xa6, 0xae, 0x63, 0xd2, 0x01, 0x43, 0xbb, 0x7e, 0x7a, 0xa5, 0x62, 0xdf,
    0x8a, 0x31, 0xbd, 0x24, 0x1b, 0x1b, 0xeb, 0xfe, 0xdf, 0xd1, 0x31, 0x61, 0x4a, 0xfa, 0xdd, 0x6e,
    0x62, 0x0c, 0xa9, 0xcd, 0x08, 0x0c, 0xa1, 0x1b, 0xe7, 0xf2, 0xed, 0x36, 0x22, 0xd0, 0x5d, 0x80,
    0x78, 0xeb, 0x6f, 0x5a, 0x58, 0x18, 0xb5, 0xaf, 0x82, 0x77, 0x4c, 0x95, 0xce, 0xc6, 0x4d, 0xda,
    0xca, 0xef, 0x68, 0xa6, 0x6d, 0x71, 0x4d, 0xf1, 0x14, 0xaf, 0x68, 0x25, 0xb8, 0xf3, 0xff, 0xbe,
};


#ifdef DEBUG

//debug key whose privatekey is checked in as misc/debug.privkey
const uint8_t __attribute__ ((section (".pubkeys"))) _RSA_KEY_GOOGLE_DEBUG[] = {
    0x2d, 0xff, 0xa6, 0xb5, 0x65, 0x87, 0xbe, 0x61, 0xd1, 0xe1, 0x67, 0x10, 0xa1, 0x9b, 0xc6, 0xca,
    0xc8, 0xb1, 0xf0, 0xaa, 0x88, 0x60, 0x9f, 0xa1, 0x00, 0xa1, 0x41, 0x9a, 0xd8, 0xb4, 0xd1, 0x74,
    0x9f, 0x23, 0x28, 0x0d, 0xc2, 0xc4, 0x37, 0x15, 0xb1, 0x4a, 0x80, 0xca, 0xab, 0xb9, 0xba, 0x09,
    0x7d, 0xf8, 0x44, 0xd6, 0xa2, 0x72, 0x28, 0x12, 0x91, 0xf6, 0xa5, 0xea, 0xbd, 0xf8, 0x81, 0x6b,
    0xd2, 0x3c, 0x50, 0xa2, 0xc6, 0x19, 0x54, 0x48, 0x45, 0x8d, 0x92, 0xac, 0x01, 0xda, 0x14, 0x32,
    0xdb, 0x05, 0x82, 0x06, 0x30, 0x25, 0x09, 0x7f, 0x5a, 0xbb, 0x86, 0x64, 0x70, 0x98, 0x64, 0x1e,
    0xe6, 0xca, 0x1d, 0xc1, 0xcb, 0xb6, 0x23, 0xd2, 0x62, 0x00, 0x46, 0x97, 0xd5, 0xcc, 0xe6, 0x36,
    0x72, 0xec, 0x2e, 0x43, 0x1f, 0x0a, 0xaf, 0xf2, 0x51, 0xe1, 0xcd, 0xd2, 0x98, 0x5d, 0x7b, 0x64,
    0xeb, 0xd1, 0x35, 0x4d, 0x59, 0x13, 0x82, 0x6c, 0xbd, 0xc4, 0xa2, 0xfc, 0xad, 0x64, 0x73, 0xe2,
    0x71, 0xb5, 0xf4, 0x45, 0x53, 0x6b, 0xc3, 0x56, 0xb9, 0x8b, 0x3d, 0xeb, 0x00, 0x48, 0x6e, 0x29,
    0xb1, 0xb4, 0x8e, 0x2e, 0x43, 0x39, 0xef, 0x45, 0xa0, 0xb8, 0x8b, 0x5f, 0x80, 0xb5, 0x0c, 0xc3,
    0x03, 0xe3, 0xda, 0x51, 0xdc, 0xec, 0x80, 0x2c, 0x0c, 0xdc, 0xe2, 0x71, 0x0a, 0x14, 0x4f, 0x2c,
    0x22, 0x2b, 0x0e, 0xd1, 0x8b, 0x8f, 0x93, 0xd2, 0xf3, 0xec, 0x3a, 0x5a, 0x1c, 0xba, 0x80, 0x54,
    0x23, 0x7f, 0xb0, 0x54, 0x8b, 0xe3, 0x98, 0x22, 0xbb, 0x4b, 0xd0, 0x29, 0x5f, 0xce, 0xf2, 0xaa,
    0x99, 0x89, 0xf2, 0xb7, 0x5d, 0x8d, 0xb2, 0x72, 0x0b, 0x52, 0x02, 0xb8, 0xa4, 0x37, 0xa0, 0x3b,
    0xfe, 0x0a, 0xbc, 0xb3, 0xb3, 0xed, 0x8f, 0x8c, 0x42, 0x59, 0xbe, 0x4e, 0x31, 0xed, 0x11, 0x9b,
};

#endif


PREPOPULATED_ENCR_KEY(google_encr_key, ENCR_KEY_GOOGLE_PREPOPULATED, 0xf1, 0x51, 0x9b, 0x2e, 0x26, 0x6c, 0xeb, 0xe7, 0xd6, 0xd6, 0x0d, 0x17, 0x11, 0x94, 0x99, 0x19, 0x1c, 0xfb, 0x71, 0x56, 0x53, 0xf7, 0xe0, 0x7d, 0x90, 0x07, 0x53, 0x68, 0x10, 0x95, 0x1b, 0x70);




