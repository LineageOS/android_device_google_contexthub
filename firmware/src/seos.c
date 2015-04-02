#include <platform.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <printf.h>
#include <eventQ.h>
#include <timer.h>
#include <stdio.h>
#include <seos.h>
#include <heap.h>
#include <slab.h>
#include <cpu.h>



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
    struct AppEntry funcs;

    /* for some basic number of subbed events, the array is stored directly here. after that, a heap chunk is used */
    uint32_t subbedEventsInt[MAX_EMBEDDED_EVT_SUBS];
};

union DeferredAction {
    struct {
        uint32_t tid;
        uint32_t evt;
    } evtSub;
};

#define EVT_SUBSCRIBE_TO_EVT         0x00000000
#define EVT_UNSUBSCRIBE_TO_EVT       0x00000001



static struct EvtQueue *mEvtsInternal, *mEvtsExternal;
static struct SlabAllocator* mDeferedActionsSlab;
static struct Task mTasks[MAX_TASKS];
static uint32_t mNextTid = 1;

static void osInit(void)
{
    cpuInit();
    heapInit();
    platInitialize();

    osLog(LOG_INFO, "SEOS Initializing\n");

    /* init task list */
    memset(mTasks, 0, sizeof(mTasks));

    /* create the queues */
    if (!evtQueueSubsystemInit() || !(mEvtsInternal = evtQueueAlloc(512)) || !(mEvtsExternal = evtQueueAlloc(256))) {
        osLog(LOG_INFO, "events failed to init\n");
        return;
    }

    mDeferedActionsSlab = slabAllocatorNew(sizeof(union DeferredAction), 1, 32 /* for now? */);
    if (!mDeferedActionsSlab) {
        osLog(LOG_INFO, "deferred actions list failed to init\n");
        return;
    }
}

static void osStartTasks(void)
{
    extern struct AppEntry __app_start, __app_end;
    struct AppEntry *app;
    uint32_t i, nTasks = 0;

    osLog(LOG_INFO, "SEOS Registering tasks\n");
    for (app = &__app_start; app != &__app_end && nTasks < MAX_TASKS; app++, nTasks++) {
        memcpy(&mTasks[nTasks].funcs, app, sizeof(*app));
        mTasks[nTasks].subbedEvtListSz = MAX_EMBEDDED_EVT_SUBS;
        mTasks[nTasks].subbedEvents = mTasks[nTasks].subbedEventsInt;
        mTasks[nTasks].tid = mNextTid++;
    }

    osLog(LOG_INFO, "SEOS Starting tasks\n");
    for (i = 0; i < nTasks; i++)
        mTasks[i].funcs.start(mTasks[i].tid);
}

static struct Task* osTaskFindByTid(uint32_t tid)
{
    uint32_t i;

    for(i = 0; i < MAX_TASKS; i++)
        if (mTasks[i].tid == tid)
            return mTasks + i;

    return NULL;
}

static void osInternalEvtHandle(uint32_t evtType, void *evtData)
{
    union DeferredAction *da = (union DeferredAction*)evtData;
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
    }
}

void __attribute__((noreturn)) osMain(void)
{
    EventFreeF evtFree;
    uint32_t evtType, i, j;
    void *evtData;

    platDisableInterrupts();
    osInit();
    osStartTasks();
    platEnableInterrupts();

    while (true) {

        /* get an event */
        if (!evtQueueDequeue(mEvtsInternal, &evtType, &evtData, &evtFree, true))
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
                    if (mTasks[i].subbedEvents[j] == evtType) {
                        mTasks[i].funcs.handle(evtType, evtData);
                        break;
                    }
                }
            }
        }

        /* free it */
        if (evtFree)
            evtFree(evtData);
    }
}

static void osDeferredActionFreeF(void* event)
{
    slabAllocatorFree(mDeferedActionsSlab, event);
}

static bool osEventSubscribeUnsubscribe(uint32_t tid, uint32_t evtType, bool sub)
{
    union DeferredAction *act = slabAllocatorAlloc(mDeferedActionsSlab);

    if (!act)
        return false;
    act->evtSub.evt = evtType;
    act->evtSub.tid = tid;

    if (osEnqueueEvt(sub ? EVT_SUBSCRIBE_TO_EVT : EVT_UNSUBSCRIBE_TO_EVT, act, osDeferredActionFreeF, false))
        return true;

    slabAllocatorFree(mDeferedActionsSlab, act);
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

bool osEnqueueEvt(uint32_t evtType, void *evtData, EventFreeF evtFreeF, bool external)
{
    return evtQueueEnqueue(external ? mEvtsExternal : mEvtsInternal, evtType, evtData, evtFreeF);
}

static bool osLogPutcharF(void* userData, char c)
{
    platLogPutchar(c);
    return true;
}

void osLog(enum LogLevel level, const char *str, ...)
{
    va_list vl;

    osLogPutcharF(NULL, level);
    va_start(vl, str);
    cvprintf(osLogPutcharF, NULL, str, vl);
    va_end(vl);
}






