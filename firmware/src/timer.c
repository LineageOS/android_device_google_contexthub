#include <cpu/inc/atomicBitset.h>
#include <plat/inc/rtc.h>
#include <atomicBitset.h>
#include <platform.h>
#include <atomic.h>
#include <stdlib.h>
#include <stdio.h>
#include <timer.h>
#include <seos.h>
#include <cpu.h>
#include <slab.h>

#define MAX_INTERNAL_EVENTS       32 //also used for external app timer() calls

struct Timer {
    uint64_t      expires; /* time of next expiration */
    uint64_t      period;  /* 0 for oneshot */
    uint32_t      id;      /* 0 for disabled */
    uint32_t      jitterPpm;
    uint32_t      driftPpm;
    TaggedPtr     callInfo;
    void         *callData;
};


ATOMIC_BITSET_DECL(mTimersValid, MAX_TIMERS, static);
static struct SlabAllocator *mInternalEvents;
static struct Timer mTimers[MAX_TIMERS];
static volatile uint32_t mNextTimerId = 0;


uint64_t timGetTime(void)
{
    return platGetTicks();
}

static struct Timer *timFindTimerById(uint32_t timId) /* no locks taken. be careful what you do with this */
{
    uint32_t i;

    for (i = 0; i < MAX_TIMERS; i++)
        if (mTimers[i].id == timId)
            return mTimers + i;

    return NULL;
}

static void timerCallFuncFreeF(void* event)
{
    slabAllocatorFree(mInternalEvents, event);
}

static void timCallFunc(TaggedPtr callInfo, uint32_t id, void *callData)
{
    if (taggedPtrIsPtr(callInfo)) {
        return ((TimTimerCbkF)taggedPtrToPtr(callInfo))(id, callData);
    } else {
        struct TimerEvent *evt = (struct TimerEvent *)slabAllocatorAlloc(mInternalEvents);

        if (!evt)
            return;

        evt->timerId = id;
        evt->data = callData;
        if (osEnqueuePrivateEvt(EVT_APP_TIMER, evt, timerCallFuncFreeF, taggedPtrToUint(callInfo)))
            return;

        slabAllocatorFree(mInternalEvents, evt);
        return;
    }
}

static bool timFireAsNeededAndUpdateAlarms(void)
{
    uint32_t maxDrift = 0, maxJitter = 0, maxErrTotal = 0;
    bool somethingDone, totalSomethingDone = false;
    uint64_t nextTimer;
    TaggedPtr callInfo;
    uint32_t i, id;
    void *callData;

    do {
        somethingDone = false;
        nextTimer = 0;

        for (i = 0; i < MAX_TIMERS; i++) {
            if (!mTimers[i].id)
                continue;

            if (mTimers[i].expires <= timGetTime()) {
                somethingDone = true;
                callInfo = mTimers[i].callInfo;
                callData = mTimers[i].callData;
                id = mTimers[i].id;
                if (mTimers[i].period)
                    mTimers[i].expires += mTimers[i].period;
                else {
                    mTimers[i].id = 0;
                    atomicBitsetClearBit(mTimersValid, i);
                }
                timCallFunc(callInfo, id, callData);
            }
            else {
                if (mTimers[i].jitterPpm > maxJitter)
                    maxJitter = mTimers[i].jitterPpm;
                if (mTimers[i].driftPpm > maxDrift)
                    maxDrift = mTimers[i].driftPpm;
                if (mTimers[i].driftPpm + mTimers[i].jitterPpm > maxErrTotal)
                    maxErrTotal = mTimers[i].driftPpm + mTimers[i].jitterPpm;
                if (!nextTimer || nextTimer > mTimers[i].expires)
                    nextTimer = mTimers[i].expires;
            }
        }

        totalSomethingDone = totalSomethingDone || somethingDone;

    //we loop while loop does something, or while (if next timer exists), it is due by the time loop ends, or platform code fails to set an alarm to wake us for it
    } while (somethingDone || (nextTimer && (timGetTime() >= nextTimer || !platSleepClockRequest(nextTimer, maxJitter, maxDrift, maxErrTotal))));

    if (!nextTimer)
        platSleepClockRequest(0, 0, 0, 0);

    return totalSomethingDone;
}

static uint32_t timTimerSetEx(uint64_t length, uint32_t jitterPpm, uint32_t driftPpm, TaggedPtr info, void* data, bool oneShot)
{
    uint64_t curTime = timGetTime();
    int32_t idx = atomicBitsetFindClearAndSet(mTimersValid);
    struct Timer *t;
    uint32_t timId;

    if (idx < 0) /* no free timers */
        return 0;

    /* generate next timer ID */
    do {
        timId = atomicAdd(&mNextTimerId, 1);
    } while (!timId || timFindTimerById(timId));

    /* grab our struct & fill it in */
    t = mTimers + idx;
    t->expires = curTime + length;
    t->period = oneShot ? 0 : length;
    t->jitterPpm = jitterPpm;
    t->driftPpm = driftPpm;
    t->callInfo = info;
    t->callData = data;

    /* as soon as we write timer Id, it becomes valid and might fire */
    t->id = timId;

    /* fire as needed & recalc alarms*/
    timFireAsNeededAndUpdateAlarms();

    /* woo hoo - done */
    return timId;
}

uint32_t timTimerSet(uint64_t length, uint32_t jitterPpm, uint32_t driftPpm, TimTimerCbkF cbk, void* data, bool oneShot)
{
    return timTimerSetEx(length, jitterPpm, driftPpm, taggedPtrMakeFromPtr(cbk), data, oneShot);
}

uint32_t timTimerSetAsApp(uint64_t length, uint32_t jitterPpm, uint32_t driftPpm, uint32_t tid, void* data, bool oneShot)
{
    return timTimerSetEx(length, jitterPpm, driftPpm, taggedPtrMakeFromUint(tid), data, oneShot);
}

bool timTimerCancel(uint32_t timerId)
{
    uint64_t intState = cpuIntsOff();
    struct Timer *t = timFindTimerById(timerId);

    if (t)
        t->id = 0; /* this disables it */

    cpuIntsRestore(intState);

    /* this frees struct */
    if (t) {
        atomicBitsetClearBit(mTimersValid, t - mTimers);
        return true;
    }

    return false;
}

bool timIntHandler(void)
{
    return timFireAsNeededAndUpdateAlarms();
}

void timInit(void)
{
    atomicBitsetInit(mTimersValid, MAX_TIMERS);

    mInternalEvents = slabAllocatorNew(sizeof(struct TimerEvent), 4, MAX_INTERNAL_EVENTS);
}
