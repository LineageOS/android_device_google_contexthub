#include <cpu/inc/atomicBitset.h>
#include <atomicBitset.h>
#include <platform.h>
#include <atomic.h>
#include <stdlib.h>
#include <stdio.h>
#include <timer.h>
#include <seos.h>


struct Timer {
    uint64_t      expires; /* time of next expiration */
    uint32_t      period;  /* 0 for oneshot */
    uint32_t      id;      /* 0 for disabled */
    TimTimerCbkF  cbk;
    uint32_t      jitterPpm;
    uint32_t      driftPpm;
    TimTimerCbkF  cbkF;
    void         *cbkData;
};


ATOMIC_BITSET_DECL(mTimersValid, MAX_TIMERS, static);
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

static bool timerSetAlarms(uint64_t nextTimer, uint64_t curTime, uint32_t maxJitterPpm, uint32_t maxDriftPpm, uint32_t maxErrTotalPpm)
{
    //here the code to set next timer of some variety will live
    //note that maxErrTotalPpm != maxDriftPpm + maxJitterPpm is quite possible since it is possible to have:
    // timer A allowing 300ppm of jitter and 10pp of drift and timer B allowing 20ppm of jitter and 500ppm of drift
    // in that case we'd see maxJitterPpm = 200, maxDriftPpm = 500, maxErrTotalPpm = 520  (MAX of all timers' allowable error totals)
    //return true if timer was set. false if you failed (you will be called right back though. so false is usually reserved for cases
    // like "it is too soon to set a timer")

    //todo
    return false;
}

static void timFireAsNeededAndUpdateAlarms(void)
{
    uint32_t maxDrift = 0, maxJitter = 0, maxErrTotal = 0;
    bool somethingDone = false;
    uint64_t nextTimer = 0;
    TimTimerCbkF cbkF;
    uint64_t curTime;
    uint32_t i, id;
    void *cbkD;

    do {

        for (i = 0; i < MAX_TIMERS; i++) {
            if (!mTimers[i].id)
                continue;

            if (mTimers[i].expires <= timGetTime()) {
                somethingDone = true;
                cbkF = mTimers[i].cbkF;
                cbkD = mTimers[i].cbkData;
                id = mTimers[i].id;
                if (mTimers[i].period)
                    mTimers[i].expires += mTimers[i].period;
                else {
                    mTimers[i].id = 0;
                    atomicBitsetClearBit(mTimersValid, i);
                }
                cbkF(id, cbkD);
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
        curTime = timGetTime();
    } while (somethingDone || curTime <= nextTimer || !timerSetAlarms(nextTimer, curTime, maxJitter, maxDrift, maxErrTotal));
}

uint32_t timTimerSet(uint64_t length, uint32_t jitterPpm, uint32_t driftPpm, TimTimerCbkF cbk, void* data, bool oneShot)
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
    t->cbkF = cbk;
    t->cbkData = data;

    /* as soon as we write timer Id, it becomes valid and might fire */
    t->id = timId;

    /* fire as needed & recalc alarms*/
    timFireAsNeededAndUpdateAlarms();

    /* woo hoo - done */
    return timId;
}

bool timTimerCancel(uint32_t timerId)
{
    uint64_t intState = platDisableInterrupts();
    struct Timer *t = timFindTimerById(timerId);

    if (t)
        t->id = 0; /* this disables it */

    platRestoreInterrupts(intState);

    /* this frees struct */
    if (t) {
        atomicBitsetClearBit(mTimersValid, t - mTimers);
        return true;
    }

    return false;
}

void timIntHandler(void)
{
    timFireAsNeededAndUpdateAlarms();
}

void timInit(void)
{
    atomicBitsetInit(mTimersValid, MAX_TIMERS);
}



