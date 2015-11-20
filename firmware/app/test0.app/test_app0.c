#include <stdlib.h>
#include <string.h>

#include <seos.h>
#include <timer.h>
#include <syscallDo.h>

static uint32_t mMyTid;
static int cnt;

static bool start_task(uint32_t myTid)
{
    mMyTid = myTid;
    cnt = 5;

    return eOsEventSubscribe(myTid, EVT_APP_START);
}

static void end_task(void)
{
    eOsLog(LOG_DEBUG, "App 0 terminating");
}

static void handle_event(uint32_t evtType, const void* evtData)
{
    const struct TimerEvent *te;
    uint32_t timerId;

    if (evtType == EVT_APP_START) {
        timerId = eOsTimTimerSet(1000000000ULL, 50, 50, mMyTid, (void *)&cnt, false);
        eOsLog(LOG_INFO, "App 0 started with tid %u timerid %u\n", mMyTid, timerId);
    } else if (evtType == EVT_APP_TIMER) {
        te = evtData;
        eOsLog(LOG_INFO, "App 0 received timer %u callback: %d\n", te->timerId, *(int *)te->data);
        if (cnt-- <= 0)
            eOsTimTimerCancel(te->timerId);
    }
}

APP_INIT(start_task, end_task, handle_event);





