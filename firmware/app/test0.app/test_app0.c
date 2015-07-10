#include <stdlib.h>
#include <string.h>

#include <seos.h>
#include <syscallDo.h>

static uint32_t mMyTid;

static bool start_task(uint32_t myTid)
{
    mMyTid = myTid;

    return eOsEventSubscribe(myTid, EVT_APP_START);
}

static void end_task(void)
{
    eOsLog(LOG_DEBUG, "App 0 terminating");
}

static void handle_event(uint32_t evtType, const void* evtData)
{
    if (evtType == EVT_APP_START)
        eOsLog(LOG_INFO, "App 0 started with tid %u", mMyTid);
}

APP_INIT(start_task, end_task, handle_event);





