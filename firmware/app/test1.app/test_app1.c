#include <stdlib.h>
#include <string.h>

#include <seos.h>

static bool start_task(uint32_t myTid)
{
    //todo
    return true;
}

static void end_task(void)
{
    //todo
}

static void handle_event(uint32_t evtType, const void* evtData)
{
    //todo
}

APP_INIT(0, start_task, end_task, handle_event);
