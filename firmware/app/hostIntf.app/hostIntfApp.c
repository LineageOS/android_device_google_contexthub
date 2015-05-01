#include <seos.h>
#include <hostIntf.h>

static void hostIntfAppStart(uint32_t tid)
{
    hostIntfRequest();
}

static void hostIntfAppEnd(void)
{
    hostIntfRelease();
}

static void hostintfAppHandle(uint32_t evtType, const void* evtData)
{
}

APP_INIT(hostIntfAppStart, hostIntfAppEnd, hostintfAppHandle);
