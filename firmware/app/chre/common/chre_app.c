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

#include <eventnums.h>
#include <seos.h>
#include <timer.h>
#include <toolchain.h>
#include <crt_priv.h>

#include <chre.h>

/*
 * Common CHRE App support code
 */

static bool chreappStart(uint32_t tid)
{
    __crt_init();
    return nanoappStart();
}

static void chreappEnd(void)
{
    nanoappEnd();
    __crt_exit();
}

static void chreappHandle(uint32_t eventTypeAndTid, const void *eventData)
{
    uint16_t evt = eventTypeAndTid;
    uint16_t srcTid = eventTypeAndTid >> 16;
    const void *data = eventData;

    union EventLocalData {
    struct chreMessageFromHostData msg;
    } u;

    switch(evt) {
    case EVT_APP_TIMER:
        evt = CHRE_EVENT_TIMER;
        data = ((struct TimerEvent *)eventData)->data;
        break;
    case EVT_APP_FROM_HOST:
        evt = CHRE_EVENT_MESSAGE_FROM_HOST;
        data = &u.msg;
        u.msg.message = (uint8_t*)eventData + 1;
        u.msg.reservedMessageType = 0;
        u.msg.messageSize = *(uint8_t*)eventData;
        break;
    case EVT_APP_FROM_HOST_CHRE:
    {
        const struct NanohubMsgChreHdr *hdr = eventData;
        evt = CHRE_EVENT_MESSAGE_FROM_HOST;
        data = &u.msg;
        u.msg.message = hdr + 1;
        u.msg.reservedMessageType = hdr->appEvent;
        u.msg.messageSize = hdr->size;
        break;
    }
    }
    nanoappHandleEvent(srcTid, evt, data);
}

// Collect entry points
const struct AppFuncs SET_EXTERNAL_APP_ATTRIBUTES(used, section (".app_init"),visibility("default")) _mAppFuncs = {
    .init   = chreappStart,
    .end    = chreappEnd,
    .handle = chreappHandle,
};

// declare version for compatibility with current runtime
const uint32_t SET_EXTERNAL_APP_VERSION(used, section (".app_version"), visibility("default")) _mAppVer = 0;
