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

#include <algos/ap_hub_sync.h>
#include <cpu/inc/cpuMath.h>

#include <limits.h>
#include <seos.h>

#define S_IN_NS(s)          (UINT64_C(1000000000)*(s))

#define SYNC_EXPIRATION     S_IN_NS(50) //50 sec in ns, at max 500us diff
#define SYNC_WINDOW_TIMEOUT S_IN_NS(2)  //2 sec in ns
#define SYNC_FILTER_B       8
#define SYNC_FILTER_A       1

#define DEBUG_SYNC          false

enum ApHubSyncState {
    NOT_INITED = 0,
    USE_MAX,
    USE_FILTERED
};

void ahsync_reset(struct ApHubSync* sync) {
    sync->state = 0;
    if (DEBUG_SYNC) {
        osLog(LOG_DEBUG, "ApHub sync reset");
    }
}

void ahsync_add_delta(struct ApHubSync* sync, uint64_t ap_time, uint64_t hub_time) {

    int64_t delta = ap_time - hub_time;

    // if data is expired or last_ts is not set before, reset
    if (ap_time > sync->last_ts + SYNC_EXPIRATION || sync->last_ts == 0) {
        ahsync_reset(sync);
    }

    sync->last_ts = ap_time;

    if (sync->state == NOT_INITED) {
        // setup the win_max before switching state
        sync->win_max = delta;
        sync->win_timeout = ap_time + SYNC_WINDOW_TIMEOUT;

        sync->state = USE_MAX;
    } else {
        sync->win_max = (delta > sync->win_max) ? delta : sync->win_max;
        if (ap_time > sync->win_timeout) {
            // collected a window

            // setup delta_est before switching state
            if (sync->state == USE_MAX) {
                sync->delta_est = sync->win_max;
            } else {
                sync->delta_est = ((SYNC_FILTER_B - SYNC_FILTER_A) * sync->delta_est +
                                   SYNC_FILTER_A * sync->win_max) / SYNC_FILTER_B;
            }
            sync->state = USE_FILTERED;
            if (DEBUG_SYNC) {
                osLog(LOG_DEBUG, "ApHub new sync offset = %" PRId64, sync->delta_est);
            }
            // start new window by resetting win_max and win_timeout after this window is done
            sync->win_max = INT64_MIN;
            sync->win_timeout = ap_time + SYNC_WINDOW_TIMEOUT;
        }
    }
}

int64_t ahsync_get_delta(struct ApHubSync* sync, uint64_t hub_time) {
    int64_t ret;
    switch (sync->state) {
        case NOT_INITED:
            ret = 0;
            break;
        case USE_MAX:
            ret = sync->win_max;
            break;
        case USE_FILTERED:
            ret = sync->delta_est;
            break;
        default:
            // indicate error, should never happen
            ret = INT64_MIN;
            osLog(LOG_WARN, "ApHub sync: Invalid sync state %d", sync->state);
            ahsync_reset(sync);
    }
    return ret;
}


