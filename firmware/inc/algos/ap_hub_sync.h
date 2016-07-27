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

#ifndef AP_HUB_SYNC_H__
#define AP_HUB_SYNC_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * This implements an AP-HUB time sync algorithm that is expected to improve time sync accuracy by
 * avoiding communication latency jitter.
 *
 * It uses max of (ap_time - hub_time) in a window, which is more consistent than average, to
 * establish mapping between ap timestamp and hub stamp. Additional low pass filtering is added
 * to further lowering jitter (it is not expected for two clocks to drift much in short time).
 *
 * Max is slightly anti-intuitive here because difference is defined as ap_time - hub_time. Max of
 * that is equivalent to min of hub_time - ap_time, which corresponds to a packet that get delayed
 * by system scheduling minimally (closer to the more consistent hardware related latency).
 */

struct ApHubSync {
    uint64_t last_ts;           // ap_time of previous data point, used for control expiration
    int64_t delta_est;          // the estimated delta between two clocks, filtered.

    int64_t win_max;            // track the maximum timestamp difference in a window
    uint64_t win_timeout;       // track window expiration time
    uint8_t state;              // internal state of the sync
};

// reset data structure
void ahsync_reset(struct ApHubSync* sync);

// add a data point (a pair of ap_time and the corresponding hub time).
void ahsync_add_delta(struct ApHubSync* sync, uint64_t ap_time, uint64_t hub_time);

// get the estimation of time delta
int64_t ahsync_get_delta(struct ApHubSync* sync, uint64_t hub_time);

#ifdef __cplusplus
}
#endif

#endif  // AP_HUB_SYNC_H__
