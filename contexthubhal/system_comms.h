/*
 * Copyright (c) 2016, Google. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _NANOHUB_SYSTEM_COMMS_H_
#define _NANOHUB_SYSTEM_COMMS_H_

#include <hardware/context_hub.h>
#include "nanohubhal.h"

//rx: return 0 if handled, 1 if not handled, else -errno
int system_comms_handle_rx(const void *name, uint32_t len, const void *data);
int system_comms_handle_tx(const struct hub_message_t *outMsg);


//messages to the HostIf nanoapp & their replies (mesages and replies both begin with u8 message_type)
#define NANOHUB_EXT_APPS_ON        0 // () -> (char success)
#define NANOHUB_EXT_APPS_OFF       1 // () -> (char success)
#define NANOHUB_EXT_APP_DELETE     2 // (u64 name) -> (char success)    //idempotent
#define NANOHUB_QUERY_MEMINFO      3 // () -> (mem_info)
#define NANOHUB_QUERY_APPS         4 // (u32 idxStart) -> (app_info[idxStart] OR EMPTY IF NO MORE)
#define NANOHUB_QUERY_RSA_KEYS     5 // (u32 byteOffset) -> (u8 data[1 or more bytes] OR EMPTY IF NO MORE)
#define NANOHUB_START_UPLOAD       6 // (char isOs, u32 totalLenToTx) -> (char success)
#define NANOHUB_CONT_UPLOAD        7 // (u32 offset, u8 data[]) -> (char success)
#define NANOHUB_FINISH_UPLOAD      8 // () -> (char success)
#define NANOHUB_REBOOT             9 // () -> (char success)



#define NANOHUB_UPLOAD_CHUNK_SZ_MAX 64
#define NANOHUB_MEM_SZ_UNKNOWN      0xFFFFFFFFUL

struct NanohubAppInfo {
    uint8_t name[APP_NAME_LEN];
    uint32_t version, flashUse, ramUse;
};

struct NanohubMemInfo {
    //sizes
    uint32_t flashSz, blSz, osSz, sharedSz, eeSz;
    uint32_t ramSz;

    //use
    uint32_t blUse, osUse, sharedUse, eeUse;
    uint32_t ramUse;
};



#endif

