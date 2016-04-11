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

#define LOG_TAG "NanohubHAL"
#include <hardware/context_hub.h>
#include "nanohub_perdevice.h"
#include "system_comms.h"
#include "nanohubhal.h"
#include <utils/Log.h>
#include <inttypes.h>
#include <stdbool.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

static pthread_mutex_t mLock = PTHREAD_MUTEX_INITIALIZER; //protects every global here
static bool mHaveRsaKeys = false, mAppInfoReqInProgress = false, mUploadingOs, mUploadingStarted = false;
static uint32_t mRsaOffset, mUploadLen, mUploadPos, mAppQueryIdx = 0;
static uint8_t *mUploadData = NULL, *mRsaKeys = NULL;
static struct NanohubAppInfo *mAppInfo = NULL;
static uint32_t mUploadResult;

static const struct hub_app_name_t mHostIfAppName = {
    .id = NANO_APP_ID(NANOAPP_VENDOR_GOOGLE, 0)
};

static uint32_t leToHost32(uint32_t in)
{
    const uint8_t *buf = (const uint8_t*)&in;
    uint32_t ret = buf[3];

    ret = (ret << 8) + buf[2];
    ret = (ret << 8) + buf[1];
    return (ret << 8) + buf[0];
}

static void send_msg_to_java(uint32_t typ, uint32_t len, const void *data)
{
    hal_hub_call_rx_msg_f(&get_hub_info()->os_app_name, typ, len, data);
}

static int system_comms_handle_app_info_query_locked(bool start)
{
    if (start) {
        mAppInfoReqInProgress = true;
        mAppInfo = NULL;
        mAppQueryIdx = 0;
    }
    struct {
        uint8_t cmd;
        uint32_t idx;
    } __attribute__((packed)) cmd;

    cmd.cmd = NANOHUB_QUERY_APPS;
    memcpy(&cmd.idx, &mAppQueryIdx, sizeof(cmd.idx));

    return hal_hub_do_send_msg(&mHostIfAppName, sizeof(cmd), &cmd);
}

static int system_comms_send_app_info_to_java_locked(void)
{
    uint32_t infosSz;
    struct mem_range_t *ranges = (struct mem_range_t*)calloc(1, mAppQueryIdx * 2 *sizeof(struct mem_range_t));
    struct hub_app_info *infos = (struct hub_app_info*)calloc(1, infosSz = mAppQueryIdx * sizeof(struct hub_app_info));
    uint32_t i, j = 0;

    if (ranges && infos) {

        for (i = 0; i < mAppQueryIdx; i++) { //loop over apps

            uint32_t startRange = j;

            //copy mem info
            if (mAppInfo[i].flashUse != NANOHUB_MEM_SZ_UNKNOWN) {
                ranges[j].type = HUB_MEM_TYPE_MAIN;
                ranges[j].total_bytes = mAppInfo[i].flashUse;
                j++;
            }
            if (mAppInfo[i].ramUse != NANOHUB_MEM_SZ_UNKNOWN) {
                ranges[j].type = HUB_MEM_TYPE_RAM;
                ranges[j].total_bytes = mAppInfo[i].ramUse;
                j++;
            }

            //populate app info
            infos[i].app_name = mAppInfo[i].name;
            infos[i].version = mAppInfo[i].version;
            infos[i].num_mem_ranges = j - startRange;
            infos[i].mem_usage = ranges + startRange;
        }

        //send
        send_msg_to_java(CONTEXT_HUB_QUERY_APPS, infosSz, infos);
    }

    if(ranges)
        free(ranges);

    if (infos)
        free(infos);

    if (mAppInfo)
        free(mAppInfo);
    mAppInfo = NULL;

    return 0;
}

static int system_comms_handle_rx_app_info(uint32_t len, const uint8_t *data)
{
    int ret = 0;

    pthread_mutex_lock(&mLock);

    if (!mAppInfoReqInProgress) {
        ALOGW("Unexpected app_info packet RXed. dropping");
        goto out;
    }

    if (len == sizeof(struct NanohubAppInfo)) {
        void *tmp = (struct NanohubAppInfo*)realloc(mAppInfo, sizeof(struct NanohubAppInfo[mAppQueryIdx + 1]));

        if (tmp) {
            mAppInfo = tmp;
            memcpy(&mAppInfo[mAppQueryIdx++], data, sizeof(struct NanohubAppInfo));

            if (!system_comms_handle_app_info_query_locked(false))
                goto out; // expect more entries
        }
    }

    //if we're here, we're done
    mAppInfoReqInProgress = false;
    ret = system_comms_send_app_info_to_java_locked();

out:
    pthread_mutex_unlock(&mLock);
    return ret;
}

static int system_comms_handle_rx_mem_info(uint32_t len, const uint8_t *data)
{
    int ret = 0;

    if (len != sizeof(struct NanohubMemInfo))
        ALOGW("Mem info command reply is a weird size");
    else {
        const struct NanohubMemInfo *mi = (const struct NanohubMemInfo*)data;
        struct mem_range_t ranges[4];
        uint32_t nRanges = 0;

        //calcualte each
        struct mem_range_t rangeShared = {
            .type = HUB_MEM_TYPE_MAIN,
            .total_bytes = leToHost32(mi->sharedSz),
            .free_bytes = leToHost32(mi->sharedSz) - leToHost32(mi->sharedUse),
        };

        struct mem_range_t rangeOs = {
            .type = HUB_MEM_TYPE_OS,
            .total_bytes = leToHost32(mi->osSz),
            .free_bytes = leToHost32(mi->osSz) - leToHost32(mi->osUse),
        };

        struct mem_range_t rangeEe = {
            .type = HUB_MEM_TYPE_EEDATA,
            .total_bytes = leToHost32(mi->eeSz),
            .free_bytes = leToHost32(mi->eeSz) - leToHost32(mi->eeUse),
        };

        struct mem_range_t rangeRam = {
            .type = HUB_MEM_TYPE_RAM,
            .total_bytes = leToHost32(mi->ramSz),
            .free_bytes = leToHost32(mi->ramSz) - leToHost32(mi->ramUse),
        };

        //if each is valid, copy to output area
        if (leToHost32(mi->sharedSz) != NANOHUB_MEM_SZ_UNKNOWN && leToHost32(mi->sharedUse) != NANOHUB_MEM_SZ_UNKNOWN)
            ranges[nRanges++] = rangeShared;

        if (leToHost32(mi->osSz) != NANOHUB_MEM_SZ_UNKNOWN && leToHost32(mi->osUse) != NANOHUB_MEM_SZ_UNKNOWN)
            ranges[nRanges++] = rangeOs;

        if (leToHost32(mi->eeSz) != NANOHUB_MEM_SZ_UNKNOWN && leToHost32(mi->eeUse) != NANOHUB_MEM_SZ_UNKNOWN)
            ranges[nRanges++] = rangeEe;

        if (leToHost32(mi->ramSz) != NANOHUB_MEM_SZ_UNKNOWN && leToHost32(mi->ramUse) != NANOHUB_MEM_SZ_UNKNOWN)
            ranges[nRanges++] = rangeRam;

        //send it out
        send_msg_to_java(CONTEXT_HUB_QUERY_MEMORY, sizeof(struct mem_range_t[nRanges]), ranges);
    }

    return ret;
}

static int system_comms_handle_upload_piece_locked(void)
{
    //are we just starting?
    if (!mUploadingStarted) {
        uint8_t msg[6] = {NANOHUB_START_UPLOAD, mUploadingOs, mUploadLen, mUploadLen >> 8, mUploadLen >> 16, mUploadLen >> 24};

        //TODO: future: RSA check here as an optimization

        mUploadingStarted = true;
        return hal_hub_do_send_msg(&mHostIfAppName, sizeof(msg), msg);
    }

    //are we just finishing
    else if (mUploadPos == mUploadLen) {
        uint8_t msg = NANOHUB_FINISH_UPLOAD;

        return hal_hub_do_send_msg(&mHostIfAppName, 1, &msg);
    }

    //guess we're in the middle of it - upload NANOHUB_UPLOAD_CHUNK_SZ_MAX bytes or what's left, whichever is smaller
    else {
        uint32_t now = mUploadLen - mUploadPos;
        uint8_t msg[NANOHUB_UPLOAD_CHUNK_SZ_MAX + 5] = {NANOHUB_CONT_UPLOAD, mUploadPos, mUploadPos >> 8, mUploadPos >> 16, mUploadPos >> 24};

        if (now > NANOHUB_UPLOAD_CHUNK_SZ_MAX)
            now = NANOHUB_UPLOAD_CHUNK_SZ_MAX;

        memcpy(msg + 5, mUploadData + mUploadPos, now);
        mUploadPos += now;

        return hal_hub_do_send_msg(&mHostIfAppName, now + 5, msg);
    }
}

static int system_comms_handle_rx_upload_finish_locked(bool success)
{
    uint8_t msg = NANOHUB_REBOOT;
    uint8_t result = success ? NANOHUB_APP_LOADED : NANOHUB_APP_NOT_LOADED;

    free(mUploadData);
    mUploadData = NULL;

    send_msg_to_java(mUploadingOs ? CONTEXT_HUB_LOAD_OS : CONTEXT_HUB_LOAD_APP,
            sizeof(result), &result);

    if (success) {
        mUploadResult = true;
        return hal_hub_do_send_msg(&mHostIfAppName, 1, &msg);
    }

    return 0;
}

static int system_comms_handle_rx_upload_status(uint32_t len, const uint8_t *data, bool fini)
{
    int ret = 0;

    pthread_mutex_lock(&mLock);

    if (len != 1 || !*data) {
        ALOGW("msg len is bad");
        ret = system_comms_handle_rx_upload_finish_locked(false);
        goto out;
    }

    if (fini)
        ret = system_comms_handle_rx_upload_finish_locked(true);
    else
        ret = system_comms_handle_upload_piece_locked();

out:
    pthread_mutex_unlock(&mLock);
    return ret;
}

static int system_comms_handle_rx_upload_start(uint32_t len, const uint8_t *data)
{
    return system_comms_handle_rx_upload_status(len, data, false);
}

static int system_comms_handle_rx_upload_cont(uint32_t len, const uint8_t *data)
{
    return system_comms_handle_rx_upload_status(len, data, false);
}

static int system_comms_handle_rx_upload_fini(uint32_t len, const uint8_t *data)
{
    return system_comms_handle_rx_upload_status(len, data, true);
}

static int system_comms_handle_rx_reboot(uint32_t len, const uint8_t *data)
{
    uint8_t ret = NANOHUB_APPS_RUNNING;

    if (len == sizeof(uint32_t)) {
        uint32_t status;
        memcpy(&status, data, sizeof(status));
        ALOGI("Nanohub reboot status: %08" PRIX32, status);
    }

    pthread_mutex_lock(&mLock);
    if (mUploadResult)
        send_msg_to_java(mUploadingOs ? CONTEXT_HUB_LOAD_OS : CONTEXT_HUB_LOAD_APP, sizeof(ret), &ret);
    mUploadResult = false;
    pthread_mutex_unlock(&mLock);

    return 0;
}

static int system_comms_handle_get_rsa_keys_locked(void)
{
    uint8_t msg[5] = {NANOHUB_QUERY_RSA_KEYS, mRsaOffset, mRsaOffset >> 8, mRsaOffset >> 16, mRsaOffset >> 24};

    return hal_hub_do_send_msg(&mHostIfAppName, sizeof(msg), msg);
}

static int system_comms_handle_rx_rsa_keys(uint32_t len, const uint8_t *data)
{
    int ret = 0;

    pthread_mutex_lock(&mLock);

    if (mHaveRsaKeys) {
        ALOGW("Got RSA keys when they were not expected - dropping");
        goto out;
    }

    if (len) { // more key material has arived
        void *tmp = realloc(mRsaKeys, mRsaOffset + len);

        if (data) { //append and ask for more
            mRsaKeys = tmp;
            memcpy(mRsaKeys + mRsaOffset, data, len);
            mRsaOffset += len;
            if (!system_comms_handle_get_rsa_keys_locked())
                goto out;
        }
    }

    //if we're here, we're done getting rsa keys - continue whatever upload started all this
    mHaveRsaKeys = true;

    ret = system_comms_handle_upload_piece_locked();

out:
    pthread_mutex_unlock(&mLock);
    return ret;
}

int system_comms_handle_rx(const struct nano_message *msg)
{
    const uint8_t *data = &msg->data[0];
    int len = msg->hdr.len;
    uint8_t msgType;

    //we only care for messages from HostIF
    if (memcmp(&msg->hdr.app_name, &mHostIfAppName, sizeof(mHostIfAppName)) != 0)
        return 1;

    //they must all be at least 1 byte long
    if (len < 1)
        return -EINVAL;

    len--;
    switch (msgType = *data++) {

    case NANOHUB_EXT_APPS_ON:
        if (len != 1)
            return -EINVAL;
        send_msg_to_java(CONTEXT_HUB_APPS_ENABLE, 1, data);
        break;

    case NANOHUB_EXT_APPS_OFF:
        if (len != 1)
            return -EINVAL;
        send_msg_to_java(CONTEXT_HUB_APPS_DISABLE, 1, data);
        break;

    case NANOHUB_EXT_APP_DELETE:
        if (len != 1)
            return -EINVAL;
        send_msg_to_java(CONTEXT_HUB_UNLOAD_APP, 1, data);
        break;

    case NANOHUB_QUERY_APPS:
        return system_comms_handle_rx_app_info(len, data);

    case NANOHUB_QUERY_MEMINFO:
        return system_comms_handle_rx_mem_info(len, data);

    case NANOHUB_QUERY_RSA_KEYS:
        return system_comms_handle_rx_rsa_keys(len, data);

    case NANOHUB_START_UPLOAD:
        return system_comms_handle_rx_upload_start(len, data);

    case NANOHUB_CONT_UPLOAD:
        return system_comms_handle_rx_upload_cont(len, data);

    case NANOHUB_FINISH_UPLOAD:
        return system_comms_handle_rx_upload_fini(len, data);

    case NANOHUB_REBOOT:
        return system_comms_handle_rx_reboot(len, data);

    default:
        ALOGW("Unknown nanohub reply packet %u ( + %u bytes of data) - dropped", msgType, len);
        break;
    }

    return 0;
}

static int system_comms_handle_upload(const void *data, uint32_t len, bool isOs)
{
    int ret;

    pthread_mutex_lock(&mLock);

    if (mUploadData) {    // no concurrent uploads
        ret = -EBUSY;
        ALOGW("Refusing to upload while another upload is ongoing");
        goto out;
    }

    mUploadData = malloc(len);
    if (!mUploadData) {
        ret = -ENOMEM;
        ALOGW("OOPS OOM");
        goto out;
    }

    memcpy(mUploadData, data, len);
    mUploadLen = len;
    mUploadPos = 0;
    mUploadingOs = isOs;
    mUploadingStarted = false;

    if (!mHaveRsaKeys) { //no keys -> ask for them
        mRsaOffset = 0;
        ret = system_comms_handle_get_rsa_keys_locked();
        goto out;
    }

    ret = system_comms_handle_upload_piece_locked();

out:
    pthread_mutex_unlock(&mLock);
    return ret;
}

static int system_comms_handle_app_info_query(bool start)
{
    int ret;

    pthread_mutex_lock(&mLock);

    ret = mAppInfoReqInProgress ? -EBUSY : system_comms_handle_app_info_query_locked(start);

    pthread_mutex_unlock(&mLock);
    return ret;
}

int system_comms_handle_tx(const struct hub_message_t *outMsg)
{
    struct {
        uint8_t cmd;
        struct hub_app_name_t app_name;
    } __attribute__((packed)) cmd;
    uint8_t len = 1;

    switch (outMsg->message_type) {

    case CONTEXT_HUB_APPS_ENABLE:
        cmd.cmd = NANOHUB_EXT_APPS_ON;
        break;

    case CONTEXT_HUB_APPS_DISABLE:
        cmd.cmd = NANOHUB_EXT_APPS_OFF;
        break;

    case CONTEXT_HUB_LOAD_APP:
        return system_comms_handle_upload(outMsg->message, outMsg->message_len, false);

    case CONTEXT_HUB_UNLOAD_APP:
        if (outMsg->message_len != sizeof(cmd.app_name))
            return -EINVAL;

        cmd.cmd = NANOHUB_EXT_APP_DELETE;
        memcpy(&cmd.app_name, outMsg->message, sizeof(cmd.app_name));
        len += sizeof(cmd.app_name);
        break;

    case CONTEXT_HUB_QUERY_APPS:
        return system_comms_handle_app_info_query(true);

    case CONTEXT_HUB_QUERY_MEMORY:
        cmd.cmd = NANOHUB_QUERY_MEMINFO;
        break;

    case CONTEXT_HUB_LOAD_OS:
        return system_comms_handle_upload(outMsg->message, outMsg->message_len, true);

    default:
        ALOGW("Unknown os message type %u\n", outMsg->message_type);
        return -EINVAL;
    }

    return hal_hub_do_send_msg(&mHostIfAppName, len, &cmd);
}
