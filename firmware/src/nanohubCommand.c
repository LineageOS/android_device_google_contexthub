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

#include <plat/inc/taggedPtr.h>
#include <plat/inc/rtc.h>
#include <inttypes.h>
#include <string.h>
#include <stdint.h>
#include <sys/endian.h>

#include <atomicBitset.h>
#include <hostIntf.h>
#include <hostIntf_priv.h>
#include <nanohubCommand.h>
#include <nanohubPacket.h>
#include <seos.h>
#include <util.h>
#include <mpu.h>
#include <heap.h>
#include <sensType.h>
#include <timer.h>
#include <crc.h>
#include <rsa.h>
#include <appSec.h>
#include <plat/inc/bl.h>
#include <plat/inc/plat.h>
#include <variant/inc/variant.h>

#define NANOHUB_COMMAND(_reason, _handler, _minReqType, _maxReqType) \
        { .reason = _reason, .handler = _handler, \
          .minDataLen = sizeof(_minReqType), .maxDataLen = sizeof(_maxReqType) }

static struct DownloadState
{
    struct AppSecState *appSecState;
    uint32_t size;
    uint32_t srcOffset;
    uint32_t dstOffset;
    uint8_t *start;
    uint32_t crc;
    uint32_t srcCrc;
    uint8_t  data[NANOHUB_PACKET_PAYLOAD_MAX];
    uint8_t  len;
    uint8_t  chunkReply;
    uint8_t  type;
    bool     erase;
} *mDownloadState;

static size_t getOsHwVersion(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubOsHwVersionsResponse *resp = tx;
    resp->hwType = htole16(platHwType());
    resp->hwVer = htole16(platHwVer());
    resp->blVer = htole16(platBlVer());
    resp->osVer = htole16(OS_VER);
    resp->variantVer = htole32(VARIANT_VER);

    return sizeof(*resp);
}

static size_t getAppVersion(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubAppVersionsRequest *req = rx;
    struct NanohubAppVersionsResponse *resp = tx;
    uint32_t appIdx, appVer, appSize;

    if (osAppInfoById(le64toh(req->appId), &appIdx, &appVer, &appSize)) {
        resp->appVer = htole32(appVer);
        return sizeof(*resp);
    }

    return 0;
}

static size_t queryAppInfo(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubAppInfoRequest *req = rx;
    struct NanohubAppInfoResponse *resp = tx;
    uint64_t appId;
    uint32_t appVer, appSize;

    if (osAppInfoByIndex(le32toh(req->appIdx), &appId, &appVer, &appSize)) {
        resp->appId = htole64(appId);
        resp->appVer = htole32(appVer);
        resp->appSize = htole32(appSize);
        return sizeof(*resp);
    }

    return 0;
}

static AppSecErr writeCbk(const void *data, uint32_t len)
{
    AppSecErr ret;

    mpuAllowRamExecution(true);
    mpuAllowRomWrite(true);
    if (BL.blProgramShared(mDownloadState->start + mDownloadState->dstOffset, (uint8_t *)data, len, BL_FLASH_KEY1, BL_FLASH_KEY2) < 0) {
        ret = APP_SEC_BAD;
    } else {
        ret = APP_SEC_NO_ERROR;
        mDownloadState->dstOffset += len;
    }
    mpuAllowRomWrite(false);
    mpuAllowRamExecution(false);

    return ret;
}

static AppSecErr pubKeyFindCbk(const uint32_t *gotKey, bool *foundP)
{
    const uint32_t *ptr;
    uint32_t numKeys, i;

    *foundP = false;
    ptr = BL.blGetPubKeysInfo(&numKeys);
    for (i = 0; ptr && i < numKeys; i++, ptr += RSA_LIMBS) {
        if (!memcmp(gotKey, ptr, RSA_BYTES)) {
            *foundP = true;
            break;
        }
    }

    return APP_SEC_NO_ERROR;
}

static AppSecErr aesKeyAccessCbk(uint64_t keyIdx, void *keyBuf)
{
    extern const struct StmPlatEeDataGeneric __eedata_start;
    extern char __eedata_end[];
    const struct StmPlatEeDataGeneric *hdr;
    const struct StmPlatEeDataEncrKey *key;

    hdr = &__eedata_start;
    while (((uintptr_t)&__eedata_end) - ((uintptr_t)hdr) >= sizeof(struct StmPlatEeDataGeneric) && hdr->eeDataType != 0xFFFFF) {
        switch (hdr->eeDataType) {
        case EE_DATA_TYPE_ENCR_KEY:
            key = (const struct StmPlatEeDataEncrKey *)hdr;
            if (key->keyID == keyIdx) {
                memcpy(keyBuf, key->key, sizeof(key->key));
                return APP_SEC_NO_ERROR;
            }
            break;
        }
        hdr = (const struct StmPlatEeDataGeneric*)(((((uintptr_t)hdr) + sizeof(struct StmPlatEeDataGeneric) + hdr->eeDataLen) + 3) &~ 3);
    }

    return APP_SEC_KEY_NOT_FOUND;
}

static void resetDownloadState()
{
    if (mDownloadState->appSecState)
        appSecDeinit(mDownloadState->appSecState);
    mDownloadState->appSecState = appSecInit(writeCbk, pubKeyFindCbk, aesKeyAccessCbk, false);
    mDownloadState->srcOffset = 0;
    mDownloadState->srcCrc = ~0;
    mDownloadState->dstOffset = 4; // skip over header
}

static size_t startFirmwareUpload(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubStartFirmwareUploadRequest *req = rx;
    struct NanohubStartFirmwareUploadResponse *resp = tx;
    uint8_t *shared, *shared_start, *shared_end;
    int len, total_len;
    uint32_t sharedSz;

    shared_start = platGetSharedAreaInfo(&sharedSz);
    shared_end = shared_start + sharedSz;

    if (!mDownloadState)
        mDownloadState = heapAlloc(sizeof(struct DownloadState));

    mDownloadState->type = req->type;
    mDownloadState->size = le32toh(req->size);
    mDownloadState->crc = le32toh(req->crc);
    mDownloadState->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED;

    for (shared = shared_start;
         shared < shared_end && shared[0] != 0xFF;
         shared += total_len) {
        len = (shared[1] << 16) | (shared[2] << 8) | shared[3];
        total_len = sizeof(uint32_t) + ((len + 3) & ~3) + sizeof(uint32_t);
    }

    if (shared + sizeof(uint32_t) + ((mDownloadState->size + 3) & ~3) + sizeof(uint32_t) < shared_end) {
        mDownloadState->start = shared;
        mDownloadState->erase = false;
    } else {
        mDownloadState->start = shared_start;
        mDownloadState->erase = true;
    }
    resetDownloadState();

    resp->accepted = 1;

    return sizeof(*resp);
}

static void firmwareErase(void *cookie)
{
    osLog(LOG_INFO, "hostIntfFirmwareErase: Firmware Erase\n");
    if (mDownloadState->erase == true) {
        mpuAllowRamExecution(true);
        mpuAllowRomWrite(true);
        BL.blEraseShared(BL_FLASH_KEY1, BL_FLASH_KEY2);
        mpuAllowRomWrite(false);
        mpuAllowRamExecution(false);
        mDownloadState->erase = false;
        hostIntfSetInterrupt(NANOHUB_INT_CMD_WAIT);
    }
}

static uint8_t firmwareFinish(bool valid)
{
    uint8_t buffer[7];
    int padlen;
    uint32_t crc;
    uint16_t marker;
    static const char magic[] = APP_HDR_MAGIC;
    const struct AppHdr *app;
    AppSecErr ret;

    ret = appSecRxDataOver(mDownloadState->appSecState);

    if (ret == APP_SEC_NO_ERROR && valid && mDownloadState->type == BL_FLASH_APP_ID) {
        app = (const struct AppHdr *)&mDownloadState->start[4];

        if (app->marker != APP_HDR_MARKER_UPLOADING ||
                mDownloadState->size < sizeof(uint32_t) + sizeof(struct AppHdr) ||
                memcmp(magic, app->magic, sizeof(magic)-1) ||
                app->fmtVer != APP_HDR_VER_CUR) {
            marker = APP_HDR_MARKER_DELETED;
        } else {
            marker = APP_HDR_MARKER_VALID;
        }

        osLog(LOG_INFO, "Loaded %s app: %ld bytes @ %p\n", marker == APP_HDR_MARKER_VALID ? "valid" : "invalid", mDownloadState->size, mDownloadState->start);

        mpuAllowRamExecution(true);
        mpuAllowRomWrite(true);
        if (BL.blProgramShared((uint8_t *)&app->marker, (uint8_t *)&marker, sizeof(marker), BL_FLASH_KEY1, BL_FLASH_KEY2) < 0) {
            mpuAllowRomWrite(false);
            mpuAllowRamExecution(false);
            return NANOHUB_FIRMWARE_CHUNK_REPLY_RESEND;
        }
    } else {
        mpuAllowRamExecution(true);
        mpuAllowRomWrite(true);
    }

    if (ret == APP_SEC_NO_ERROR && valid)
        buffer[0] = ((mDownloadState->type & 0xF) << 4) | (mDownloadState->type & 0xF);
    else
        buffer[0] = 0x00;

    buffer[1] = (mDownloadState->dstOffset - 4) >> 16;
    buffer[2] = (mDownloadState->dstOffset - 4) >> 8;
    buffer[3] = (mDownloadState->dstOffset - 4);

    if (BL.blProgramShared(mDownloadState->start, buffer, 4, BL_FLASH_KEY1, BL_FLASH_KEY2) < 0) {
        mpuAllowRomWrite(false);
        mpuAllowRamExecution(false);
        return NANOHUB_FIRMWARE_CHUNK_REPLY_RESEND;
    }

    crc = ~crc32(mDownloadState->start, mDownloadState->dstOffset, ~0);
    padlen =  (4 - (mDownloadState->dstOffset & 3)) & 3;
    memset(buffer, 0x00, padlen);
    memcpy(&buffer[padlen], &crc, sizeof(uint32_t));
    mDownloadState->size = mDownloadState->dstOffset + padlen + sizeof(uint32_t);

    if (BL.blProgramShared(mDownloadState->start + mDownloadState->dstOffset, buffer, padlen + sizeof(uint32_t), BL_FLASH_KEY1, BL_FLASH_KEY2) < 0) {
        mpuAllowRomWrite(false);
        mpuAllowRamExecution(false);
        return NANOHUB_FIRMWARE_CHUNK_REPLY_RESEND;
    }

    mpuAllowRomWrite(false);
    mpuAllowRamExecution(false);
    heapFree(mDownloadState);
    mDownloadState = NULL;

    return NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED;
}

static void firmwareWrite(void *cookie)
{
    AppSecErr ret;

    if (mDownloadState->type == BL_FLASH_APP_ID)
        ret = appSecRxData(mDownloadState->appSecState, mDownloadState->data, mDownloadState->len);
    else
        ret = writeCbk(mDownloadState->data, mDownloadState->len);

    if (ret == APP_SEC_NO_ERROR) {
        if (mDownloadState->srcOffset == mDownloadState->size && mDownloadState->crc == ~mDownloadState->srcCrc) {
            mDownloadState->chunkReply = firmwareFinish(true);
        } else {
            mDownloadState->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED;
        }
    } else {
        mDownloadState->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_CANCEL;
    }
    hostIntfSetBusy(false);
}

static size_t firmwareChunk(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    uint32_t offset;
    uint8_t len;
    struct NanohubFirmwareChunkRequest *req = rx;
    struct NanohubFirmwareChunkResponse *resp = tx;

    offset = le32toh(req->offset);
    len = rx_len - sizeof(req->offset);

    if (!mDownloadState) {
        resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_CANCEL_NO_RETRY;
    } else if (mDownloadState->chunkReply != NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED) {
        resp->chunkReply = mDownloadState->chunkReply;
    } else if (mDownloadState->erase == true) {
        resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_WAIT;
        osDefer(firmwareErase, NULL, false);
    } else if (offset != mDownloadState->srcOffset) {
        resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_RESTART;
        resetDownloadState();
    } else {
        mDownloadState->srcCrc = crc32(req->data, len, mDownloadState->srcCrc);
        mDownloadState->srcOffset += len;
        if ((mDownloadState->srcOffset == mDownloadState->size && mDownloadState->crc != ~mDownloadState->srcCrc) || (mDownloadState->srcOffset > mDownloadState->size)) {
            firmwareFinish(false);
            resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_CANCEL;
        } else {
            memcpy(mDownloadState->data, req->data, len);
            mDownloadState->len = len;
            resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED;
            hostIntfSetBusy(true);
            osDefer(firmwareWrite, NULL, false);
        }
    }

    return sizeof(*resp);
}

static size_t getInterrupt(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubGetInterruptResponse *resp = tx;
    ATOMIC_BITSET_DECL(interrupts, MAX_INTERRUPTS,);

    hostIntfCopyClearInterrupts(interrupts, MAX_INTERRUPTS);
    memcpy(resp->interrupts, interrupts->words, sizeof(resp->interrupts));

    return sizeof(*resp);
}

static size_t maskInterrupt(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubMaskInterruptRequest *req = rx;
    struct NanohubMaskInterruptResponse *resp = tx;

    hostIntfSetInterruptMask(req->interrupt);

    resp->accepted = true;
    return sizeof(*resp);
}

static size_t unmaskInterrupt(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubUnmaskInterruptRequest *req = rx;
    struct NanohubUnmaskInterruptResponse *resp = tx;

    hostInfClearInterruptMask(req->interrupt);

    resp->accepted = true;
    return sizeof(*resp);
}

struct EvtPacket
{
    uint8_t sensType;
    uint8_t length;
    uint16_t pad;
    uint64_t timestamp;
    uint8_t data[NANOHUB_SENSOR_DATA_MAX];
} __attribute__((packed));

#define SYNC_DATAPOINTS 16
#define SYNC_RESET      10000000000ULL /* 10 seconds, ~100us drift */

struct TimeSync
{
    uint64_t lastTime;
    uint64_t delta[SYNC_DATAPOINTS];
    uint64_t avgDelta;
    uint8_t cnt;
    uint8_t tail;
} __attribute__((packed));

static uint64_t getAvgDelta(struct TimeSync *sync);

static void addDelta(struct TimeSync *sync, uint64_t apTime, uint64_t hubTime)
{
    if (apTime - sync->lastTime > SYNC_RESET) {
        sync->tail = 0;
        sync->cnt = 0;
    }

    sync->delta[sync->tail++] = apTime - hubTime;

    sync->lastTime = apTime;

    if (sync->tail >= SYNC_DATAPOINTS)
        sync->tail = 0;

    if (sync->cnt < SYNC_DATAPOINTS)
        sync->cnt ++;

    sync->avgDelta = 0ULL;
}

static uint64_t getAvgDelta(struct TimeSync *sync)
{
    int i;
    int32_t avg;

    if (!sync->cnt)
        return 0ULL;
    else if (!sync->avgDelta) {
        for (i=1, avg=0; i<sync->cnt; i++)
            avg += (int32_t)(sync->delta[i] - sync->delta[0]);
        sync->avgDelta = (avg / sync->cnt) + sync->delta[0];
    }
    return sync->avgDelta;
}

static size_t readEvent(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubReadEventRequest *req = rx;
    struct NanohubReadEventResponse *resp = tx;
    struct EvtPacket *packet = tx;
    int length, sensor;
    static struct TimeSync timeSync = { };

    addDelta(&timeSync, req->apBootTime, timestamp);

    if (hostIntfPacketDequeue(packet)) {
        length = packet->length + sizeof(resp->evtType);
        sensor = packet->sensType;
        // TODO combine messages if multiple can fit in a single packet
        if (sensor == SENS_TYPE_INVALID) {
#ifdef DEBUG_LOG_EVT
            resp->evtType = htole32(DEBUG_LOG_EVT);
#else
            resp->evtType = 0x00000000;
#endif
        } else {
            resp->evtType = htole32(EVT_NO_FIRST_SENSOR_EVENT + sensor);
            if (packet->timestamp)
                packet->timestamp += getAvgDelta(&timeSync);
        }
    } else {
        length = 0;
    }

    return length;
}

static size_t writeEvent(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubWriteEventRequest *req = rx;
    struct NanohubWriteEventResponse *resp = tx;
    uint8_t *packet = heapAlloc(rx_len - sizeof(req->evtType));

    memcpy(packet, req->evtData, rx_len - sizeof(req->evtType));
    resp->accepted = osEnqueueEvt(le32toh(req->evtType), packet, heapFree);
    if (!resp->accepted)
        heapFree(packet);

    return sizeof(*resp);
}

const static struct NanohubCommand mBuiltinCommands[] = {
        NANOHUB_COMMAND(NANOHUB_REASON_GET_OS_HW_VERSIONS,
                getOsHwVersion,
                struct NanohubOsHwVersionsRequest,
                struct NanohubOsHwVersionsRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_GET_APP_VERSIONS,
                getAppVersion,
                struct NanohubAppVersionsRequest,
                struct NanohubAppVersionsRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_QUERY_APP_INFO,
                queryAppInfo,
                struct NanohubAppInfoRequest,
                struct NanohubAppInfoRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_START_FIRMWARE_UPLOAD,
                startFirmwareUpload,
                struct NanohubStartFirmwareUploadRequest,
                struct NanohubStartFirmwareUploadRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_FIRMWARE_CHUNK,
                firmwareChunk,
                __le32,
                struct NanohubFirmwareChunkRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_GET_INTERRUPT,
                getInterrupt,
                struct NanohubGetInterruptRequest,
                struct NanohubGetInterruptRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_MASK_INTERRUPT,
                maskInterrupt,
                struct NanohubMaskInterruptRequest,
                struct NanohubMaskInterruptRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_UNMASK_INTERRUPT,
                unmaskInterrupt,
                struct NanohubUnmaskInterruptRequest,
                struct NanohubUnmaskInterruptRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_READ_EVENT,
                readEvent,
                struct NanohubReadEventRequest,
                struct NanohubReadEventRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_WRITE_EVENT,
                writeEvent,
                __le32,
                struct NanohubWriteEventRequest),
};

const struct NanohubCommand *nanohubFindCommand(uint32_t packetReason)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(mBuiltinCommands); i++) {
        const struct NanohubCommand *cmd = &mBuiltinCommands[i];
        if (cmd->reason == packetReason)
            return cmd;
    }
    return NULL;
}
