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
#include <plat/inc/bl.h>
#include <variant/inc/variant.h>

#define NANOHUB_COMMAND(_reason, _handler, _minReqType, _maxReqType) \
        { .reason = _reason, .handler = _handler, \
          .minDataLen = sizeof(_minReqType), .maxDataLen = sizeof(_maxReqType) }

static size_t getOsHwVersion(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    struct NanohubOsHwVersionsResponse *resp = tx;
    resp->hwType = htole16(platHwType());
    resp->hwVer = htole16(platHwVer());
    resp->blVer = htole16(platBlVer());
    resp->osVer = htole16(OS_VER);

    return sizeof(*resp);
}

static uint32_t mFirmwareSize;
static uint32_t mFirmwareOffset;
static uint8_t *mFirmwareStart;
static bool mFirmwareErase;

static size_t startFirmwareUpload(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    extern char __shared_start[];
    extern char __shared_end[];
    struct NanohubStartFirmwareUploadRequest *req = rx;
    struct NanohubStartFirmwareUploadResponse *resp = tx;
    uint8_t *shared_start = (uint8_t *)&__shared_start;
    uint8_t *shared_end = (uint8_t *)&__shared_end;
    uint8_t *shared;
    int len, total_len;

    mFirmwareSize = le32toh(req->size);

    for (shared = shared_start;
         shared < shared_end && shared[0] != 0xFF;
         shared += total_len) {
        len = (shared[1] << 16) | (shared[2] << 8) | shared[3];
        total_len = sizeof(uint32_t) + ((len + 3) & ~3) + sizeof(uint32_t);
    }

    if (shared + mFirmwareSize < shared_end) {
        mFirmwareStart = shared;
        mFirmwareErase = false;
    } else {
        mFirmwareStart = shared_start;
        mFirmwareErase = true;
    }
    mFirmwareOffset = 0;

    resp->accepted = 1;

    return sizeof(*resp);
}

static void firmwareErase(void *cookie)
{
    if (mFirmwareErase == true) {
        mpuAllowRamExecution(true);
        mpuAllowRomWrite(true);
        BL.blEraseShared(BL_FLASH_KEY1, BL_FLASH_KEY2);
        mpuAllowRomWrite(false);
        mpuAllowRamExecution(false);
        mFirmwareErase = false;
        hostIntfSetInterrupt(NANOHUB_INT_CMD_WAIT);
    }
}

static size_t firmwareChunk(void *rx, uint8_t rx_len, void *tx, uint64_t timestamp)
{
    uint32_t offset;
    uint8_t len;
    struct NanohubFirmwareChunkRequest *req = rx;
    struct NanohubFirmwareChunkResponse *resp = tx;

    offset = le32toh(req->offset);
    len = rx_len - sizeof(req->offset);

    if (mFirmwareErase == true) {
        resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_WAIT;
        osDefer(firmwareErase, NULL);
    } else if (offset != mFirmwareOffset) {
        resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_RESTART;
        mFirmwareOffset = 0;
    } else {
        mpuAllowRamExecution(true);
        mpuAllowRomWrite(true);
        if (BL.blProgramShared(mFirmwareStart + mFirmwareOffset, req->data, len, BL_FLASH_KEY1, BL_FLASH_KEY2) < 0) {
            resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_RESEND;
        } else {
            resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED;
            mFirmwareOffset += len;
        }
        mpuAllowRomWrite(false);
        mpuAllowRamExecution(false);
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

    if (!sync->cnt)
        return 0ULL;
    else if (!sync->avgDelta) {
        sync->avgDelta = 0;
        for (i=0; i<sync->cnt; i++)
            sync->avgDelta += sync->delta[i];
        sync->avgDelta /= sync->cnt;
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
