#include <plat/inc/taggedPtr.h>
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

static size_t hostIntfGetOsHwVersion(void *rx, uint8_t, void *tx);
static size_t hostIntfStartFirmwareUpload(void *rx, uint8_t, void *tx);
static size_t hostIntfFirmwareChunk(void *rx, uint8_t, void *tx);
static size_t hostIntfGetInterrupt(void *rx, uint8_t, void *tx);
static size_t hostIntfReadEvent(void *rx, uint8_t, void *tx);
static size_t hostIntfWriteEvent(void *rx, uint8_t, void *tx);

const struct NanohubCommand gBuiltinCommands[] = {
        NANOHUB_COMMAND(NANOHUB_REASON_GET_OS_HW_VERSIONS,
                hostIntfGetOsHwVersion,
                struct NanohubOsHwVersionsRequest,
                struct NanohubOsHwVersionsRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_START_FIRMWARE_UPLOAD,
                hostIntfStartFirmwareUpload,
                struct NanohubStartFirmwareUploadRequest,
                struct NanohubStartFirmwareUploadRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_FIRMWARE_CHUNK,
                hostIntfFirmwareChunk,
                __le32,
                struct NanohubFirmwareChunkRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_GET_INTERRUPT,
                hostIntfGetInterrupt,
                struct NanohubGetInterruptRequest,
                struct NanohubGetInterruptRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_READ_EVENT,
                hostIntfReadEvent,
                struct NanohubReadEventRequest,
                struct NanohubReadEventRequest),
        NANOHUB_COMMAND(NANOHUB_REASON_WRITE_EVENT,
                hostIntfWriteEvent,
                __le32,
                struct NanohubWriteEventRequest),
};

static size_t hostIntfGetOsHwVersion(void *rx, uint8_t rx_len, void *tx)
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

static size_t hostIntfStartFirmwareUpload(void *rx, uint8_t rx_len, void *tx)
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

static void hostIntfFirmwareErase(void *cookie)
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

static size_t hostIntfFirmwareChunk(void *rx, uint8_t rx_len, void *tx)
{
    uint32_t offset;
    uint8_t len;
    struct NanohubFirmwareChunkRequest *req = rx;
    struct NanohubFirmwareChunkResponse *resp = tx;

    offset = le32toh(req->offset);
    len = rx_len - sizeof(req->offset);

    if (mFirmwareErase == true) {
        resp->chunkReply = NANOHUB_FIRMWARE_CHUNK_REPLY_WAIT;
        osDefer(hostIntfFirmwareErase, NULL);
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

static size_t hostIntfGetInterrupt(void *rx, uint8_t rx_len, void *tx)
{
    struct NanohubGetInterruptResponse *resp = tx;
    ATOMIC_BITSET_DECL(interrupts, MAX_INTERRUPTS,);

    hostIntfCopyClearInterrupts(interrupts, MAX_INTERRUPTS);
    memcpy(resp->interrupts, interrupts->words, sizeof(resp->interrupts));

    return sizeof(*resp);
}

struct EvtPacket
{
    uint64_t timestamp;
    uint8_t data[];
};

static size_t hostIntfReadEvent(void *rx, uint8_t rx_len, void *tx)
{
    struct NanohubReadEventRequest *req = rx;
    struct NanohubReadEventResponse *resp = tx;
    int length, sensor;
    struct EvtPacket *packet;
    uint64_t currTime = timGetTime();

    if (hostIntfPacketDequeue((void **)&packet, &length, &sensor)) {
        length += sizeof(resp->evtType);
        // TODO combine messages if multiple can fit in a single packet
        if (sensor == SENS_TYPE_INVALID) {
#ifdef DEBUG_LOG_EVT
            resp->evtType = DEBUG_LOG_EVT;
#else
            resp->evtType = 0x00000000;
#endif
        } else {
            resp->evtType = htole32(EVT_NO_FIRST_SENSOR_EVENT + sensor);
            if (packet->timestamp != 0ull)
                packet->timestamp += req->apBootTime - currTime;
        }
        memcpy(resp->evtData, packet, length);

        hostIntfPacketFree(packet);
    } else {
        length = 0;
    }

    return length;
}

static size_t hostIntfWriteEvent(void *rx, uint8_t rx_len, void *tx)
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

const struct NanohubCommand *nanohubFindCommand(uint32_t packetReason)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(gBuiltinCommands); i++) {
        const struct NanohubCommand *cmd = &gBuiltinCommands[i];
        if (cmd->reason == packetReason)
            return cmd;
    }
    return NULL;
}
