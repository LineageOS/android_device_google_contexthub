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
#include <plat/inc/bl.h>

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

static size_t hostIntfReadEvent(void *rx, uint8_t rx_len, void *tx)
{
    struct NanohubReadEventResponse *resp = tx;
    uint8_t length = 0;
    TaggedPtr evtFreeInfo;
    uint32_t evtType;
    void *evtData;
    uint8_t *packet;

    if (osDequeueExtEvt(&evtType, &evtData, &evtFreeInfo)) {
        packet = (uint8_t *)evtData;
        length = sizeof(resp->evtType) + *packet;
        resp->evtType = htole32(evtType);
        memcpy(resp->evtData, packet+1, length);

        if (evtFreeInfo)
            ((EventFreeF)(taggedPtrToPtr(evtFreeInfo)))(evtData); //this is a bad hack that *WILL* break soon. Good!
    }

    return length;
}

static size_t hostIntfWriteEvent(void *rx, uint8_t rx_len, void *tx)
{
    struct NanohubWriteEventRequest *req = rx;
    struct NanohubWriteEventResponse *resp = tx;
    uint8_t *packet = heapAlloc(rx_len - sizeof(req->evtType));

    memcpy(packet, req->evtData, rx_len - sizeof(req->evtType));
    resp->accepted = osEnqueueEvt(le32toh(req->evtType), packet, heapFree, false);
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
