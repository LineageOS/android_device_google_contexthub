#include <inttypes.h>
#include <stdint.h>
#include <sys/endian.h>

#include <crc.h>
#include <hostIntf.h>
#include <hostIntf_priv.h>
#include <nanohubCommand.h>
#include <nanohubPacket.h>
#include <plat/inc/pwr.h>
#include <seos.h>
#include <util.h>
#include <atomicBitset.h>
#include <atomic.h>
#include <gpio.h>
#include <apInt.h>

static const struct HostIntfComm *gComm;

static uint8_t gRxBuf[NANOHUB_PACKET_SIZE_MAX];
static size_t gRxSize;
static uint8_t gTxBuf[NANOHUB_PACKET_SIZE_MAX + 1];
static size_t gTxSize;
static uint8_t *gTxBufPtr;
static uint32_t gSeq;
static const struct NanohubCommand *gRxCmd;
ATOMIC_BITSET_DECL(gInterrupt, MAX_INTERRUPTS, static);
ATOMIC_BITSET_DECL(gInterruptMask, MAX_INTERRUPTS, static);

static void hostIntfRxPacket();
static void hostIntfTxPacket(uint32_t reason, uint8_t len,
        HostIntfCommCallbackF callback);

static void hostIntfRxDone(size_t rx, int err);
static void hostIntfGenerateAck(void *cookie);

static void hostIntfTxAckDone(size_t tx, int err);
static void hostIntfGenerateResponse(void *cookie);

static void hostIntfTxPayloadDone(size_t tx, int err);

static inline void *hostIntfGetPayload(uint8_t *buf)
{
    struct NanohubPacket *packet = (struct NanohubPacket *)buf;
    return packet->data;
}

static inline uint8_t hostIntfGetPayloadLen(uint8_t *buf)
{
    struct NanohubPacket *packet = (struct NanohubPacket *)buf;
    return packet->len;
}

static inline struct NanohubPacketFooter *hostIntfGetFooter(uint8_t *buf)
{
    struct NanohubPacket *packet = (struct NanohubPacket *)buf;
    return (struct NanohubPacketFooter *)(buf + sizeof(*packet) + packet->len);
}

static inline __le32 hostIntfComputeCrc(uint8_t *buf)
{
    struct NanohubPacket *packet = (struct NanohubPacket *)buf;
    uint32_t crc = crc32(packet, packet->len + sizeof(*packet));
    return htole32(crc);
}

static inline const struct NanohubCommand *hostIntfFindHandler(uint8_t *buf, size_t size)
{
    struct NanohubPacket *packet = (struct NanohubPacket *)buf;
    struct NanohubPacketFooter *footer;
    __le32 packetCrc;
    uint32_t packetReason;
    const struct NanohubCommand *cmd;

    if (size < NANOHUB_PACKET_SIZE(0)) {
        osLog(LOG_WARN, "%s: received incomplete packet (size = %zu)\n", __func__, size);
        return NULL;
    }

    if (size != NANOHUB_PACKET_SIZE(packet->len)) {
        osLog(LOG_WARN, "%s: size mismatch (size = %zu, packet->len = %zu)\n",
                __func__, size, NANOHUB_PACKET_SIZE(packet->len));
        return NULL;
    }

    footer = hostIntfGetFooter(buf);
    packetCrc = hostIntfComputeCrc(buf);
    if (footer->crc != packetCrc) {
        osLog(LOG_WARN, "%s: CRC mismatch (calculated %08" PRIx32 ", footer->crc = %08" PRIx32 ")\n",
                __func__, le32toh(packetCrc), le32toh(footer->crc));
        return NULL;
    }

    gSeq = packet->seq;
    packetReason = le32toh(packet->reason);

    if ((cmd = nanohubFindCommand(packetReason)) != NULL) {
        if (packet->len < cmd->minDataLen || packet->len > cmd->maxDataLen) {
            osLog(LOG_WARN, "%s: payload size mismatch (reason = %08" PRIx32 ", min sizeof(payload) = %zu, max sizeof(payload) = %zu, packet->len = %zu)\n",
                    __func__, cmd->reason, cmd->minDataLen, cmd->maxDataLen,
                    packet->len);
            return NULL;
        }

        return cmd;
    }

    osLog(LOG_WARN, "%s: unknown reason %08" PRIx32 "\n",
            __func__, packetReason);
    return NULL;
}

static void hostIntfTxPacket(__le32 reason, uint8_t len,
        HostIntfCommCallbackF callback)
{
    struct NanohubPacket *txPacket = (struct NanohubPacket *)gTxBuf;
    txPacket->reason = reason;
    txPacket->seq = gSeq;
    txPacket->sync = NANOHUB_SYNC_BYTE;
    txPacket->len = len;

    struct NanohubPacketFooter *txFooter = hostIntfGetFooter(gTxBuf);
    txFooter->crc = hostIntfComputeCrc(gTxBuf);

    gTxSize = NANOHUB_PACKET_SIZE(len);
    gTxBufPtr = gTxBuf;
    gComm->txPacket(gTxBufPtr, gTxSize, callback);
}

static inline void hostIntfTxPacketDone(int err, size_t tx,
        HostIntfCommCallbackF callback)
{
    if (!err && tx < gTxSize) {
        gTxSize -= tx;
        gTxBufPtr += tx;

        gComm->txPacket(gTxBufPtr, gTxSize, callback);
    }
}

void hostIntfRequest()
{
    atomicBitsetInit(gInterrupt, MAX_INTERRUPTS);
    atomicBitsetInit(gInterruptMask, MAX_INTERRUPTS);
    gComm = platHostIntfInit();
    if (gComm) {
        int err = gComm->request();
        if (!err)
            hostIntfRxPacket();
    }
}

static inline void hostIntfRxPacket()
{
    gComm->rxPacket(gRxBuf, sizeof(gRxBuf), hostIntfRxDone);
}

static void hostIntfRxDone(size_t rx, int err)
{
    gRxSize = rx;

    if (err != 0) {
        osLog(LOG_ERROR, "%s: failed to receive request: %d\n", __func__, err);
        hostIntfRxPacket();
        return;
    }

    osDefer(hostIntfGenerateAck, NULL);
}

static void hostIntfGenerateAck(void *cookie)
{
    uint32_t reason;

    gRxCmd = hostIntfFindHandler(gRxBuf, gRxSize);
    if (gRxCmd)
        reason = NANOHUB_REASON_ACK;
    else
        reason = NANOHUB_REASON_NAK;

    hostIntfTxPacket(reason, 0, hostIntfTxAckDone);
}

static void hostIntfTxAckDone(size_t tx, int err)
{
    hostIntfTxPacketDone(err, tx, hostIntfTxAckDone);

    if (err) {
        osLog(LOG_ERROR, "%s: failed to ACK request: %d\n", __func__, err);
        hostIntfRxPacket();
        return;
    }
    if (!gRxCmd) {
        osLog(LOG_DEBUG, "%s: NACKed invalid request\n", __func__);
        hostIntfRxPacket();
        return;
    }

    osDefer(hostIntfGenerateResponse, NULL);
}

static void hostIntfGenerateResponse(void *cookie)
{
    void *rxPayload = hostIntfGetPayload(gRxBuf);
    uint8_t rx_len = hostIntfGetPayloadLen(gRxBuf);
    void *txPayload = hostIntfGetPayload(gTxBuf);
    uint8_t respLen = gRxCmd->handler(rxPayload, rx_len, txPayload);

    hostIntfTxPacket(gRxCmd->reason, respLen, hostIntfTxPayloadDone);
}

static void hostIntfTxPayloadDone(size_t tx, int err)
{
    hostIntfTxPacketDone(err, tx, hostIntfTxPayloadDone);

    if (err)
        osLog(LOG_ERROR, "%s: failed to send response: %d\n", __func__, err);

    hostIntfRxPacket();
}

void hostIntfRelease()
{
    gComm->release();
}

void hostIntfCopyClearInterrupts(struct AtomicBitset *dst)
{
    atomicBitsetInit(dst, dst->numBits);

    apIntClear(false);
    apIntClear(true);

    atomicBitsetXchg(gInterrupt, dst);
}

void hostIntfSetInterrupt(uint32_t bit)
{
    atomicBitsetSetBit(gInterrupt, bit);
    apIntSet(!atomicBitsetGetBit(gInterruptMask, bit));
}

void hostInfClearInterrupt(uint32_t bit)
{
    atomicBitsetClearBit(gInterrupt, bit);
}

void hostIntfSetInterruptMask(uint32_t bit)
{
    atomicBitsetSetBit(gInterruptMask, bit);
}

void hostInfClearInterruptMask(uint32_t bit)
{
    atomicBitsetClearBit(gInterruptMask, bit);
}
