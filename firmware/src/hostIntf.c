#include <inttypes.h>
#include <stdint.h>
#include <sys/endian.h>

#include <crc.h>
#include <i2c.h>
#include <hostIntf.h>
#include <hostIntf_priv.h>
#include <nanohubPacket.h>
#include <plat/inc/pwr.h>
#include <seos.h>
#include <util.h>

#define NANOHUB_I2C_SLAVE_ADDRESS     0x55

struct NanohubCommand {
    uint32_t reason;
    size_t (*handler)(void *);
    uint8_t dataLen;
};

#define NANOHUB_COMMAND(_reason, _handler, _reqType) \
        { .reason = _reason, .handler = _handler, .dataLen = sizeof(_reqType) }

static size_t hostIntfGetOsHwVersion(void *payload);

const struct NanohubCommand gBuiltinCommands[] = {
        NANOHUB_COMMAND(NANOHUB_REASON_GET_OS_HW_VERSIONS,
                hostIntfGetOsHwVersion,
                struct NanohubOsHwVersionsRequest),
};


static I2cBus gI2cBusId;
static uint8_t gRxBuf[NANOHUB_PACKET_SIZE_MAX];
static size_t gRxSize;
static uint8_t gTxBuf[NANOHUB_PACKET_SIZE_MAX];
static size_t gTxSize;
static uint8_t *gTxBufPtr;
static uint32_t gSeq;
static const struct NanohubCommand *gRxCmd;

static void hostIntfTxPacket(uint32_t reason, uint8_t len,
        I2cCallbackF callback);

static void hostIntfRxDone(void *cookie, size_t tx, size_t rx, int err);
static void hostIntfGenerateAck(void *cookie);

static void hostIntfTxAckDone(void *cookie, size_t tx, size_t rx, int err);
static void hostIntfGenerateResponse(void *cookie);

static void hostIntfTxPayloadDone(void *cookie, size_t tx, size_t rx, int err);
static void hostIntfTxPreambleDone(void *cookie, size_t tx, size_t rx, int err);


static inline void *hostIntfGetPayload(uint8_t *buf)
{
    struct NanohubPacket *packet = (struct NanohubPacket *)buf;
    return packet->data;
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
    size_t i;

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
    for (i = 0; i < ARRAY_SIZE(gBuiltinCommands); i++) {
        const struct NanohubCommand *cmd = &gBuiltinCommands[i];
        if (cmd->reason != packetReason)
            continue;

        if (cmd->dataLen != packet->len) {
            osLog(LOG_WARN, "%s: payload size mismatch (reason = %08" PRIx32 ", sizeof(payload) = %zu, packet->len = %zu)\n",
                    __func__, cmd->reason, cmd->dataLen, packet->len);
            return NULL;
        }

        return cmd;
    }

    osLog(LOG_WARN, "%s: unknown reason %08" PRIx32 "\n",
            __func__, packetReason);
    return NULL;
}

static size_t hostIntfGetOsHwVersion(void *payload)
{
    struct NanohubOsHwVersionsResponse *resp = payload;
    resp->hwType = htole16(platHwType());
    resp->hwVer = htole16(platHwVer());
    resp->blVer = htole16(platBlVer());
    resp->osVer = htole16(OS_VER);

    return sizeof(*resp);
}

static void hostIntfTxPacket(__le32 reason, uint8_t len, I2cCallbackF callback)
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
    i2cSlaveTxPacket(gI2cBusId, gTxBufPtr, gTxSize, callback, NULL);
}

static inline void hostIntfTxPacketDone(int err, size_t tx,
        I2cCallbackF callback)
{
    if (err < 0 || tx >= gTxSize) {
        i2cSlaveTxPreamble(gI2cBusId, NANOHUB_PREAMBLE_BYTE,
                hostIntfTxPreambleDone, NULL);
    } else {
        gTxSize -= tx;
        gTxBufPtr += tx;

        i2cSlaveTxPacket(gI2cBusId, gTxBufPtr, gTxSize, callback, NULL);
    }
}

void hostIntfRequest()
{
    gI2cBusId = platHostIntfI2cBus();
    i2cSlaveRequest(gI2cBusId, NANOHUB_I2C_SLAVE_ADDRESS);
    i2cSlaveEnableRx(gI2cBusId, gRxBuf, sizeof(gRxBuf), hostIntfRxDone, NULL);
}

static void hostIntfRxDone(void *cookie, size_t tx, size_t rx, int err)
{
    gRxSize = rx;
    i2cSlaveTxPreamble(gI2cBusId, NANOHUB_PREAMBLE_BYTE,
            hostIntfTxPreambleDone, NULL);

    if (err != 0) {
        osLog(LOG_ERROR, "%s: failed to receive request: %d\n", __func__, err);
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

static void hostIntfTxAckDone(void *cookie, size_t tx, size_t rx, int err)
{
    hostIntfTxPacketDone(err, tx, hostIntfTxAckDone);

    if (err) {
        osLog(LOG_ERROR, "%s: failed to ACK request: %d\n", __func__, err);
        return;
    }
    if (!gRxCmd) {
        osLog(LOG_DEBUG, "%s: NACKed invalid request\n", __func__);
        return;
    }

    osDefer(hostIntfGenerateResponse, NULL);
}

static void hostIntfGenerateResponse(void *cookie)
{
    void *txPayload = hostIntfGetPayload(gTxBuf);
    uint8_t respLen = gRxCmd->handler(txPayload);

    hostIntfTxPacket(gRxCmd->reason, respLen, hostIntfTxPayloadDone);
}

static void hostIntfTxPayloadDone(void *cookie, size_t tx, size_t rx, int err)
{
    hostIntfTxPacketDone(err, tx, hostIntfTxPayloadDone);

    if (err)
        osLog(LOG_ERROR, "%s: failed to send response: %d\n", __func__, err);
}

static void hostIntfTxPreambleDone(void *cookie, size_t tx, size_t rx, int err)
{
}

void hostIntfRelease()
{
    i2cSlaveRelease(gI2cBusId);
}
