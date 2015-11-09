#include <inttypes.h>
#include <stdint.h>
#include <sys/endian.h>
#include <string.h>

#include <plat/inc/plat.h>
#include <plat/inc/pwr.h>
#include <variant/inc/variant.h>

#include <platform.h>
#include <crc.h>
#include <hostIntf.h>
#include <hostIntf_priv.h>
#include <nanohubCommand.h>
#include <nanohubPacket.h>
#include <seos.h>
#include <util.h>
#include <atomicBitset.h>
#include <atomic.h>
#include <gpio.h>
#include <apInt.h>
#include <sensors.h>
#include <slab.h>
#include <timer.h>

struct ConfigCmd
{
    uint64_t latency;
    uint32_t rate;
    uint8_t sensorType;
    uint8_t enabled : 1;
    uint8_t flush : 1;
    uint8_t calibrate : 1;
} __attribute__((packed));

struct ActiveSensor
{
    uint64_t lastInterrupt;
    uint64_t latency;
    uint32_t rate;
    uint8_t sensorType;
    uint8_t numAxis;
    uint8_t interrupt;
    uint8_t reserved[2];
    uint32_t sensorHandle;
} __attribute__((packed));

static uint8_t mSensorList[SENS_TYPE_LAST_USER];
static struct EvtQueue *mEvtsExternal;
static struct SlabAllocator *mTxSlab, *mActiveSensorSlab;

static const struct HostIntfComm *gComm;
static uint8_t gRxBuf[NANOHUB_PACKET_SIZE_MAX];
static size_t gRxSize;
static bool gTxRetrans;
static struct
{
    uint8_t prePreamble;
    uint8_t buf[NANOHUB_PACKET_SIZE_MAX];
    uint8_t postPreamble;
} gTxBuf = { .prePreamble = NANOHUB_PREAMBLE_BYTE, .postPreamble = NANOHUB_PREAMBLE_BYTE };
static size_t gTxSize;
static uint8_t *gTxBufPtr;
static uint32_t gSeq;
static const struct NanohubCommand *gRxCmd;
ATOMIC_BITSET_DECL(gInterrupt, MAX_INTERRUPTS, static);
ATOMIC_BITSET_DECL(gInterruptMask, MAX_INTERRUPTS, static);
static uint32_t gHostIntfTid;

static void hostIntfRxPacket();
static void hostIntfTxPacket(uint32_t reason, uint8_t len,
        HostIntfCommCallbackF callback);

static void hostIntfRxDone(size_t rx, int err);
static void hostIntfGenerateAck(void *cookie);

static void hostIntfTxAckDone(size_t tx, int err);
static void hostIntfGenerateResponse(void *cookie);

static void hostIntfTxPayloadDone(size_t tx, int err);

static void handleEventFreeing(uint32_t evtType, void *evtData, uintptr_t evtFreeData) // watch out, this is synchronous
{
    if (!evtFreeData)
        return;

    ((EventFreeF)taggedPtrToPtr(evtFreeData))(evtData);
}

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

    if (gSeq == packet->seq) {
        gTxRetrans = true;
        return gRxCmd;
    } else {
        gTxRetrans = false;
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

static void hostIntfTxBuf(int size, uint8_t *buf, HostIntfCommCallbackF callback)
{
    gTxSize = size;
    gTxBufPtr = buf;
    gComm->txPacket(gTxBufPtr, gTxSize, callback);
}

static void hostIntfTxPacket(__le32 reason, uint8_t len,
        HostIntfCommCallbackF callback)
{
    struct NanohubPacket *txPacket = (struct NanohubPacket *)(gTxBuf.buf);
    txPacket->reason = reason;
    txPacket->seq = gSeq;
    txPacket->sync = NANOHUB_SYNC_BYTE;
    txPacket->len = len;

    struct NanohubPacketFooter *txFooter = hostIntfGetFooter(gTxBuf.buf);
    txFooter->crc = hostIntfComputeCrc(gTxBuf.buf);

    // send starting with the prePremable byte
    hostIntfTxBuf(1+NANOHUB_PACKET_SIZE(len), &gTxBuf.prePreamble, callback);
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

static bool hostIntfRequest(uint32_t tid)
{
    gHostIntfTid = tid;
    atomicBitsetInit(gInterrupt, MAX_INTERRUPTS);
    atomicBitsetInit(gInterruptMask, MAX_INTERRUPTS);
    hostIntfSetInterruptMask(NANOHUB_INT_NONWAKEUP);

    gComm = platHostIntfInit();
    if (gComm) {
        int err = gComm->request();
        if (!err) {
            hostIntfRxPacket();
            mEvtsExternal = evtQueueAlloc(64, handleEventFreeing);
            // TODO: Can possibly switch mTxSlab to heap. Need to make sure
            // fragmentation doesn't negatively impact performance.
            // (hopefully should be ok cause the allocations are large)
            mTxSlab = slabAllocatorNew(255, 4, 66);
            mActiveSensorSlab = slabAllocatorNew(sizeof(struct ActiveSensor), 4, MAX_REGISTERED_SENSORS);
            memset(mSensorList, 0xFF, sizeof(mSensorList));
            osEventSubscribe(tid, EVT_NO_SENSOR_CONFIG_EVENT);
#ifdef DEBUG_LOG_EVT
            osEventSubscribe(tid, DEBUG_LOG_EVT);
#endif
            return true;
        }
    }

    return false;
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
    gRxCmd = hostIntfFindHandler(gRxBuf, gRxSize);

    if (gRxCmd) {
        if (gTxRetrans)
            hostIntfTxBuf(gTxSize, &gTxBuf.prePreamble, hostIntfTxPayloadDone);
        else
            hostIntfTxPacket(NANOHUB_REASON_ACK, 0, hostIntfTxAckDone);
    } else {
        hostIntfTxPacket(NANOHUB_REASON_NAK, 0, hostIntfTxAckDone);
    }
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
    void *txPayload = hostIntfGetPayload(gTxBuf.buf);
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

static void hostIntfRelease()
{
    gComm->release();
}

void hostIntfPacketFree(void *ptr)
{
    slabAllocatorFree(mTxSlab, ptr);
}

bool hostIntfPacketDequeue(void **ptr, int *length, int *sensor)
{
    bool ret;
    uintptr_t free;
    uint32_t evtType;

    ret = evtQueueDequeue(mEvtsExternal, &evtType, ptr, &free, false);

    *length = evtType & 0xFF;
    *sensor = (evtType >> 8) & 0xFF;

    return ret;
}

static void hostIntfHandleEvent(uint32_t evtType, const void* evtData)
{
    struct ConfigCmd *cmd;
    int i;
    uint64_t currentTime;
    int length;
    bool discard;
    uint8_t *data;
    struct TripleAxisDataEvent *triple;
    struct SingleAxisDataEvent *single;
    struct ActiveSensor *sensor;
    const struct SensorInfo *si;

#ifdef DEBUG_LOG_EVT
    if (evtType == DEBUG_LOG_EVT) {
        data = slabAllocatorAlloc(mTxSlab);
        discard = true;
        length = *(uint8_t *)evtData;
        memcpy(data, (uint8_t *)evtData+1, length);
        if (!evtQueueEnqueue(mEvtsExternal, (discard << 31) | ((SENS_TYPE_INVALID & 0xFF) << 8) | (length & 0xFF), data, taggedPtrMakeFromPtr(hostIntfPacketFree)))
            hostIntfPacketFree(data);
    } else
#endif
    if (evtType == EVT_NO_SENSOR_CONFIG_EVENT) { // config
        cmd = (struct ConfigCmd *)evtData;
        if (mSensorList[cmd->sensorType] < MAX_REGISTERED_SENSORS) {
            sensor = slabAllocatorGetNth(mActiveSensorSlab, mSensorList[cmd->sensorType]);
            if (cmd->flush) {
                sensorFlush(sensor->sensorHandle);
            } else if (cmd->calibrate) {
                /* sensorCalibrate(sensor->sensorHandle); */
            } else if (cmd->enabled) {
                sensorRequestRateChange(gHostIntfTid, sensor->sensorHandle, cmd->rate, cmd->latency);
            } else {
                sensorRelease(gHostIntfTid, sensor->sensorHandle);
                osEventUnsubscribe(gHostIntfTid, sensorGetMyEventType(cmd->sensorType));
                mSensorList[cmd->sensorType] = 0xFF;
                sensor->sensorType = SENS_TYPE_INVALID;
                slabAllocatorFree(mActiveSensorSlab, sensor);
            }
        } else if (cmd->enabled && (sensor = slabAllocatorAlloc(mActiveSensorSlab))) {
            for (i=0; (si = sensorFind(cmd->sensorType, i, &sensor->sensorHandle)) != NULL; i++) {
                if (sensorRequest(gHostIntfTid, sensor->sensorHandle, cmd->rate, cmd->latency)) {
                    mSensorList[cmd->sensorType] = slabAllocatorGetIndex(mActiveSensorSlab, sensor);
                    sensor->sensorType = cmd->sensorType;
                    sensor->numAxis = si->numAxis;
                    sensor->interrupt = si->interrupt;
                    sensor->rate = cmd->rate;
                    sensor->latency = cmd->latency;
                    osEventSubscribe(gHostIntfTid, sensorGetMyEventType(cmd->sensorType));
                    break;
                } else {
                    si = NULL;
                }
            }
            if (!si)
                slabAllocatorFree(mActiveSensorSlab, sensor);
        }
    } else if (evtType >= EVT_NO_FIRST_SENSOR_EVENT && evtType < EVT_NO_SENSOR_CONFIG_EVENT) { // data
        if (mSensorList[evtType & 0xFF] < MAX_REGISTERED_SENSORS) {
            currentTime = timGetTime();
            data = slabAllocatorAlloc(mTxSlab);
            if (!data) {
                osLog(LOG_INFO, "slabAllocator returned NULL\n");
                return;
            }
            sensor = slabAllocatorGetNth(mActiveSensorSlab, mSensorList[evtType & 0xFF]);
            // TODO: batch multiple samples from the same sensor
            // TODO: also split if the incoming data is too big
            if (evtData == SENSOR_DATA_EVENT_FLUSH) {
                switch (sensor->numAxis) {
                case NUM_AXIS_EMBEDDED:
                case NUM_AXIS_ONE:
                    single = (struct SingleAxisDataEvent *)data;
                    discard = false;
                    length = sizeof(struct SingleAxisDataEvent);
                    single->referenceTime = 0ull;
                    break;
                case NUM_AXIS_THREE:
                    triple = (struct TripleAxisDataEvent *)data;
                    discard = false;
                    length = sizeof(struct TripleAxisDataEvent);
                    triple->referenceTime = 0ull;
                    break;
                default:
                    slabAllocatorFree(mTxSlab, data);
                    return;
                }
            } else {
                switch (sensor->numAxis) {
                case NUM_AXIS_EMBEDDED:
                    single = (struct SingleAxisDataEvent *)data;
                    discard = true;
                    length = sizeof(struct SingleAxisDataEvent) + sizeof(struct SingleAxisDataPoint);
                    single->referenceTime = currentTime;
                    single->samples[0].numSamples = 1;
                    single->samples[0].idata = (uint32_t)evtData;
                    break;
                case NUM_AXIS_ONE:
                    single = (struct SingleAxisDataEvent *)evtData;
                    discard = true;
                    length = sizeof(struct SingleAxisDataEvent) + single->samples[0].numSamples * sizeof(struct SingleAxisDataPoint);
                    memcpy(data, single, length);
                    break;
                case NUM_AXIS_THREE:
                    triple = (struct TripleAxisDataEvent *)evtData;
                    discard = true;
                    length = sizeof(struct TripleAxisDataEvent) + triple->samples[0].numSamples * sizeof(struct TripleAxisDataPoint);
                    memcpy(data, triple, length);
                    break;
                default:
                    slabAllocatorFree(mTxSlab, data);
                    return;
                }
            }

            if (!evtQueueEnqueue(mEvtsExternal, (discard << 31) | ((sensor->sensorType & 0xFF) << 8) | (length & 0xFF), data, taggedPtrMakeFromPtr(hostIntfPacketFree))) {
                hostIntfPacketFree(data);
            } else {
                // TODO: handle mismatch latency between hostIntf and sensor
                // sensorGetCurLatency(sensor->sensorHandle) != sensor->latency
                if (currentTime >= sensor->lastInterrupt + sensor->latency) {
#if 0 // TODO: when non-wakeup sensors are enabled/disabled over AP suspend
                    hostIntfSetInterrupt(sensor->interrupt);
#else
                    hostIntfSetInterrupt(NANOHUB_INT_WAKEUP);
#endif
                    sensor->lastInterrupt = currentTime;
                }
            }
        }
    }
}

void hostIntfCopyClearInterrupts(struct AtomicBitset *dst, uint32_t numBits)
{
    atomicBitsetInit(dst, numBits);

    apIntClear(false);
    apIntClear(true);

    atomicBitsetXchg(gInterrupt, dst);
}

void hostIntfSetInterrupt(uint32_t bit)
{
    atomicBitsetSetBit(gInterrupt, bit);
    if (!atomicBitsetGetBit(gInterruptMask, bit)) {
        platRequestDevInSleepMode(Stm32sleepWakeup, 12);
        apIntSet(true);
    } else {
        apIntSet(false);
    }
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

INTERNAL_APP_INIT(0x0000000000000001, hostIntfRequest, hostIntfRelease, hostIntfHandleEvent);
