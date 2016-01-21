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
#include <timer.h>
#include <heap.h>
#include <simpleQ.h>

#define MAX_NUM_BLOCKS      350 /* times 252 = 88200 bytes */
#define SENSOR_INIT_DELAY   500000000 /* ns */

enum ConfigCmds
{
    CONFIG_CMD_DISABLE      = 0,
    CONFIG_CMD_ENABLE       = 1,
    CONFIG_CMD_FLUSH        = 2,
    CONFIG_CMD_CFG_DATA     = 3,
    CONFIG_CMD_CALIBRATE    = 4,
};

struct ConfigCmd
{
    uint64_t latency;
    uint32_t rate;
    uint8_t sensType;
    uint8_t cmd;
    uint16_t flags;
} __attribute__((packed));

struct DataBuffer
{
    uint8_t sensType;
    uint8_t length;
    uint16_t pad;
    uint64_t referenceTime;
    union
    {
        struct SensorFirstSample firstSample;
        struct SingleAxisDataPoint single[NANOHUB_SENSOR_DATA_MAX / sizeof(struct SingleAxisDataPoint)];
        struct TripleAxisDataPoint triple[NANOHUB_SENSOR_DATA_MAX / sizeof(struct TripleAxisDataPoint)];
        struct WifiScanResult wifiScanResults[NANOHUB_SENSOR_DATA_MAX / sizeof(struct WifiScanResult)];
        uint8_t buffer[NANOHUB_SENSOR_DATA_MAX];
    };
} __attribute__((packed));

struct ActiveSensor
{
    uint64_t lastInterrupt;
    uint64_t lastTimestamp;
    uint64_t latency;
    uint64_t lastTime;
    struct DataBuffer buffer;
    uint32_t rate;
    uint32_t sensorHandle;
    uint16_t minSamples;
    uint16_t curSamples;
    uint8_t numAxis;
    uint8_t interrupt;
    uint8_t numSamples;
    uint8_t packetSamples;
    uint8_t oneshot : 1;
    uint8_t discard : 1;
    uint8_t reserved : 6;
} __attribute__((packed));

static uint8_t mSensorList[SENS_TYPE_LAST_USER];
static struct SimpleQueue *mOutputQ;
static struct ActiveSensor *mActiveSensorTable;
static uint8_t mNumSensors;
static uint8_t mLastSensor;

static const struct HostIntfComm *mComm;
static bool mBusy;
static uint64_t mRxTimestamp;
static uint8_t mRxBuf[NANOHUB_PACKET_SIZE_MAX];
static size_t mRxSize;
static bool mTxRetrans;
static struct
{
    uint8_t pad; // packet header is 10 bytes. + 2 to word align
    uint8_t prePreamble;
    uint8_t buf[NANOHUB_PACKET_SIZE_MAX];
    uint8_t postPreamble;
} mTxBuf;
static size_t mTxSize;
static uint8_t *mTxBufPtr;
static uint32_t mSeq;
static const struct NanohubCommand *mRxCmd;
ATOMIC_BITSET_DECL(mInterrupt, MAX_INTERRUPTS, static);
ATOMIC_BITSET_DECL(mInterruptMask, MAX_INTERRUPTS, static);
static uint32_t mHostIntfTid;

static void hostIntfRxPacket();
static void hostIntfTxPacket(uint32_t reason, uint8_t len, uint32_t seq,
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
    uint32_t crc = crc32(packet, packet->len + sizeof(*packet), CRC_INIT);
    return htole32(crc);
}

static inline const struct NanohubCommand *hostIntfFindHandler(uint8_t *buf, size_t size, uint32_t *seq)
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

    if (mSeq == packet->seq) {
        mTxRetrans = true;
        return mRxCmd;
    } else {
        mTxRetrans = false;
    }

    *seq = packet->seq;

    if (mBusy)
        return NULL;

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
    mTxSize = size;
    mTxBufPtr = buf;
    mComm->txPacket(mTxBufPtr, mTxSize, callback);
}

static void hostIntfTxPacket(__le32 reason, uint8_t len, uint32_t seq,
        HostIntfCommCallbackF callback)
{
    struct NanohubPacket *txPacket = (struct NanohubPacket *)(mTxBuf.buf);
    txPacket->reason = reason;
    txPacket->seq = seq;
    txPacket->sync = NANOHUB_SYNC_BYTE;
    txPacket->len = len;

    struct NanohubPacketFooter *txFooter = hostIntfGetFooter(mTxBuf.buf);
    txFooter->crc = hostIntfComputeCrc(mTxBuf.buf);

    // send starting with the prePremable byte
    hostIntfTxBuf(1+NANOHUB_PACKET_SIZE(len), &mTxBuf.prePreamble, callback);
}

static inline void hostIntfTxPacketDone(int err, size_t tx,
        HostIntfCommCallbackF callback)
{
    if (!err && tx < mTxSize) {
        mTxSize -= tx;
        mTxBufPtr += tx;

        mComm->txPacket(mTxBufPtr, mTxSize, callback);
    }
}

static bool hostIntfRequest(uint32_t tid)
{
    mHostIntfTid = tid;
    atomicBitsetInit(mInterrupt, MAX_INTERRUPTS);
    atomicBitsetInit(mInterruptMask, MAX_INTERRUPTS);
    hostIntfSetInterruptMask(NANOHUB_INT_NONWAKEUP);
    mTxBuf.prePreamble = NANOHUB_PREAMBLE_BYTE;
    mTxBuf.postPreamble = NANOHUB_PREAMBLE_BYTE;

    mComm = platHostIntfInit();
    if (mComm) {
        int err = mComm->request();
        if (!err) {
            hostIntfRxPacket();
            osEventSubscribe(mHostIntfTid, EVT_APP_START);
            return true;
        }
    }

    return false;
}

static inline void hostIntfRxPacket()
{
    mComm->rxPacket(mRxBuf, sizeof(mRxBuf), hostIntfRxDone);
}

static void hostIntfRxDone(size_t rx, int err)
{
    mRxTimestamp = rtcGetTime();
    mRxSize = rx;

    if (err != 0) {
        osLog(LOG_ERROR, "%s: failed to receive request: %d\n", __func__, err);
        hostIntfRxPacket();
        return;
    }

    hostIntfGenerateAck(NULL);
}

static void hostIntfGenerateAck(void *cookie)
{
    uint32_t seq = 0;

    mRxCmd = hostIntfFindHandler(mRxBuf, mRxSize, &seq);

    if (mRxCmd) {
        if (mTxRetrans) {
            hostIntfTxBuf(mTxSize, &mTxBuf.prePreamble, hostIntfTxPayloadDone);
        } else {
            mSeq = seq;
            hostIntfTxPacket(NANOHUB_REASON_ACK, 0, seq, hostIntfTxAckDone);
        }
    } else {
        if (mBusy)
            hostIntfTxPacket(NANOHUB_REASON_NAK_BUSY, 0, seq, hostIntfTxAckDone);
        else
            hostIntfTxPacket(NANOHUB_REASON_NAK, 0, seq, hostIntfTxAckDone);
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
    if (!mRxCmd) {
        if (!mBusy)
            osLog(LOG_DEBUG, "%s: NACKed invalid request\n", __func__);
        hostIntfRxPacket();
        return;
    }

    osDefer(hostIntfGenerateResponse, NULL, true);
}

static void hostIntfGenerateResponse(void *cookie)
{
    void *rxPayload = hostIntfGetPayload(mRxBuf);
    uint8_t rx_len = hostIntfGetPayloadLen(mRxBuf);
    void *txPayload = hostIntfGetPayload(mTxBuf.buf);
    uint8_t respLen = mRxCmd->handler(rxPayload, rx_len, txPayload, mRxTimestamp);

    hostIntfTxPacket(mRxCmd->reason, respLen, mSeq, hostIntfTxPayloadDone);
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
    mComm->release();
}

static void resetBuffer(struct ActiveSensor *sensor)
{
    sensor->discard = true;
    sensor->buffer.length = 0;
    memset(&sensor->buffer.firstSample, 0x00, sizeof(struct SensorFirstSample));
}

void hostIntfSetBusy(bool busy)
{
    mBusy = busy;
}

bool hostIntfPacketDequeue(void *data)
{
    struct DataBuffer *buffer;
    bool ret = false;
    struct ActiveSensor *sensor;
    int i;

    if (!(ret = simpleQueueDequeue(mOutputQ, data))) {
        // nothing in queue. look for partial buffers to flush
        for (i = 0; i < mNumSensors; i++, mLastSensor = (mLastSensor + 1) % mNumSensors) {
            if (mActiveSensorTable[mLastSensor].buffer.length > 0) {
                memcpy(data, &mActiveSensorTable[mLastSensor].buffer, sizeof(struct DataBuffer));
                resetBuffer(mActiveSensorTable + mLastSensor);
                ret = true;
                mLastSensor = (mLastSensor + 1) % mNumSensors;
                break;
            }
        }
    }

    if (ret) {
        buffer = data;
        if (buffer->sensType > SENS_TYPE_INVALID && buffer->sensType <= SENS_TYPE_LAST_USER && mSensorList[buffer->sensType - 1] < MAX_REGISTERED_SENSORS) {
            sensor = mActiveSensorTable + mSensorList[buffer->sensType - 1];
            sensor->curSamples -= buffer->firstSample.numSamples;
        }
    }

    return ret;
}

static void initCompleteCallback(uint32_t timerId, void *data)
{
    osEnqueuePrivateEvt(EVT_APP_START, NULL, NULL, mHostIntfTid);
}

static bool queueDiscard(void *data, bool onDelete)
{
    struct DataBuffer *buffer = data;
    struct ActiveSensor *sensor;

    if (buffer->sensType > SENS_TYPE_INVALID && buffer->sensType <= SENS_TYPE_LAST_USER && mSensorList[buffer->sensType - 1] < MAX_REGISTERED_SENSORS) { // data
        sensor = mActiveSensorTable + mSensorList[buffer->sensType - 1];

        if (sensor->curSamples - buffer->firstSample.numSamples >= sensor->minSamples || onDelete) {
            sensor->curSamples -= buffer->firstSample.numSamples;

            return true;
        } else {
            return false;
        }
    } else {
        return true;
    }
}

static bool initSensors()
{
    int i, j, blocks, maxBlocks, numBlocks = 0, numAxis, packetSamples;
    bool present, error;
    const struct SensorInfo *si;
    uint32_t handle;

    mNumSensors = 0;

    for (i = SENS_TYPE_INVALID + 1; i <= SENS_TYPE_LAST_USER; i++) {
        for (j = 0, present = 0, error = 0; (si = sensorFind(i, j, &handle)) != NULL; j++) {
            if (!sensorGetInitComplete(handle)) {
                osLog(LOG_INFO, "initSensors: %s not ready!\n", si->sensorName);
                timTimerSet(SENSOR_INIT_DELAY, 0, 50, initCompleteCallback, NULL, true);
                return false;
            } else {
                if (!present) {
                    present = 1;
                    numAxis = si->numAxis;
                    switch (si->numAxis) {
                    case NUM_AXIS_EMBEDDED:
                    case NUM_AXIS_ONE:
                        packetSamples = NANOHUB_SENSOR_DATA_MAX / sizeof(struct SingleAxisDataPoint);
                        break;
                    case NUM_AXIS_THREE:
                        packetSamples = NANOHUB_SENSOR_DATA_MAX / sizeof(struct TripleAxisDataPoint);
                        break;
                    case NUM_AXIS_WIFI:
                        packetSamples = NANOHUB_SENSOR_DATA_MAX / sizeof(struct WifiScanResult);
                        break;
                    default:
                        packetSamples = 1;
                        error = true;
                    }
                    if (si->minSamples > MAX_MIN_SAMPLES)
                        maxBlocks = (MAX_MIN_SAMPLES + packetSamples - 1) / packetSamples;
                    else
                        maxBlocks = (si->minSamples + packetSamples - 1) / packetSamples;
                } else {
                    if (si->numAxis != numAxis) {
                        error = true;
                    } else {
                        if (si->minSamples > MAX_MIN_SAMPLES)
                            blocks = (MAX_MIN_SAMPLES + packetSamples - 1) / packetSamples;
                        else
                            blocks = (si->minSamples + packetSamples - 1) / packetSamples;

                        maxBlocks = maxBlocks > blocks ? maxBlocks : blocks;
                    }
                }
            }
        }

        if (present && !error) {
            mNumSensors ++;
            numBlocks += maxBlocks;
        }
    }

    if (numBlocks > MAX_NUM_BLOCKS) {
        osLog(LOG_INFO, "initSensors: numBlocks of %d exceeds maximum of %d\n", numBlocks, MAX_NUM_BLOCKS);
        numBlocks = MAX_NUM_BLOCKS;
    }

    mOutputQ = simpleQueueAlloc(numBlocks, sizeof(struct DataBuffer), queueDiscard);
    mActiveSensorTable = heapAlloc(mNumSensors * sizeof(struct ActiveSensor));
    memset(mActiveSensorTable, 0x00, mNumSensors * sizeof(struct ActiveSensor));

    for (i = SENS_TYPE_INVALID + 1, j = 0; i <= SENS_TYPE_LAST_USER && j < mNumSensors; i++) {
        if ((si = sensorFind(i, 0, &handle)) != NULL) {
            mSensorList[i - 1] = j;
            mActiveSensorTable[j].rate = 0ul;
            mActiveSensorTable[j].latency = 0ull;
            mActiveSensorTable[j].numAxis = si->numAxis;
            mActiveSensorTable[j].interrupt = si->interrupt;
            if (si->biasType != SENS_TYPE_INVALID) {
                mSensorList[si->biasType - 1] = j;
                osEventSubscribe(mHostIntfTid, sensorGetMyEventType(si->biasType));
            }
            if (si->minSamples > MAX_MIN_SAMPLES) {
                mActiveSensorTable[j].minSamples = MAX_MIN_SAMPLES;
                osLog(LOG_INFO, "initSensors: %s: minSamples of %d exceeded max of %d\n", si->sensorName, si->minSamples, MAX_MIN_SAMPLES);
            } else {
                mActiveSensorTable[j].minSamples = si->minSamples;
            }
            mActiveSensorTable[j].curSamples = 0;
            resetBuffer(mActiveSensorTable + j);
            mActiveSensorTable[j].buffer.sensType = i;
            mActiveSensorTable[j].oneshot = false;
            mActiveSensorTable[j].lastInterrupt = 0ull;
            switch (si->numAxis) {
            case NUM_AXIS_EMBEDDED:
            case NUM_AXIS_ONE:
                mActiveSensorTable[j].packetSamples = NANOHUB_SENSOR_DATA_MAX / sizeof(struct SingleAxisDataPoint);
                break;
            case NUM_AXIS_THREE:
                mActiveSensorTable[j].packetSamples = NANOHUB_SENSOR_DATA_MAX / sizeof(struct TripleAxisDataPoint);
                break;
            case NUM_AXIS_WIFI:
                mActiveSensorTable[j].packetSamples = NANOHUB_SENSOR_DATA_MAX / sizeof(struct WifiScanResult);
                break;
            }
            j++;
        } else if (mSensorList[i-1] == 0) {
            mSensorList[i-1] = MAX_REGISTERED_SENSORS;
        }
    }

    return true;
}

static void copySingleSamples(struct ActiveSensor *sensor, const struct SingleAxisDataEvent *single)
{
    int i;
    uint32_t deltaTime;
    uint8_t numSamples;

    for (i=0; i<single->samples[0].firstSample.numSamples; i++) {
        if (sensor->buffer.firstSample.numSamples == sensor->packetSamples) {
            simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
            resetBuffer(sensor);
        }

        if (sensor->buffer.firstSample.numSamples == 0) {
            if (i == 0) {
                sensor->lastTime = sensor->buffer.referenceTime = single->referenceTime;
            } else {
                sensor->lastTime += single->samples[i].deltaTime;
                sensor->buffer.referenceTime = sensor->lastTime;
            }
            sensor->buffer.length = sizeof(struct SingleAxisDataEvent) + sizeof(struct SingleAxisDataPoint);
            sensor->buffer.single[0].idata = single->samples[i].idata;
            sensor->buffer.firstSample.numSamples = 1;
            sensor->curSamples ++;
        } else {
            if (i == 0) {
                if (sensor->lastTime > single->referenceTime) {
                    // shouldn't happen. flush current packet
                    simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                    resetBuffer(sensor);
                    i --;
                } else if (single->referenceTime - sensor->lastTime > UINT32_MAX) {
                    simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                    resetBuffer(sensor);
                    i --;
                } else {
                    deltaTime = single->referenceTime - sensor->lastTime;
                    numSamples = sensor->buffer.firstSample.numSamples;

                    sensor->buffer.length += sizeof(struct SingleAxisDataPoint);
                    sensor->buffer.single[numSamples].deltaTime = deltaTime;
                    sensor->buffer.single[numSamples].idata = single->samples[0].idata;
                    sensor->lastTime = single->referenceTime;
                    sensor->buffer.firstSample.numSamples ++;
                    sensor->curSamples ++;
                }
            } else {
                deltaTime = single->samples[i].deltaTime;
                numSamples = sensor->buffer.firstSample.numSamples;

                sensor->buffer.length += sizeof(struct SingleAxisDataPoint);
                sensor->buffer.single[numSamples].deltaTime = deltaTime;
                sensor->buffer.single[numSamples].idata = single->samples[i].idata;
                sensor->lastTime += deltaTime;
                sensor->buffer.firstSample.numSamples ++;
                sensor->curSamples ++;
            }
        }
    }
}

static void copyTripleSamples(struct ActiveSensor *sensor, const struct TripleAxisDataEvent *triple)
{
    int i;
    uint32_t deltaTime;
    uint8_t numSamples;

    for (i=0; i<triple->samples[0].firstSample.numSamples; i++) {
        if (sensor->buffer.firstSample.numSamples == sensor->packetSamples) {
            simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
            resetBuffer(sensor);
        }

        if (sensor->buffer.firstSample.numSamples == 0) {
            if (i == 0) {
                sensor->lastTime = sensor->buffer.referenceTime = triple->referenceTime;
            } else {
                sensor->lastTime += triple->samples[i].deltaTime;
                sensor->buffer.referenceTime = sensor->lastTime;
            }
            sensor->buffer.length = sizeof(struct TripleAxisDataEvent) + sizeof(struct TripleAxisDataPoint);
            sensor->buffer.triple[0].ix = triple->samples[i].ix;
            sensor->buffer.triple[0].iy = triple->samples[i].iy;
            sensor->buffer.triple[0].iz = triple->samples[i].iz;
            if (triple->samples[0].firstSample.biasPresent && triple->samples[0].firstSample.biasSample == i) {
                sensor->buffer.firstSample.biasCurrent = triple->samples[0].firstSample.biasCurrent;
                sensor->buffer.firstSample.biasPresent = 1;
                sensor->buffer.firstSample.biasSample = 0;
                sensor->discard = false;
            }
            sensor->buffer.firstSample.numSamples = 1;
            sensor->curSamples ++;
        } else {
            if (i == 0) {
                if (sensor->lastTime > triple->referenceTime) {
                    // shouldn't happen. flush current packet
                    simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                    resetBuffer(sensor);
                    i --;
                } else {
                    deltaTime = triple->referenceTime - sensor->lastTime;
                    numSamples = sensor->buffer.firstSample.numSamples;

                    sensor->buffer.length += sizeof(struct TripleAxisDataPoint);
                    sensor->buffer.triple[numSamples].deltaTime = deltaTime;
                    sensor->buffer.triple[numSamples].ix = triple->samples[0].ix;
                    sensor->buffer.triple[numSamples].iy = triple->samples[0].iy;
                    sensor->buffer.triple[numSamples].iz = triple->samples[0].iz;
                    sensor->lastTime = triple->referenceTime;
                    if (triple->samples[0].firstSample.biasPresent && triple->samples[0].firstSample.biasSample == 0) {
                        sensor->buffer.firstSample.biasCurrent = triple->samples[0].firstSample.biasCurrent;
                        sensor->buffer.firstSample.biasPresent = 1;
                        sensor->buffer.firstSample.biasSample = numSamples;
                        sensor->discard = false;
                    }
                    sensor->buffer.firstSample.numSamples ++;
                    sensor->curSamples ++;
                }
            } else {
                deltaTime = triple->samples[i].deltaTime;
                numSamples = sensor->buffer.firstSample.numSamples;

                sensor->buffer.length += sizeof(struct TripleAxisDataPoint);
                sensor->buffer.triple[numSamples].deltaTime = deltaTime;
                sensor->buffer.triple[numSamples].ix = triple->samples[i].ix;
                sensor->buffer.triple[numSamples].iy = triple->samples[i].iy;
                sensor->buffer.triple[numSamples].iz = triple->samples[i].iz;
                sensor->lastTime += deltaTime;
                if (triple->samples[0].firstSample.biasPresent && triple->samples[0].firstSample.biasSample == i) {
                    sensor->buffer.firstSample.biasCurrent = triple->samples[0].firstSample.biasCurrent;
                    sensor->buffer.firstSample.biasPresent = 1;
                    sensor->buffer.firstSample.biasSample = numSamples;
                    sensor->discard = false;
                }
                sensor->buffer.firstSample.numSamples ++;
                sensor->curSamples ++;
            }
        }
    }
}

static void copyWifiSamples(struct ActiveSensor *sensor, const struct WifiScanEvent *wifiScanEvent)
{
    int i;
    uint32_t deltaTime;
    uint8_t numSamples;

    for (i = 0; i < wifiScanEvent->results[0].firstSample.numSamples; i++) {
        if (sensor->buffer.firstSample.numSamples == sensor->packetSamples) {
            simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
            resetBuffer(sensor);
        }

        if (sensor->buffer.firstSample.numSamples == 0) {
            if (i == 0) {
                sensor->lastTime = sensor->buffer.referenceTime = wifiScanEvent->referenceTime;
            } else {
                sensor->lastTime += wifiScanEvent->results[i].deltaTime;
                sensor->buffer.referenceTime = sensor->lastTime;
            }
            sensor->buffer.length = sizeof(struct WifiScanEvent) + sizeof(struct WifiScanResult);
            memcpy(&sensor->buffer.wifiScanResults[0], &wifiScanEvent->results[i], sizeof(struct WifiScanResult));
            sensor->buffer.firstSample.numSamples = 1;
            sensor->curSamples ++;
        } else {
            if (i == 0) {
                if (sensor->lastTime > wifiScanEvent->referenceTime) {
                    // shouldn't happen. flush current packet
                    simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                    resetBuffer(sensor);
                    i --;
                } else {
                    deltaTime = wifiScanEvent->referenceTime - sensor->lastTime;
                    numSamples = sensor->buffer.firstSample.numSamples;

                    sensor->buffer.length += sizeof(struct WifiScanResult);
                    memcpy(&sensor->buffer.wifiScanResults[numSamples], &wifiScanEvent->results[0], sizeof(struct WifiScanResult));
                    sensor->lastTime = wifiScanEvent->referenceTime;
                    sensor->buffer.firstSample.numSamples ++;
                    sensor->curSamples ++;
                }
            } else {
                deltaTime = wifiScanEvent->results[i].deltaTime;
                numSamples = sensor->buffer.firstSample.numSamples;

                sensor->buffer.length += sizeof(struct WifiScanResult);
                memcpy(&sensor->buffer.wifiScanResults[numSamples], &wifiScanEvent->results[i], sizeof(struct WifiScanResult));
                sensor->lastTime += deltaTime;
                sensor->buffer.firstSample.numSamples ++;
                sensor->curSamples ++;
            }
        }
    }
}

static void hostIntfHandleEvent(uint32_t evtType, const void* evtData)
{
    struct ConfigCmd *cmd;
    int i;
    uint64_t currentTime, rtcTime;
    struct ActiveSensor *sensor;
    const struct SensorInfo *si;
    uint32_t tempSensorHandle;
#ifdef DEBUG_LOG_EVT
    struct DataBuffer *data;
#endif

    if (evtType == EVT_APP_START) {
        if (initSensors()) {
            osEventUnsubscribe(mHostIntfTid, EVT_APP_START);
            osEventSubscribe(mHostIntfTid, EVT_NO_SENSOR_CONFIG_EVENT);
#ifdef DEBUG_LOG_EVT
            osEventSubscribe(mHostIntfTid, DEBUG_LOG_EVT);
#endif
        }
    }
#ifdef DEBUG_LOG_EVT
    else if (evtType == DEBUG_LOG_EVT) {
        data = (struct DataBuffer *)evtData;
        data->sensType = SENS_TYPE_INVALID;
        simpleQueueEnqueue(mOutputQ, evtData, true);
    } else
#endif
    if (evtType == EVT_NO_SENSOR_CONFIG_EVENT) { // config
        cmd = (struct ConfigCmd *)evtData;
        if (cmd->sensType > SENS_TYPE_INVALID && cmd->sensType <= SENS_TYPE_LAST_USER && mSensorList[cmd->sensType - 1] < MAX_REGISTERED_SENSORS) {
            sensor = mActiveSensorTable + mSensorList[cmd->sensType - 1];

            if (sensor->sensorHandle) {
                if (cmd->cmd == CONFIG_CMD_FLUSH) {
                    sensorFlush(sensor->sensorHandle);
                } else if (cmd->cmd == CONFIG_CMD_ENABLE) {
                    if (sensorRequestRateChange(mHostIntfTid, sensor->sensorHandle, cmd->rate, cmd->latency)) {
                        sensor->rate = cmd->rate;
                        if (sensor->latency != cmd->latency) {
                            sensor->latency = cmd->latency;
                            sensor->lastInterrupt = timGetTime();
                        }
                    }
                } else if (cmd->cmd == CONFIG_CMD_DISABLE) {
                    sensorRelease(mHostIntfTid, sensor->sensorHandle);
                    osEventUnsubscribe(mHostIntfTid, sensorGetMyEventType(cmd->sensType));
                    sensor->sensorHandle = 0;
                    if (sensor->buffer.length) {
                        simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                        hostIntfSetInterrupt(sensor->interrupt);
                        resetBuffer(sensor);
                    }
                }
            } else if (cmd->cmd == CONFIG_CMD_ENABLE) {
                for (i=0; (si = sensorFind(cmd->sensType, i, &sensor->sensorHandle)) != NULL; i++) {
                    if (cmd->rate == SENSOR_RATE_ONESHOT) {
                        cmd->rate = SENSOR_RATE_ONCHANGE;
                        sensor->oneshot = true;
                    } else {
                        sensor->oneshot = false;
                    }

                    if (sensorRequest(mHostIntfTid, sensor->sensorHandle, cmd->rate, cmd->latency)) {
                        sensor->rate = cmd->rate;
                        sensor->latency = cmd->latency;
                        sensor->lastInterrupt = timGetTime();
                        osEventSubscribe(mHostIntfTid, sensorGetMyEventType(cmd->sensType));
                        break;
                    } else {
                        sensor->sensorHandle = 0;
                    }
                }
            } else if (cmd->cmd == CONFIG_CMD_CALIBRATE) {
                for (i=0; sensorFind(cmd->sensType, i, &tempSensorHandle) != NULL; i++)
                    sensorCalibrate(tempSensorHandle);
            } else if (cmd->cmd == CONFIG_CMD_CFG_DATA) {
                for (i=0; sensorFind(cmd->sensType, i, &tempSensorHandle) != NULL; i++)
                    sensorCfgData(tempSensorHandle, (void *)(cmd+1));
            }
        }
    } else if (evtType > EVT_NO_FIRST_SENSOR_EVENT && evtType < EVT_NO_SENSOR_CONFIG_EVENT && mSensorList[(evtType & 0xFF)-1] < MAX_REGISTERED_SENSORS) { // data
        sensor = mActiveSensorTable + mSensorList[(evtType & 0xFF) - 1];

        if (sensor->sensorHandle) {
            if (evtData == SENSOR_DATA_EVENT_FLUSH) {
                if (sensor->buffer.length == 0) {
                    sensor->buffer.length = sizeof(sensor->buffer.referenceTime) + sizeof(struct SensorFirstSample);
                    sensor->buffer.referenceTime = 0ull;
                    sensor->buffer.firstSample.numFlushes = 1;
                } else {
                    sensor->buffer.firstSample.numFlushes ++;
                }
                sensor->discard = false;
                hostIntfSetInterrupt(sensor->interrupt);
            } else {
                if (sensor->buffer.length > 0) {
                    if (sensor->buffer.firstSample.numFlushes > 0) {
                        if (!(simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard)))
                            return; // flushes more important than samples
                        else
                            resetBuffer(sensor);
                    } else if (sensor->buffer.firstSample.numSamples == sensor->packetSamples) {
                        simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                        resetBuffer(sensor);
                    }
                }

                switch (sensor->numAxis) {
                case NUM_AXIS_EMBEDDED:
                    rtcTime = rtcGetTime();
                    if (sensor->buffer.length > 0 && rtcTime - sensor->lastTime > UINT32_MAX) {
                        simpleQueueEnqueue(mOutputQ, &sensor->buffer, sensor->discard);
                        resetBuffer(sensor);
                    }
                    if (sensor->buffer.length == 0) {
                        sensor->buffer.length = sizeof(struct SingleAxisDataEvent) + sizeof(struct SingleAxisDataPoint);
                        sensor->lastTime = sensor->buffer.referenceTime = rtcTime;
                        sensor->buffer.firstSample.numSamples = 1;
                        sensor->buffer.single[0].idata = (uint32_t)evtData;
                    } else {
                        sensor->buffer.length += sizeof(struct SingleAxisDataPoint);
                        sensor->buffer.single[sensor->buffer.firstSample.numSamples].deltaTime = rtcTime - sensor->lastTime;
                        sensor->lastTime = rtcTime;
                        sensor->buffer.single[sensor->buffer.firstSample.numSamples].idata = (uint32_t)evtData;
                        sensor->buffer.firstSample.numSamples ++;
                    }
                    sensor->curSamples ++;
                    break;
                case NUM_AXIS_ONE:
                    copySingleSamples(sensor, evtData);
                    break;
                case NUM_AXIS_THREE:
                    copyTripleSamples(sensor, evtData);
                    break;
                case NUM_AXIS_WIFI:
                    copyWifiSamples(sensor, evtData);
                    break;
                default:
                    return;
                }
            }

            currentTime = timGetTime();
            if ((currentTime >= sensor->lastInterrupt + sensor->latency) ||
                ((sensor->latency > sensorGetCurLatency(sensor->sensorHandle)) &&
                    (currentTime + sensorGetCurLatency(sensor->sensorHandle) > sensor->lastInterrupt + sensor->latency))) {
                hostIntfSetInterrupt(sensor->interrupt);
                sensor->lastInterrupt += sensor->latency;
            }

            if (sensor->oneshot) {
                sensorRelease(mHostIntfTid, sensor->sensorHandle);
                osEventUnsubscribe(mHostIntfTid, evtType);
                sensor->sensorHandle = 0;
                sensor->oneshot = false;
            }
        } else if (evtData != SENSOR_DATA_EVENT_FLUSH) {
            // handle bias data which can be generated for sensors that are
            // not currently requested by the AP
            switch (sensor->numAxis) {
            case NUM_AXIS_THREE:
                if (((const struct TripleAxisDataEvent *)evtData)->samples[0].firstSample.biasPresent) {
                    copyTripleSamples(sensor, evtData);
                    hostIntfSetInterrupt(sensor->interrupt);
                }
                break;
            default:
                break;
            }
        }
    }
}

void hostIntfCopyClearInterrupts(struct AtomicBitset *dst, uint32_t numBits)
{
    atomicBitsetInit(dst, numBits);

    apIntClear(false);
    apIntClear(true);

    atomicBitsetXchg(mInterrupt, dst);
}

void hostIntfSetInterrupt(uint32_t bit)
{
    atomicBitsetSetBit(mInterrupt, bit);
    if (!atomicBitsetGetBit(mInterruptMask, bit)) {
        platRequestDevInSleepMode(Stm32sleepWakeup, 12);
        apIntSet(true);
    } else {
        apIntSet(false);
    }
}

void hostInfClearInterrupt(uint32_t bit)
{
    atomicBitsetClearBit(mInterrupt, bit);
}

void hostIntfSetInterruptMask(uint32_t bit)
{
    atomicBitsetSetBit(mInterruptMask, bit);
}

void hostInfClearInterruptMask(uint32_t bit)
{
    atomicBitsetClearBit(mInterruptMask, bit);
}

INTERNAL_APP_INIT(APP_ID_MAKE(APP_ID_VENDOR_GOOGLE, 0), 0, hostIntfRequest, hostIntfRelease, hostIntfHandleEvent);
