/*
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef HUB_CONNECTION_H_

#define HUB_CONNECTION_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>

#include <utils/Errors.h>
#include <utils/Mutex.h>
#include <utils/Thread.h>

#include "eventnums.h"
#include "hubdefs.h"
#include "ring.h"

namespace android {

struct HubConnection : public Thread {
    static HubConnection *getInstance();

    status_t initCheck() const;

    enum ProximitySensorType {
        PROXIMITY_UNKNOWN,
        PROXIMITY_ROHM,
        PROXIMITY_AMS,
    };

    // Blocks until it can return a status
    status_t getAliveCheck();

    // Blocks until it can return a type (including unknown).
    status_t getProximitySensorType(ProximitySensorType *type);

    virtual bool threadLoop();

    void queueActivate(int handle, bool enable);
    void queueSetDelay(int handle, nsecs_t delayNs);

    void queueBatch(
            int handle,
            int flags,
            int64_t sampling_period_ns,
            int64_t max_report_latency_ns);

    void queueFlush(int handle);

    void queueData(int handle, void *data, size_t length);

    void queueUsbMagBias();

    ssize_t read(sensors_event_t *ev, size_t size);

    typedef void (*ActivityFunc)(
            void *, uint64_t time_us, bool is_flush, float x, float y, float z);

    void setActivityCallback(void *cookie, ActivityFunc cb);

    void saveSensorSettings() const;

protected:
    HubConnection();
    virtual ~HubConnection();

    virtual void onFirstRef();

private:
    enum
    {
        CONFIG_CMD_DISABLE      = 0,
        CONFIG_CMD_ENABLE       = 1,
        CONFIG_CMD_FLUSH        = 2,
        CONFIG_CMD_CFG_DATA     = 3,
        CONFIG_CMD_CALIBRATE    = 4,
    };

    struct ConfigCmd
    {
        uint32_t evtType;
        uint64_t latency;
        uint32_t rate;
        uint8_t sensorType;
        uint8_t cmd;
        uint16_t flags;
        uint8_t data[];
    } __attribute__((packed));

    struct MsgCmd
    {
        uint32_t evtType;
        struct HostHubRawPacket msg;
    } __attribute__((packed));

    struct SensorState {
        uint64_t latency;
        uint32_t rate;
        uint8_t sensorType;
        uint8_t alt;
        uint8_t flushCnt;
        bool enable;
    };

    struct FirstSample
    {
        uint8_t numSamples;
        uint8_t numFlushes;
        uint8_t highAccuracy : 1;
        uint8_t biasPresent : 1;
        uint8_t biasSample : 6;
        uint8_t pad;
    };

    struct ThreeAxisSample
    {
        uint32_t deltaTime;
        float x, y, z;
    } __attribute__((packed));

    struct OneAxisSample
    {
        uint32_t deltaTime;
        union
        {
            float fdata;
            uint32_t idata;
        };
    } __attribute__((packed));

    struct nAxisEvent
    {
        uint32_t evtType;
        uint64_t referenceTime;
        union
        {
            struct FirstSample firstSample;
            struct OneAxisSample oneSamples[];
            struct ThreeAxisSample threeSamples[];
        };
    } __attribute__((packed));

    static Mutex sInstanceLock;
    static HubConnection *sInstance;

    Mutex mLock;

    RingBuffer mRing;

    void *mActivityCbCookie;
    ActivityFunc mActivityCb;

    float mMagBias[3];
    float mUsbMagBias;
    uint8_t mMagAccuracy;
    uint8_t mMagAccuracyRestore;

    float mGyroBias[3];

    SensorState mSensorState[NUM_COMMS_SENSORS_PLUS_1];

    uint64_t mStepCounterOffset;
    uint64_t mLastStepCount;

    int mUinputFd;

    int mFd;
    struct pollfd mPollFds[2];
    int mNumPollFds;

    sensors_event_t *initEv(sensors_event_t *ev, uint64_t timestamp, uint32_t type, uint32_t sensor);
    void magAccuracyUpdate(float x, float y, float z);
    void processSample(uint64_t timestamp, uint32_t type, uint32_t sensor, struct OneAxisSample *sample, bool highAccuracy);
    void processSample(uint64_t timestamp, uint32_t type, uint32_t sensor, struct ThreeAxisSample *sample, bool highAccuracy);

    status_t initializeUinputNode();
    void sendFolioEvent(int32_t data);
    void initConfigCmd(struct ConfigCmd *cmd, int handle);

    DISALLOW_EVIL_CONSTRUCTORS(HubConnection);
};

}  // namespace android

#endif  // HUB_CONNECTION_H_
