#include <stdlib.h>
#include <string.h>
#include <timer.h>
#include <heap.h>
#include <plat/inc/rtc.h>
#include <plat/inc/syscfg.h>
#include <hostIntf.h>
#include <nanohubPacket.h>

#include <seos.h>
#include <accelerometer.h>

#include <nanohub_math.h>
#include <fusion/fusion.h>
#include <sensors.h>
#include <limits.h>
#include <slab.h>

#define MAX_NUM_COMMS_EVENT_SAMPLES 15
#define MAX_NUM_SAMPLES         MAX_NUM_COMMS_EVENT_SAMPLES
#define EVT_SENSOR_ACC_DATA_RDY sensorGetMyEventType(SENS_TYPE_ACCEL)
#define EVT_SENSOR_GYR_DATA_RDY sensorGetMyEventType(SENS_TYPE_GYRO)
#define EVT_SENSOR_MAG_DATA_RDY sensorGetMyEventType(SENS_TYPE_MAG)
#define EVT_SENSOR_ORIENTATION_DATA_RDY sensorGetMyEventType(SENS_TYPE_ORIENTATION)

enum {
    FUSION_FLAG_ENABLED             = 1,
    FUSION_FLAG_INITIALIZED         = 8,
    FUSION_FLAG_GAME_ENABLED        = 16,
    FUSION_FLAG_GAME_INITIALIZED    = 32
};

struct OrientationSensorSample {
    uint64_t time;
    float x, y, z;
};

struct OrientationTask {
    uint32_t tid;
    uint32_t handle;

    struct Fusion fusion;

    struct OrientationSensorSample samples[3][MAX_NUM_SAMPLES];
    struct TripleAxisDataEvent *ev;
    size_t sample_indices[3];
    size_t sample_counts[3];

    uint64_t last_gyro_time;
    uint64_t last_acc_time;
    uint64_t latency;
    uint32_t rate;
    uint32_t flags;

    bool use_gyro_data;
    bool use_mag_data;
    bool first_evt;
    bool active;
};

struct ConfigStat {
   uint32_t rate;
   uint64_t latency;
   uint8_t clientId;
   uint8_t enable : 1;
   uint8_t flush : 1;
   uint8_t calibrate : 1;
} __attribute__((packed));

static uint32_t OrientationRates[] = {
    SENSOR_HZ(12.5f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    0,
};

static struct OrientationTask mTask;
static struct ConfigStat mImuConfig;
static const struct SensorInfo mSi =
{
    "Orientation",
    OrientationRates,
    SENS_TYPE_ORIENTATION,
    NUM_AXIS_THREE,
    { NANOHUB_INT_NONWAKEUP }
};

static uint64_t mCnt = 0;

static struct SlabAllocator *mDataSlab;

static void dataEvtFree(void *ptr)
{
    slabAllocatorFree(mDataSlab, ptr);
}

// index 0: accel; 1: gyro; 2: mag.
static void fillSamples(struct TripleAxisDataEvent *ev, int index)
{
    size_t i, sampleCnt;

    if (index == 1 && !mTask.use_gyro_data) {
        return;
    }
    if (index == 2 && !mTask.use_mag_data) {
        return;
    }

    sampleCnt = ev->samples[0].deltaTime;
    if (sampleCnt >= MAX_NUM_SAMPLES) {
        struct TripleAxisDataPoint *sample;
        // Copy the last MAX_NUM_SAMPLES samples over and reset the sample index.
        mTask.sample_indices[index] = 0;
        mTask.sample_counts[index] = MAX_NUM_SAMPLES;

        for (i = 0; i < MAX_NUM_SAMPLES; ++i) {
            sample = &ev->samples[sampleCnt - MAX_NUM_SAMPLES + i];
            mTask.samples[index][i].x = sample->x;
            mTask.samples[index][i].y = sample->y;
            mTask.samples[index][i].z = sample->z;
            if (i == 0)
                mTask.samples[index][i].time = ev->referenceTime;
            else
                mTask.samples[index][i].time = ev->referenceTime + sample->deltaTime;
        }

    } else {
        struct TripleAxisDataPoint *sample;
        size_t n = mTask.sample_counts[index];
        size_t start = (mTask.sample_indices[index] + n) % MAX_NUM_SAMPLES;

        size_t copy, copy2;
        copy = MAX_NUM_SAMPLES - start;
        if (copy > sampleCnt) {
            copy = sampleCnt;
        }

        for (i = 0; i < copy; ++i) {
            sample = &ev->samples[i];
            mTask.samples[index][start + i].x = sample->x;
            mTask.samples[index][start + i].y = sample->y;
            mTask.samples[index][start + i].z = sample->z;
            if (i == 0)
                mTask.samples[index][i].time = ev->referenceTime;
            else
                mTask.samples[index][start + i].time = ev->referenceTime + sample->deltaTime;
        }

        copy2 = sampleCnt - copy;

        for (i = 0; i < copy2; ++i) {
            sample = &ev->samples[copy + i];
            mTask.samples[index][i].x = sample->x;
            mTask.samples[index][i].y = sample->y;
            mTask.samples[index][i].z = sample->z;
            if (copy + i == 0)
                mTask.samples[index][i].time = ev->referenceTime;
            else
                mTask.samples[index][i].time = ev->referenceTime + sample->deltaTime;
        }

        mTask.sample_counts[index] += sampleCnt;

        // FIFO overflow: over-write the oldest samples and move the sample index
        if (mTask.sample_counts[index] > MAX_NUM_SAMPLES) {
            mTask.sample_indices[index] = (mTask.sample_indices[index]
                                         + mTask.sample_counts[index]
                                         - MAX_NUM_SAMPLES) % MAX_NUM_SAMPLES;
            mTask.sample_counts[index] = MAX_NUM_SAMPLES;
        }
    }
}

static void addSample(uint64_t time, float x, float y, float z)
{
    struct TripleAxisDataPoint *sample;
    uint32_t deltaTime;

    if (mTask.ev == NULL) {
        mTask.ev = slabAllocatorAlloc(mDataSlab);
        if (mTask.ev == NULL) {
            // slaballocation failed
            osLog(LOG_ERROR, "Slab Allocation Failed\n");
            return;
        }
        mTask.ev->samples[0].deltaTime = 0;
        mTask.ev->referenceTime = time;
    }

    if (mTask.ev->samples[0].deltaTime >= MAX_NUM_COMMS_EVENT_SAMPLES) {
        osLog(LOG_ERROR, "BAD_INDEX\n");
        return;
    }

    sample = &mTask.ev->samples[mTask.ev->samples[0].deltaTime++];

    deltaTime = (time > mTask.ev->referenceTime)
                ? (time - mTask.ev->referenceTime) : 0;

    // the first deltatime is for sample count
    if (mTask.ev->samples[0].deltaTime > 1)
        sample->deltaTime = deltaTime;

    sample->x = x;
    sample->y = y;
    sample->z = z;

    if (mTask.ev->samples[0].deltaTime == MAX_NUM_COMMS_EVENT_SAMPLES) {
        osEnqueueEvt(EVT_SENSOR_ORIENTATION_DATA_RDY, mTask.ev, dataEvtFree);
        mTask.ev = NULL;
    }
}

static void updateOutput(ssize_t last_accel_sample_index, uint64_t last_sensor_time)
{
    //osLog(LOG_INFO, "updating output.\n");

    if (fusionHasEstimate(&mTask.fusion)) {
        struct Mat33 R;
        fusionGetRotationMatrix(&mTask.fusion, &R);

        struct Vec4 attitude;
        fusionGetAttitude(&mTask.fusion, &attitude);

        // x, y, z = yaw, pitch, roll
        static const float kRad2deg = 180.0f / M_PI;
        float x = atan2f(-R.elem[0][1], R.elem[0][0]) * kRad2deg;
        float y = atan2f(-R.elem[1][2], R.elem[2][2]) * kRad2deg;
        float z = asinf(R.elem[0][2]) * kRad2deg;

        if (x < 0.0f) {
            x += 360.0f;
        }

        addSample(last_sensor_time, x, y, z);
        if (mCnt++ % 100 == 0) {
            osLog(LOG_INFO, "orientation: %d   %d   %d\n", (int)(x), (int)(y), (int)(z));
        }
    }
}

static void drainSamples()
{
    //int rv = 23769;
    size_t i = mTask.sample_indices[0];
    size_t j = 0;
    size_t k = 0;

    if (mTask.use_gyro_data) {
        j = mTask.sample_indices[1];
    }

    if (mTask.use_mag_data) {
        k = mTask.sample_indices[2];
    }

    while (mTask.sample_counts[0] > 0
            && (!mTask.use_gyro_data || mTask.sample_counts[1] > 0)
            && (!mTask.use_mag_data || mTask.sample_counts[2] > 0)) {

        size_t which = 0;

        uint64_t a_time = mTask.samples[0][i].time;
        uint64_t g_time = mTask.use_gyro_data ? mTask.samples[1][j].time
                            : ULONG_LONG_MAX;
        uint64_t m_time = mTask.use_mag_data ? mTask.samples[2][k].time
                            : ULONG_LONG_MAX;

        // priority with same timestamp: gyro > acc > mag
        if (g_time <= a_time && g_time <= m_time) {
            which = 1;
        } else if (a_time <= m_time) {
            which = 0;
        } else {
            which = 2;
        }
        switch (which) {
            case 0:
            {
                struct Vec3 a;
                initVec3(&a, mTask.samples[0][i].x, mTask.samples[0][i].y, mTask.samples[0][i].z);

                float dT = (mTask.samples[0][i].time - mTask.last_acc_time) * 1.0E-6f;
                mTask.last_acc_time = mTask.samples[0][i].time;

                if (mCnt ++ % 100 == 0)

                osLog(LOG_INFO, "   ORIENTATION_ACC:  %6d   %6d   %6d   %6d   %6d\n",
                        (int)(mTask.samples[0][i].x * 1000),
                        (int)(mTask.samples[0][i].y * 1000),
                        (int)(mTask.samples[0][i].z * 1000),
                        (int)(mTask.samples[0][i].time * 1.0E0f),
                        (int)(dT * 1.0E6f));

                fusionHandleAcc(&mTask.fusion, &a, dT);

                updateOutput(i, mTask.samples[0][i].time);

                --mTask.sample_counts[0];
                if (++i == MAX_NUM_SAMPLES) {
                    i = 0;
                }
                break;
            }

            case 1:
            {
                struct Vec3 w;
                initVec3(&w, mTask.samples[1][j].x, mTask.samples[1][j].y, mTask.samples[1][j].z);

                float dT = (mTask.samples[1][j].time - mTask.last_gyro_time) * 1.0E-6f;
                mTask.last_gyro_time = mTask.samples[1][j].time;

                if (mCnt ++ % 100 == 0)

                osLog(LOG_INFO, "   ORIENTATION_GYR:  %6d   %6d   %6d   %6d   %6d\n",
                        (int)(mTask.samples[1][i].x * 1000),
                        (int)(mTask.samples[1][i].y * 1000),
                        (int)(mTask.samples[1][i].z * 1000),
                        (int)(mTask.samples[1][i].time * 1.0E0f),
                        (int)(dT * 1.0E6f));

                fusionHandleGyro(&mTask.fusion, &w, dT);

                --mTask.sample_counts[1];
                if (++j == MAX_NUM_SAMPLES) {
                    j = 0;
                }
                break;
            }

            case 2:
            {
                struct Vec3 m;
                initVec3(&m, mTask.samples[2][k].x, mTask.samples[2][k].y, mTask.samples[2][k].z);

                if (mCnt ++ % 100 == 0)

                osLog(LOG_INFO, "   ORIENTATION_MAG:  %6d   %6d   %6d   %6d\n",
                        (int)(mTask.samples[2][i].x * 1000),
                        (int)(mTask.samples[2][i].y * 1000),
                        (int)(mTask.samples[2][i].z * 1000),
                        (int)(mTask.samples[2][i].time * 1.0E0f));

                fusionHandleMag(&mTask.fusion, &m);

                --mTask.sample_counts[2];
                if (++k == MAX_NUM_SAMPLES) {
                    k = 0;
                }
                break;
            }
        }
    }

    mTask.sample_indices[0] = i;

    if (mTask.use_gyro_data) {
        mTask.sample_indices[1] = j;
    }

    if (mTask.use_mag_data) {
        mTask.sample_indices[2] = k;
    }

    if (mTask.ev != NULL) {
        //XXX: post to AP
        heapFree(mTask.ev);
        mTask.ev = NULL;
    }
}

static void configureFusion()
{
    mTask.flags |= FUSION_FLAG_ENABLED;
    initFusion(
            &mTask.fusion,
            (mTask.use_mag_data ? FUSION_USE_MAG : 0) |
            (mTask.use_gyro_data ? FUSION_USE_GYRO : 0) |
            ((mTask.flags & FUSION_FLAG_INITIALIZED) ? 0 : FUSION_REINITIALIZE));
    mTask.flags |= FUSION_FLAG_INITIALIZED;
}

static bool orientationPower(bool on)
{
    if (on) {
        // if this is power on, we enable the imu when set rate.
        mTask.active = true;

        osEventSubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
        osEventSubscribe(mTask.tid, EVT_SENSOR_GYR_DATA_RDY);
        osEventSubscribe(mTask.tid, EVT_SENSOR_MAG_DATA_RDY);
    } else {
        mTask.active = false;

        osEventUnsubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
        osEventUnsubscribe(mTask.tid, EVT_SENSOR_GYR_DATA_RDY);
        osEventUnsubscribe(mTask.tid, EVT_SENSOR_MAG_DATA_RDY);

        osEnqueueEvt(EVT_SENSOR_ACC_CONFIG, &mImuConfig, NULL);
        osEnqueueEvt(EVT_SENSOR_GYR_CONFIG, &mImuConfig, NULL);
        osEnqueueEvt(EVT_SENSOR_MAG_CONFIG, &mImuConfig, NULL);
    }

    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);

    return true;
}

static bool orientationSetRate(uint32_t rate, uint64_t latency)
{
    mTask.rate = rate;
    mTask.latency = latency;

    osEnqueueEvt(EVT_SENSOR_ACC_CONFIG, &mImuConfig, NULL);
    osEnqueueEvt(EVT_SENSOR_GYR_CONFIG, &mImuConfig, NULL);
    osEnqueueEvt(EVT_SENSOR_MAG_CONFIG, &mImuConfig, NULL);

    return true;
}

static bool orientationFirmwareUpload()
{
    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_FW_STATE_CHG,
            mTask.rate, mTask.latency);
    return true;
}

static void config(struct ConfigStat *orientation_config)
{
    if (orientation_config->enable == 0 && mTask.active) {
        // if we are on, cmd is to turn off, release sensor
        sensorRelease(orientation_config->clientId, mTask.handle);
    } else if (orientation_config->enable == 1 && !mTask.active) {
        // if we are off, cmd is to turn on, request sensor
        sensorRequest(orientation_config->clientId, mTask.handle,
                orientation_config->rate, orientation_config->latency);
    } else {
        // if we are on, cmd is to turn on, change the config if needed.
        sensorRequestRateChange(orientation_config->clientId, mTask.handle,
                orientation_config->rate, orientation_config->latency);
    }
}

static void orientationHandleEvent(uint32_t evtType, const void* evtData)
{
    struct ConfigStat *configCmd;
    struct TripleAxisDataEvent *ev;

    switch (evtType) {
        case EVT_SENSOR_ORIENTATION_CONFIG:
            configCmd = (struct ConfigStat *)evtData;
            memcpy(&mImuConfig, evtData, sizeof(mImuConfig));
            mImuConfig.clientId = mTask.tid;
            config(configCmd);
            break;
        case EVT_SENSOR_ACC_DATA_RDY:
            ev = (struct TripleAxisDataEvent *)evtData;
            fillSamples(ev, 0);
            drainSamples();
        case EVT_SENSOR_GYR_DATA_RDY:
            ev = (struct TripleAxisDataEvent *)evtData;
            fillSamples(ev, 1);
            drainSamples();
        case EVT_SENSOR_MAG_DATA_RDY:
            ev = (struct TripleAxisDataEvent *)evtData;
            fillSamples(ev, 2);
            drainSamples();
            break;
        default:
            osLog(LOG_ERROR, "ORIENTATION SENSOR: Invalid EvtType: %ld\n", evtType);
            break;
    }
}

static const struct SensorOps mSops =
{
    orientationPower,
    orientationFirmwareUpload,
    orientationSetRate,
    NULL,
    NULL
};

static bool orientationStart(uint32_t tid)
{
    osLog(LOG_INFO, "        ORIENTATION:  %ld\n", tid);
    size_t i, slabSize;

    mTask.tid = tid;
    mTask.use_gyro_data = true;
    mTask.use_mag_data = true;
    mTask.last_gyro_time = 0ull;
    mTask.last_acc_time = 0ull;
    mTask.flags = 0;

    mTask.first_evt = true;

    for (i = 0; i < 3; i++) {
         mTask.sample_counts[i] = 0;
         mTask.sample_indices[i] = 0;
    }

    configureFusion();

    mTask.handle = sensorRegister(&mSi, &mSops);

    mTask.rate = SENSOR_HZ(100.0f);
    mTask.active = false;
    mTask.latency = 0;

    slabSize = sizeof(struct TripleAxisDataEvent)
        + MAX_NUM_COMMS_EVENT_SAMPLES * sizeof(struct TripleAxisDataPoint);
    mDataSlab = slabAllocatorNew(slabSize, 4, 10); // 10 slots for now..

    osEventSubscribe(mTask.tid, EVT_SENSOR_ORIENTATION_CONFIG);
    return true;
}

static void orientationEnd()
{
    mTask.flags &= ~FUSION_FLAG_INITIALIZED;
    slabAllocatorDestroy(mDataSlab);
}

INTERNAL_APP_INIT(
        0x0000000000000005ULL,
        orientationStart,
        orientationEnd,
        orientationHandleEvent);

