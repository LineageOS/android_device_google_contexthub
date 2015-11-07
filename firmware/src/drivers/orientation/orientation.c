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

enum
{
    FUSION_FLAG_ENABLED             = 0x01,
    FUSION_FLAG_INITIALIZED         = 0x08,
    FUSION_FLAG_GAME_ENABLED        = 0x10,
    FUSION_FLAG_GAME_INITIALIZED    = 0x20
};

enum SampleType
{
    SAMPLE_TYPE_ACCEL,
    SAMPLE_TYPE_GYRO,
    SAMPLE_TYPE_MAG,
    SAMPLE_TYPE_COUNT
};


struct OrientationSensorSample {
    uint64_t time;
    float x, y, z;
};

struct OrientationTask {
    uint32_t tid;
    uint32_t handle;
    uint32_t accelHandle;
    uint32_t gyroHandle;
    uint32_t magHandle;

    struct Fusion fusion;

    struct OrientationSensorSample samples[SAMPLE_TYPE_COUNT][MAX_NUM_SAMPLES];
    struct TripleAxisDataEvent *ev;
    size_t sample_indices[SAMPLE_TYPE_COUNT];
    size_t sample_counts[SAMPLE_TYPE_COUNT];

    uint64_t last_gyro_time;
    uint64_t last_acc_time;
    uint64_t latency;
    uint32_t rate;
    uint32_t flags;

    bool use_gyro_data;
    bool use_mag_data;
    bool first_evt;
};

static uint32_t OrientationRates[] = {
    SENSOR_HZ(12.5f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    0,
};

static struct OrientationTask mTask;
static const struct SensorInfo mSi =
{
    "Orientation",
    OrientationRates,
    SENS_TYPE_ORIENTATION,
    NUM_AXIS_THREE,
    { NANOHUB_INT_NONWAKEUP }
};

static struct SlabAllocator *mDataSlab;

static void dataEvtFree(void *ptr)
{
    slabAllocatorFree(mDataSlab, ptr);
}

static void fillSamples(struct TripleAxisDataEvent *ev, enum SampleType index)
{
    size_t i, sampleCnt, n, start, copy, copy2;
    struct TripleAxisDataPoint *sample;

    if (index == SAMPLE_TYPE_GYRO && !mTask.use_gyro_data) {
        return;
    }
    if (index == SAMPLE_TYPE_MAG && !mTask.use_mag_data) {
        return;
    }

    sampleCnt = ev->samples[0].numSamples;
    if (sampleCnt >= MAX_NUM_SAMPLES) {
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
        n = mTask.sample_counts[index];
        start = (mTask.sample_indices[index] + n) % MAX_NUM_SAMPLES;

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
                mTask.samples[index][start + i].time = ev->referenceTime;
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
    }
}

static void drainSamples()
{
    size_t i = mTask.sample_indices[SAMPLE_TYPE_ACCEL];
    size_t j = 0;
    size_t k = 0;
    size_t which;
    struct Vec3 a, w, m;
    float dT;
    uint64_t a_time, g_time, m_time;

    if (mTask.use_gyro_data)
        j = mTask.sample_indices[SAMPLE_TYPE_GYRO];

    if (mTask.use_mag_data)
        k = mTask.sample_indices[SAMPLE_TYPE_MAG];

    while (mTask.sample_counts[SAMPLE_TYPE_ACCEL] > 0
            && (!mTask.use_gyro_data || mTask.sample_counts[SAMPLE_TYPE_GYRO] > 0)
            && (!mTask.use_mag_data || mTask.sample_counts[SAMPLE_TYPE_MAG] > 0)) {
        a_time = mTask.samples[SAMPLE_TYPE_ACCEL][i].time;
        g_time = mTask.use_gyro_data ? mTask.samples[SAMPLE_TYPE_GYRO][j].time
                            : ULONG_LONG_MAX;
        m_time = mTask.use_mag_data ? mTask.samples[SAMPLE_TYPE_MAG][k].time
                            : ULONG_LONG_MAX;

        // priority with same timestamp: gyro > acc > mag
        if (g_time <= a_time && g_time <= m_time) {
            which = SAMPLE_TYPE_GYRO;
        } else if (a_time <= m_time) {
            which = SAMPLE_TYPE_ACCEL;
        } else {
            which = SAMPLE_TYPE_MAG;
        }

        switch (which) {
        case SAMPLE_TYPE_ACCEL:
            initVec3(&a, mTask.samples[SAMPLE_TYPE_ACCEL][i].x, mTask.samples[SAMPLE_TYPE_ACCEL][i].y, mTask.samples[SAMPLE_TYPE_ACCEL][i].z);

            dT = (mTask.samples[SAMPLE_TYPE_ACCEL][i].time - mTask.last_acc_time) * 1.0E-6f;
            mTask.last_acc_time = mTask.samples[SAMPLE_TYPE_ACCEL][i].time;

            fusionHandleAcc(&mTask.fusion, &a, dT);

            updateOutput(i, mTask.samples[SAMPLE_TYPE_ACCEL][i].time);

            --mTask.sample_counts[SAMPLE_TYPE_ACCEL];
            if (++i == MAX_NUM_SAMPLES)
                i = 0;
            break;
        case SAMPLE_TYPE_GYRO:
            initVec3(&w, mTask.samples[SAMPLE_TYPE_GYRO][j].x, mTask.samples[SAMPLE_TYPE_GYRO][j].y, mTask.samples[SAMPLE_TYPE_GYRO][j].z);

            dT = (mTask.samples[SAMPLE_TYPE_GYRO][j].time - mTask.last_gyro_time) * 1.0E-6f;
            mTask.last_gyro_time = mTask.samples[SAMPLE_TYPE_GYRO][j].time;

            fusionHandleGyro(&mTask.fusion, &w, dT);

            --mTask.sample_counts[SAMPLE_TYPE_GYRO];
            if (++j == MAX_NUM_SAMPLES)
                j = 0;
            break;
        case SAMPLE_TYPE_MAG:
            initVec3(&m, mTask.samples[SAMPLE_TYPE_MAG][k].x, mTask.samples[SAMPLE_TYPE_MAG][k].y, mTask.samples[SAMPLE_TYPE_MAG][k].z);

            fusionHandleMag(&mTask.fusion, &m);

            --mTask.sample_counts[SAMPLE_TYPE_MAG];
            if (++k == MAX_NUM_SAMPLES)
                k = 0;
            break;
        }
    }

    mTask.sample_indices[SAMPLE_TYPE_ACCEL] = i;

    if (mTask.use_gyro_data)
        mTask.sample_indices[SAMPLE_TYPE_GYRO] = j;

    if (mTask.use_mag_data)
        mTask.sample_indices[SAMPLE_TYPE_MAG] = k;

    if (mTask.ev != NULL) {
        osEnqueueEvt(EVT_SENSOR_ORIENTATION_DATA_RDY, mTask.ev, dataEvtFree);
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
    if (on == false) {
        if (mTask.accelHandle != 0) {
            sensorRelease(mTask.tid, mTask.accelHandle);
            mTask.accelHandle = 0;
            osEventUnsubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
        }

        if (mTask.gyroHandle != 0) {
            sensorRelease(mTask.tid, mTask.gyroHandle);
            mTask.gyroHandle = 0;
            osEventUnsubscribe(mTask.tid, EVT_SENSOR_GYR_DATA_RDY);
        }

        if (mTask.magHandle != 0) {
            sensorRelease(mTask.tid, mTask.magHandle);
            mTask.magHandle = 0;
            osEventUnsubscribe(mTask.tid, EVT_SENSOR_MAG_DATA_RDY);
        }
    }

    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);

    return true;
}

static bool orientationSetRate(uint32_t rate, uint64_t latency)
{
    int i;
    mTask.rate = rate;
    mTask.latency = latency;

    if (mTask.accelHandle == 0) {
        for (i=0; sensorFind(SENS_TYPE_ACCEL, i, &mTask.accelHandle) != NULL; i++) {
            if (sensorRequest(mTask.tid, mTask.accelHandle, rate, latency)) {
                osEventSubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
                break;
            }
        }
    } else {
        sensorRequestRateChange(mTask.tid, mTask.accelHandle, rate, latency);
    }

    if (mTask.gyroHandle == 0) {
        for (i=0; sensorFind(SENS_TYPE_GYRO, i, &mTask.gyroHandle) != NULL; i++) {
            if (sensorRequest(mTask.tid, mTask.gyroHandle, rate, latency)) {
                osEventSubscribe(mTask.tid, EVT_SENSOR_GYR_DATA_RDY);
                break;
            }
        }
    } else {
        sensorRequestRateChange(mTask.tid, mTask.gyroHandle, rate, latency);
    }

    if (mTask.magHandle == 0) {
        for (i=0; sensorFind(SENS_TYPE_MAG, i, &mTask.magHandle) != NULL; i++) {
            if (sensorRequest(mTask.tid, mTask.magHandle, rate, latency)) {
                osEventSubscribe(mTask.tid, EVT_SENSOR_MAG_DATA_RDY);
                break;
            }
        }
    } else {
        sensorRequestRateChange(mTask.tid, mTask.magHandle, rate, latency);
    }

    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);

    return true;
}

static bool orientationFirmwareUpload()
{
    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_FW_STATE_CHG,
              1, 0);
    return true;
}

static bool orientationFlush()
{
    return osEnqueueEvt(EVT_SENSOR_ORIENTATION_DATA_RDY, SENSOR_DATA_EVENT_FLUSH, NULL);
}

static void orientationHandleEvent(uint32_t evtType, const void* evtData)
{
    struct TripleAxisDataEvent *ev;

    if (evtData == SENSOR_DATA_EVENT_FLUSH)
        return;

    switch (evtType) {
    case EVT_SENSOR_ACC_DATA_RDY:
        ev = (struct TripleAxisDataEvent *)evtData;
        fillSamples(ev, SAMPLE_TYPE_ACCEL);
        drainSamples();
        break;
    case EVT_SENSOR_GYR_DATA_RDY:
        ev = (struct TripleAxisDataEvent *)evtData;
        fillSamples(ev, SAMPLE_TYPE_GYRO);
        drainSamples();
        break;
    case EVT_SENSOR_MAG_DATA_RDY:
        ev = (struct TripleAxisDataEvent *)evtData;
        fillSamples(ev, SAMPLE_TYPE_MAG);
        drainSamples();
        break;
    }
}

static const struct SensorOps mSops =
{
    orientationPower,
    orientationFirmwareUpload,
    orientationSetRate,
    orientationFlush,
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

    for (i = 0; i < SAMPLE_TYPE_COUNT; i++) {
         mTask.sample_counts[i] = 0;
         mTask.sample_indices[i] = 0;
    }

    configureFusion();

    mTask.handle = sensorRegister(&mSi, &mSops);

    mTask.rate = 0;
    mTask.latency = 0;

    slabSize = sizeof(struct TripleAxisDataEvent)
        + MAX_NUM_COMMS_EVENT_SAMPLES * sizeof(struct TripleAxisDataPoint);
    mDataSlab = slabAllocatorNew(slabSize, 4, 10); // 10 slots for now..

    return true;
}

static void orientationEnd()
{
    mTask.flags &= ~FUSION_FLAG_INITIALIZED;
    slabAllocatorDestroy(mDataSlab);
}

INTERNAL_APP_INIT(
        0x0000000000000004ULL,
        orientationStart,
        orientationEnd,
        orientationHandleEvent);
