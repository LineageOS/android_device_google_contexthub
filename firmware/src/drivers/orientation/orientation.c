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
#define EVT_SENSOR_ACC_DATA_RDY         sensorGetMyEventType(SENS_TYPE_ACCEL)
#define EVT_SENSOR_GYR_DATA_RDY         sensorGetMyEventType(SENS_TYPE_GYRO)
#define EVT_SENSOR_MAG_DATA_RDY         sensorGetMyEventType(SENS_TYPE_MAG)

#define kGravityEarth 9.80665f
#define DEFAULT_GYRO_RATE_HZ    SENSOR_HZ(100.0f)
#define DEFAULT_MAG_RATE_HZ     SENSOR_HZ(50.0f)

enum
{
    FUSION_FLAG_ENABLED             = 0x01,
    FUSION_FLAG_INITIALIZED         = 0x08,
    FUSION_FLAG_GAME_ENABLED        = 0x10,
    FUSION_FLAG_GAME_INITIALIZED    = 0x20
};

enum RawSensorType
{
    ACC,
    GYR,
    MAG,
    NUM_OF_RAW_SENSOR
};

enum FusionSensorType
{
    ORIENT,
    GRAVITY,
    GEOMAG,
    LINEAR,
    GAME,
    ROTAT,
    NUM_OF_FUSION_SENSOR
};


struct FusionSensorSample {
    uint64_t time;
    float x, y, z;
};

struct FusionSensor {
    uint32_t handle;
    struct TripleAxisDataEvent *ev;
    uint64_t latency;
    uint32_t rate;
    bool active;
    bool use_gyr_data;
    bool use_mag_data;
    uint8_t idx;
};

struct FusionTask {
    uint32_t tid;
    uint32_t accelHandle;
    uint32_t gyroHandle;
    uint32_t magHandle;

    struct Fusion fusion;
    struct Fusion game;

    struct FusionSensor sensors[NUM_OF_FUSION_SENSOR];
    struct FusionSensorSample samples[NUM_OF_RAW_SENSOR][MAX_NUM_SAMPLES];
    size_t sample_indices[NUM_OF_RAW_SENSOR];
    size_t sample_counts[NUM_OF_RAW_SENSOR];
    uint32_t counters[NUM_OF_RAW_SENSOR];
    uint64_t ResamplePeriodNs[NUM_OF_RAW_SENSOR];
    uint64_t last_time[NUM_OF_RAW_SENSOR];
    struct TripleAxisDataPoint last_sample[NUM_OF_RAW_SENSOR];

    uint64_t last_gyro_time;
    uint64_t last_acc_time;
    uint32_t flags;

    uint8_t acc_cnt;
    uint8_t gyr_cnt;
    uint8_t mag_cnt;
};

static uint32_t FusionRates[] = {
    SENSOR_HZ(12.5f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    0,
};

static struct FusionTask mTask;
static const struct SensorInfo mSi[NUM_OF_FUSION_SENSOR] =
{
    {"Orientation",		            FusionRates, SENS_TYPE_ORIENTATION,     NUM_AXIS_THREE, { NANOHUB_INT_NONWAKEUP }},
    {"Gravity",                     FusionRates, SENS_TYPE_GRAVITY,         NUM_AXIS_THREE, { NANOHUB_INT_NONWAKEUP }},
    {"Geomagnetic Rotation Vector", FusionRates, SENS_TYPE_GEO_MAG_ROT_VEC, NUM_AXIS_THREE, { NANOHUB_INT_NONWAKEUP }},
    {"Linear Acceleration",         FusionRates, SENS_TYPE_LINEAR_ACCEL,    NUM_AXIS_THREE, { NANOHUB_INT_NONWAKEUP }},
    {"Game Rotation Vector",        FusionRates, SENS_TYPE_GAME_ROT_VECTOR, NUM_AXIS_THREE, { NANOHUB_INT_NONWAKEUP }},
    {"Rotation Vector",             FusionRates, SENS_TYPE_ROTATION_VECTOR, NUM_AXIS_THREE, { NANOHUB_INT_NONWAKEUP }},
};

static struct SlabAllocator *mDataSlab;

static void dataEvtFree(void *ptr)
{
    slabAllocatorFree(mDataSlab, ptr);
}

static void fillSamples(struct TripleAxisDataEvent *ev, enum RawSensorType index)
{
    bool bad_timestamp;
    size_t i, w, n, num_samples;
    struct TripleAxisDataPoint *curr_sample, *next_sample;
	uint32_t counter;
	uint64_t ResamplePeriodNs, curr_time, next_time;
    uint64_t sample_spacing_ns;
    float weight_next;

    if (index == GYR && mTask.gyr_cnt == 0) {
        return;
    }
    if (index == MAG && mTask.mag_cnt == 0) {
        return;
    }

    n = mTask.sample_counts[index];
    i = mTask.sample_indices[index];
    counter = mTask.counters[index];
    ResamplePeriodNs = mTask.ResamplePeriodNs[index];
    w = (mTask.sample_indices[index] + n) % MAX_NUM_SAMPLES;

    // check if this sensor was used before
    if (mTask.last_time[index] == ULONG_LONG_MAX) {
        curr_sample = ev->samples;
        next_sample = curr_sample + 1;
        num_samples = ev->samples[0].numSamples;
        curr_time = ev->referenceTime;
    } else {
        curr_sample = &mTask.last_sample[index];
        next_sample = ev->samples;
        num_samples = ev->samples[0].numSamples + 1;
        curr_time = mTask.last_time[index];
    }

    while (num_samples > 1) {

        if (next_sample == ev->samples)
            next_time = ev->referenceTime;
        else
            next_time = curr_time + next_sample->deltaTime;

        // error handling for non-chronological accel timestamps
        sample_spacing_ns = (next_time > curr_time) ?  (next_time - curr_time) : 0;

        // This happens during BMI160 config changes
        bad_timestamp = (sample_spacing_ns > 10 * ResamplePeriodNs);

        // Check to see if we need to move the interpolation window or
        // interpolate
        if ((counter >= sample_spacing_ns) || bad_timestamp) {
            num_samples--;
            counter -= (bad_timestamp ? counter : sample_spacing_ns);
            curr_sample = next_sample;
            next_sample++;

        } else {
            weight_next = (float)counter / sample_spacing_ns;

            mTask.samples[index][w].x = curr_sample->x + weight_next *
                (next_sample->x - curr_sample->x);
            mTask.samples[index][w].y = curr_sample->y + weight_next *
                (next_sample->y - curr_sample->y);
            mTask.samples[index][w].z = curr_sample->z + weight_next *
                (next_sample->z - curr_sample->z);
            mTask.samples[index][w].time = curr_time + counter;

            // Move the read index when buffer is full
            if (++n > MAX_NUM_SAMPLES) {
                n = MAX_NUM_SAMPLES;

                if (++i == MAX_NUM_SAMPLES) {
                    i = 0;
                }
            }

            // Reset the write index
            if (++w == MAX_NUM_SAMPLES) {
                w = 0;
            }

            // Move to the next resample
            counter += ResamplePeriodNs;
        }
        curr_time = next_time;
    }

    mTask.sample_counts[index] = n;
    mTask.sample_indices[index] = i;
    mTask.counters[index] = counter;
    mTask.last_sample[index] = *curr_sample;
    mTask.last_time[index] = curr_time;

}

static void addSample(struct FusionSensor *mSensor, uint64_t time, float x, float y, float z)
{
    struct TripleAxisDataPoint *sample;
    uint32_t deltaTime;

    if (mSensor->ev == NULL) {
        mSensor->ev = slabAllocatorAlloc(mDataSlab);
        if (mSensor->ev == NULL) {
            // slaballocation failed
            osLog(LOG_ERROR, "Slab Allocation Failed\n");
            return;
        }
        mSensor->ev->samples[0].deltaTime = 0;
        mSensor->ev->referenceTime = time;
    }

    if (mSensor->ev->samples[0].deltaTime >= MAX_NUM_COMMS_EVENT_SAMPLES) {
        osLog(LOG_ERROR, "BAD_INDEX\n");
        return;
    }

    sample = &mSensor->ev->samples[mSensor->ev->samples[0].deltaTime++];

    deltaTime = (time > mSensor->ev->referenceTime)
                ? (time - mSensor->ev->referenceTime) : 0;

    // the first deltatime is for sample count
    if (mSensor->ev->samples[0].deltaTime > 1)
        sample->deltaTime = deltaTime;

    sample->x = x;
    sample->y = y;
    sample->z = z;

    if (mSensor->ev->samples[0].deltaTime == MAX_NUM_COMMS_EVENT_SAMPLES) {
        osEnqueueEvt(sensorGetMyEventType(mSi[mSensor->idx].sensorType), mSensor->ev, dataEvtFree);
        mSensor->ev = NULL;
    }
}

static void updateOutput(ssize_t last_accel_sample_index, uint64_t last_sensor_time)
{
    struct Vec4 attitude;
    struct Vec3 g, a;
    struct Mat33 R;

    if (fusionHasEstimate(&mTask.game)) {
        if (mTask.sensors[GAME].active) {
            fusionGetAttitude(&mTask.game, &attitude);
            addSample(&mTask.sensors[GAME],
                    last_sensor_time,
                    attitude.x,
                    attitude.y,
                    attitude.z);
        }

        if (mTask.sensors[GRAVITY].active) {
            fusionGetRotationMatrix(&mTask.game, &R);
            initVec3(&g, R.elem[0][2], R.elem[1][2], R.elem[2][2]);
            vec3ScalarMul(&g, kGravityEarth);
            addSample(&mTask.sensors[GRAVITY],
                    last_sensor_time,
                    g.x, g.y, g.z);
        }
    }

    if (fusionHasEstimate(&mTask.fusion)) {
        fusionGetRotationMatrix(&mTask.fusion, &R);
        fusionGetAttitude(&mTask.fusion, &attitude);

        if (mTask.sensors[ORIENT].active) {
            // x, y, z = yaw, pitch, roll
            static const float kRad2deg = 180.0f / M_PI;
            float x = atan2f(-R.elem[0][1], R.elem[0][0]) * kRad2deg;
            float y = atan2f(-R.elem[1][2], R.elem[2][2]) * kRad2deg;
            float z = asinf(R.elem[0][2]) * kRad2deg;

            if (x < 0.0f) {
                x += 360.0f;
            }

            addSample(&mTask.sensors[ORIENT],
                    last_sensor_time, x, y, z);
        }

        if (mTask.sensors[GEOMAG].active) {
            addSample(&mTask.sensors[GEOMAG],
                    last_sensor_time,
                    attitude.x,
                    attitude.y,
                    attitude.z);
        }

        if (mTask.sensors[ROTAT].active) {
            addSample(&mTask.sensors[ROTAT],
                    last_sensor_time,
                    attitude.x,
                    attitude.y,
                    attitude.z);
        }

        if (last_accel_sample_index >= 0
                && mTask.sensors[LINEAR].active) {
            initVec3(&g, R.elem[0][2], R.elem[1][2], R.elem[2][2]);
            vec3ScalarMul(&g, kGravityEarth);
            initVec3(&a,
                    mTask.samples[0][last_accel_sample_index].x,
                    mTask.samples[0][last_accel_sample_index].y,
                    mTask.samples[0][last_accel_sample_index].z);

            addSample(&mTask.sensors[LINEAR],
                    mTask.samples[0][last_accel_sample_index].time,
                    a.x - g.x,
                    a.y - g.y,
                    a.z - g.z);
        }
    }
}

static void drainSamples()
{
    size_t i = mTask.sample_indices[ACC];
    size_t j = 0;
    size_t k = 0;
    size_t which;
    struct Vec3 a, w, m;
    float dT;
    uint64_t a_time, g_time, m_time;

    if (mTask.gyr_cnt > 0)
        j = mTask.sample_indices[GYR];

    if (mTask.mag_cnt > 0)
        k = mTask.sample_indices[MAG];

    while (mTask.sample_counts[ACC] > 0
            && (!mTask.gyr_cnt > 0 || mTask.sample_counts[GYR] > 0)
            && (!mTask.mag_cnt > 0 || mTask.sample_counts[MAG] > 0)) {
        a_time = mTask.samples[ACC][i].time;
        g_time = mTask.gyr_cnt > 0 ? mTask.samples[GYR][j].time
                            : ULONG_LONG_MAX;
        m_time = mTask.mag_cnt > 0 ? mTask.samples[MAG][k].time
                            : ULONG_LONG_MAX;

        // priority with same timestamp: gyro > acc > mag
        if (g_time <= a_time && g_time <= m_time) {
            which = GYR;
        } else if (a_time <= m_time) {
            which = ACC;
        } else {
            which = MAG;
        }

        switch (which) {
        case ACC:
            initVec3(&a, mTask.samples[ACC][i].x, mTask.samples[ACC][i].y, mTask.samples[ACC][i].z);

            dT = (mTask.samples[ACC][i].time - mTask.last_acc_time) * 1.0E-9f;
            mTask.last_acc_time = mTask.samples[ACC][i].time;

            if (mTask.flags & FUSION_FLAG_ENABLED)
                fusionHandleAcc(&mTask.fusion, &a, dT);

            if (mTask.flags & FUSION_FLAG_GAME_ENABLED)
                fusionHandleAcc(&mTask.game, &a, dT);

            updateOutput(i, mTask.samples[ACC][i].time);

            --mTask.sample_counts[ACC];
            if (++i == MAX_NUM_SAMPLES)
                i = 0;
            break;
        case GYR:
            initVec3(&w, mTask.samples[GYR][j].x, mTask.samples[GYR][j].y, mTask.samples[GYR][j].z);

            dT = (mTask.samples[GYR][j].time - mTask.last_gyro_time) * 1.0E-9f;
            mTask.last_gyro_time = mTask.samples[GYR][j].time;

            if (mTask.flags & FUSION_FLAG_ENABLED)
                fusionHandleGyro(&mTask.fusion, &w, dT);

            if (mTask.flags & FUSION_FLAG_GAME_ENABLED)
                fusionHandleGyro(&mTask.game, &w, dT);

            --mTask.sample_counts[GYR];
            if (++j == MAX_NUM_SAMPLES)
                j = 0;
            break;
        case MAG:
            initVec3(&m, mTask.samples[MAG][k].x, mTask.samples[MAG][k].y, mTask.samples[MAG][k].z);

            fusionHandleMag(&mTask.fusion, &m);

            --mTask.sample_counts[MAG];
            if (++k == MAX_NUM_SAMPLES)
                k = 0;
            break;
        }
    }

    mTask.sample_indices[ACC] = i;

    if (mTask.gyr_cnt > 0)
        mTask.sample_indices[GYR] = j;

    if (mTask.mag_cnt > 0)
        mTask.sample_indices[MAG] = k;

    for (i = ORIENT; i < NUM_OF_FUSION_SENSOR; i++) {
        if (mTask.sensors[i].ev != NULL) {
            osEnqueueEvt(sensorGetMyEventType(mSi[i].sensorType), mTask.sensors[i].ev, dataEvtFree);
            mTask.sensors[i].ev = NULL;
        }
    }
}

static void configureFusion()
{
    if (mTask.sensors[ORIENT].active
            || mTask.sensors[ROTAT].active
            || mTask.sensors[LINEAR].active
            || mTask.sensors[GEOMAG].active) {
        mTask.flags |= FUSION_FLAG_ENABLED;
        initFusion(&mTask.fusion,
                (mTask.mag_cnt > 0 ? FUSION_USE_MAG : 0) |
                (mTask.gyr_cnt > 0 ? FUSION_USE_GYRO : 0) |
                ((mTask.flags & FUSION_FLAG_INITIALIZED) ? 0 : FUSION_REINITIALIZE));
        mTask.flags |= FUSION_FLAG_INITIALIZED;
    } else {
        mTask.flags &= ~FUSION_FLAG_ENABLED;
        mTask.flags &= ~FUSION_FLAG_INITIALIZED;
    }
}

static void configureGame()
{
    if (mTask.sensors[GAME].active || mTask.sensors[GRAVITY].active) {
        mTask.flags |= FUSION_FLAG_GAME_ENABLED;
        initFusion(&mTask.game,
                FUSION_USE_GYRO | ((mTask.flags & FUSION_FLAG_INITIALIZED) ? 0 : FUSION_REINITIALIZE));
        mTask.flags |= FUSION_FLAG_GAME_INITIALIZED;
    } else {
        mTask.flags &= ~FUSION_FLAG_GAME_ENABLED;
        mTask.flags &= ~FUSION_FLAG_GAME_INITIALIZED;
    }
}

static void fusionSetRateAcc(struct FusionSensor *mSensor)
{
    int i;
    uint32_t max_rate = 0;
    uint64_t min_latency = ULONG_LONG_MAX;

	for (i = ORIENT; i < NUM_OF_FUSION_SENSOR; i++) {
		if (mTask.sensors[i].active) {
			max_rate = max_rate > mTask.sensors[i].rate ? max_rate : mTask.sensors[i].rate;
			min_latency = min_latency < mTask.sensors[i].latency ? min_latency : mTask.sensors[i].latency;
		}
	}
	mTask.ResamplePeriodNs[ACC] = (1000000000ull * 1024ull / max_rate);

    if  (mTask.accelHandle == 0) {
        mTask.sample_counts[ACC] = 0;
        mTask.sample_indices[ACC] = 0;
        mTask.counters[ACC] = 0;
        mTask.last_time[ACC] = ULONG_LONG_MAX;
        for (i=0; sensorFind(SENS_TYPE_ACCEL, i, &mTask.accelHandle) != NULL; i++) {
            if (sensorRequest(mTask.tid, mTask.accelHandle, max_rate, min_latency)) {
                osEventSubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
                break;
            }
        }
    } else {
        sensorRequestRateChange(mTask.tid, mTask.accelHandle, max_rate, min_latency);
    }
}

static void fusionSetRateGyr(struct FusionSensor *mSensor)
{
    int i;
    uint32_t max_rate = 0;
    uint64_t min_latency = ULONG_LONG_MAX;

    for (i = ORIENT; i < NUM_OF_FUSION_SENSOR; i++) {
        if (mTask.sensors[i].active && mTask.sensors[i].use_gyr_data) {
            max_rate = max_rate > mTask.sensors[i].rate ? max_rate : mTask.sensors[i].rate;
            min_latency = min_latency < mTask.sensors[i].latency ? min_latency : mTask.sensors[i].latency;
        }
    }
	max_rate = max_rate > DEFAULT_GYRO_RATE_HZ ? max_rate : DEFAULT_GYRO_RATE_HZ;
	mTask.ResamplePeriodNs[GYR] = (1000000000ull * 1024ull / max_rate);

    if (mTask.gyroHandle == 0) {
        mTask.sample_counts[GYR] = 0;
        mTask.sample_indices[GYR] = 0;
        mTask.counters[GYR] = 0;
        mTask.last_time[GYR] = ULONG_LONG_MAX;
        for (i=0; sensorFind(SENS_TYPE_GYRO, i, &mTask.gyroHandle) != NULL; i++) {
            if (sensorRequest(mTask.tid, mTask.gyroHandle, max_rate, min_latency)) {
                osEventSubscribe(mTask.tid, EVT_SENSOR_GYR_DATA_RDY);
                break;
            }
        }
    } else {
        sensorRequestRateChange(mTask.tid, mTask.gyroHandle, max_rate, min_latency);
    }
}

static void fusionSetRateMag(struct FusionSensor *mSensor)
{
    int i;
    uint32_t max_rate = 0;
    uint64_t min_latency = ULONG_LONG_MAX;

	for (i = ORIENT; i < NUM_OF_FUSION_SENSOR; i++) {
		if (mTask.sensors[i].active && mTask.sensors[i].use_mag_data) {
			max_rate = max_rate > mTask.sensors[i].rate ? max_rate : mTask.sensors[i].rate;
			min_latency = min_latency < mTask.sensors[i].latency ? min_latency : mTask.sensors[i].latency;
		}
	}
	max_rate = max_rate < DEFAULT_MAG_RATE_HZ ? max_rate : DEFAULT_MAG_RATE_HZ;
	mTask.ResamplePeriodNs[MAG] = (1000000000ull * 1024ull / max_rate);

    if (mTask.magHandle == 0) {
        mTask.sample_counts[MAG] = 0;
        mTask.sample_indices[MAG] = 0;
        mTask.counters[MAG] = 0;
        mTask.last_time[MAG] = ULONG_LONG_MAX;
        for (i=0; sensorFind(SENS_TYPE_MAG, i, &mTask.magHandle) != NULL; i++) {
            if (sensorRequest(mTask.tid, mTask.magHandle, max_rate, min_latency)) {
                osEventSubscribe(mTask.tid, EVT_SENSOR_MAG_DATA_RDY);
                break;
            }
        }
    } else {
        sensorRequestRateChange(mTask.tid, mTask.magHandle, max_rate, min_latency);
    }
}

static bool fusionPower(struct FusionSensor *mSensor, bool on)
{
    mSensor->active = on;
    if (on == false) {
        mTask.acc_cnt--;
        if (mSensor->idx != GEOMAG)
            mTask.gyr_cnt--;
        if (mSensor->idx != GAME && mSensor->idx != GRAVITY)
            mTask.mag_cnt--;

        // if cnt == 0 and Handle == 0, nothing need to be done.
        // if cnt > 0 and Handle == 0, something else is turning it on, all will be done.
        if (mTask.acc_cnt == 0 && mTask.accelHandle != 0) {
            sensorRelease(mTask.tid, mTask.accelHandle);
            mTask.accelHandle = 0;
            osEventUnsubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
        } else if (mTask.acc_cnt > 0 && mTask.accelHandle != 0) {
            fusionSetRateAcc(mSensor);
        }

        if (mTask.gyr_cnt == 0 && mTask.gyroHandle != 0) {
            sensorRelease(mTask.tid, mTask.gyroHandle);
            mTask.gyroHandle = 0;
            osEventUnsubscribe(mTask.tid, EVT_SENSOR_GYR_DATA_RDY);
        } else if (mTask.gyr_cnt > 0 && mTask.gyroHandle != 0) {
            fusionSetRateGyr(mSensor);
        }

        if (mTask.mag_cnt == 0 && mTask.magHandle != 0) {
            sensorRelease(mTask.tid, mTask.magHandle);
            mTask.magHandle = 0;
            osEventUnsubscribe(mTask.tid, EVT_SENSOR_MAG_DATA_RDY);
        } else if (mTask.mag_cnt > 0 && mTask.magHandle != 0) {
            fusionSetRateMag(mSensor);
        }
    } else {
        mTask.acc_cnt++;
        if (mSensor->idx != GEOMAG)
            mTask.gyr_cnt++;
        if (mSensor->idx != GAME && mSensor->idx != GRAVITY)
            mTask.mag_cnt++;
    }

    configureFusion();
    configureGame();
    sensorSignalInternalEvt(mSensor->handle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);

    return true;
}

static bool orientPower(bool on)
{
    return fusionPower(&mTask.sensors[ORIENT], on);
}

static bool geoMagPower(bool on)
{
    return fusionPower(&mTask.sensors[GEOMAG], on);
}

static bool linearAccPower(bool on)
{
    return fusionPower(&mTask.sensors[LINEAR], on);
}

static bool gamePower(bool on)
{
    return fusionPower(&mTask.sensors[GAME], on);
}

static bool rotationPower(bool on)
{
    return fusionPower(&mTask.sensors[ROTAT], on);
}

static bool gravityPower(bool on)
{
    return fusionPower(&mTask.sensors[GRAVITY], on);
}

static bool fusionSetRate(struct FusionSensor *mSensor, uint32_t rate, uint64_t latency)
{
    mSensor->rate = rate;
    mSensor->latency = latency;
    fusionSetRateAcc(mSensor);
    fusionSetRateGyr(mSensor);
    fusionSetRateMag(mSensor);
    sensorSignalInternalEvt(mSensor->handle, SENSOR_INTERNAL_EVT_RATE_CHG, rate, latency);

    return true;
}

static bool orientSetRate(uint32_t rate, uint64_t latency)
{
    return fusionSetRate(&mTask.sensors[ORIENT], rate, latency);
}

static bool gravitySetRate(uint32_t rate, uint64_t latency)
{
    return fusionSetRate(&mTask.sensors[GRAVITY], rate, latency);
}

static bool linearAccSetRate(uint32_t rate, uint64_t latency)
{
    return fusionSetRate(&mTask.sensors[LINEAR], rate, latency);
}

static bool geoMagSetRate(uint32_t rate, uint64_t latency)
{
    return fusionSetRate(&mTask.sensors[GEOMAG], rate, latency);
}

static bool gameSetRate(uint32_t rate, uint64_t latency)
{
    return fusionSetRate(&mTask.sensors[GAME], rate, latency);
}

static bool rotationSetRate(uint32_t rate, uint64_t latency)
{
    return fusionSetRate(&mTask.sensors[ROTAT], rate, latency);
}

static bool fusionFirmwareUpload(struct FusionSensor *mSensor)
{
    sensorSignalInternalEvt(mSensor->handle, SENSOR_INTERNAL_EVT_FW_STATE_CHG, 1, 0);
    return true;
}

static bool orientFirmwareUpload()
{
    return fusionFirmwareUpload(&mTask.sensors[ORIENT]);
}

static bool gravityFirmwareUpload()
{
    return fusionFirmwareUpload(&mTask.sensors[GRAVITY]);
}

static bool geoMagFirmwareUpload()
{
    return fusionFirmwareUpload(&mTask.sensors[GEOMAG]);
}

static bool linearAccFirmwareUpload()
{
    return fusionFirmwareUpload(&mTask.sensors[LINEAR]);
}

static bool gameFirmwareUpload()
{
    return fusionFirmwareUpload(&mTask.sensors[GAME]);
}

static bool rotationFirmwareUpload()
{
    return fusionFirmwareUpload(&mTask.sensors[ROTAT]);
}

static bool fusionFlush(struct FusionSensor *mSensor)
{
    uint32_t evtType = sensorGetMyEventType(mSi[mSensor->idx].sensorType);
    if (mSensor->ev)
        osEnqueueEvt(evtType, mSensor->ev, dataEvtFree);
    osEnqueueEvt(evtType, SENSOR_DATA_EVENT_FLUSH, NULL);
    return true;
}

static bool orientFlush()
{
    return fusionFlush(&mTask.sensors[ORIENT]);
}

static bool gravityFlush()
{
    return fusionFlush(&mTask.sensors[GRAVITY]);
}

static bool linearAccFlush()
{
    return fusionFlush(&mTask.sensors[LINEAR]);
}

static bool gameFlush()
{
    return fusionFlush(&mTask.sensors[GAME]);
}

static bool geoMagFlush()
{
    return fusionFlush(&mTask.sensors[GEOMAG]);
}

static bool rotationFlush()
{
    return fusionFlush(&mTask.sensors[ROTAT]);
}


static void fusionHandleEvent(uint32_t evtType, const void* evtData)
{
    struct TripleAxisDataEvent *ev;

    if (evtData == SENSOR_DATA_EVENT_FLUSH)
        return;

    switch (evtType) {
    case EVT_SENSOR_ACC_DATA_RDY:
        ev = (struct TripleAxisDataEvent *)evtData;
        fillSamples(ev, ACC);
        drainSamples();
        break;
    case EVT_SENSOR_GYR_DATA_RDY:
        ev = (struct TripleAxisDataEvent *)evtData;
        fillSamples(ev, GYR);
        drainSamples();
        break;
    case EVT_SENSOR_MAG_DATA_RDY:
        ev = (struct TripleAxisDataEvent *)evtData;
        fillSamples(ev, MAG);
        drainSamples();
        break;
    }
}

static const struct SensorOps mSops[NUM_OF_FUSION_SENSOR] =
{
    {orientPower, orientFirmwareUpload, orientSetRate, orientFlush, NULL, NULL},
    {gravityPower, gravityFirmwareUpload, gravitySetRate, gravityFlush, NULL, NULL},
    {geoMagPower, geoMagFirmwareUpload, geoMagSetRate, geoMagFlush, NULL, NULL},
    {linearAccPower, linearAccFirmwareUpload, linearAccSetRate, linearAccFlush, NULL, NULL},
    {gamePower, gameFirmwareUpload, gameSetRate, gameFlush, NULL, NULL},
    {rotationPower, rotationFirmwareUpload, rotationSetRate, rotationFlush, NULL, NULL},
};

static bool fusionStart(uint32_t tid)
{
    osLog(LOG_INFO, "        ORIENTATION:  %ld\n", tid);
    size_t i, slabSize;

    mTask.tid = tid;
    mTask.last_gyro_time = 0ull;
    mTask.last_acc_time = 0ull;
    mTask.flags = 0;

    for (i = 0; i < NUM_OF_RAW_SENSOR; i++) {
         mTask.sample_counts[i] = 0;
         mTask.sample_indices[i] = 0;
    }

    for (i = ORIENT; i < NUM_OF_FUSION_SENSOR; i++) {
        mTask.sensors[i].handle = sensorRegister(&mSi[i], &mSops[i]);
        mTask.sensors[i].idx = i;
        mTask.sensors[i].use_gyr_data = true;
        mTask.sensors[i].use_mag_data = true;
    }

    mTask.sensors[GEOMAG].use_gyr_data = false;
    mTask.sensors[GAME].use_mag_data = false;
    mTask.sensors[GRAVITY].use_mag_data = false;


    slabSize = sizeof(struct TripleAxisDataEvent)
        + MAX_NUM_COMMS_EVENT_SAMPLES * sizeof(struct TripleAxisDataPoint);
    mDataSlab = slabAllocatorNew(slabSize, 4, 10); // 10 slots for now..

    return true;
}

static void fusionEnd()
{
    mTask.flags &= ~FUSION_FLAG_INITIALIZED;
    slabAllocatorDestroy(mDataSlab);
}

INTERNAL_APP_INIT(
        0x0000000000000004ULL,
        fusionStart,
        fusionEnd,
        fusionHandleEvent);
