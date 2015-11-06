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
#include <sensors.h>
#include <limits.h>

#define EVT_SENSOR_ACC_DATA_RDY sensorGetMyEventType(SENS_TYPE_ACCEL)

#define ACCEL_MIN_RATE    SENSOR_HZ(50)
#define ACCEL_MAX_LATENCY 250000000ull   // 40 ms

#define BATCH_TIME      (2000000) // 2.0 seconds
#define ANGLE_THRESH    (0.819 * 9.81 * 9.81) // ~cos(35) * (1G in m/s^2)^2

struct TiltAlgoState {
    uint64_t this_batch_init_ts;
    uint32_t this_batch_num_samples;
    float this_batch_sample_sum[3];
    float this_batch_g[3];
    float last_ref_g_vector[3];
    bool last_ref_g_vector_valid;
    bool anamoly_this_batch;
    bool tilt_detected;
};

static struct TiltDetectionTask {
    struct TiltAlgoState algoState;
    uint32_t taskId;
    uint32_t handle;
    uint32_t accelHandle;
} mTask;

// *****************************************************************************

static void algoInit()
{
    memset(&mTask.algoState, 0, sizeof(struct TiltAlgoState));
}

static bool algoUpdate(struct TripleAxisDataEvent *ev)
{
    float dotProduct = 0.0f;
    uint64_t dt;
    bool latch_g_vector = false;
    bool tilt_detected = false;
    struct TiltAlgoState *state = &mTask.algoState;
    uint64_t referenceTime = ev->referenceTime;
    uint32_t numSamples = ev->samples[0].numSamples;
    uint32_t i;
    struct TripleAxisDataPoint *sample;
    uint64_t sample_ts;
    float invN;

    for (i = 0; i < numSamples; i++) {
        sample = &ev->samples[i];
        sample_ts = referenceTime + sample->deltaTime;

        if(state->this_batch_init_ts == 0) {
            state->this_batch_init_ts = sample_ts;
        }

        state->this_batch_sample_sum[0] += sample->x;
        state->this_batch_sample_sum[1] += sample->y;
        state->this_batch_sample_sum[2] += sample->z;

        state->this_batch_num_samples++;

        dt = (sample_ts - state->this_batch_init_ts);

        if(dt > BATCH_TIME) {
            invN = 1.0f / state->this_batch_num_samples;
            state->this_batch_g[0] = state->this_batch_sample_sum[0] * invN;
            state->this_batch_g[1] = state->this_batch_sample_sum[1] * invN;
            state->this_batch_g[2] = state->this_batch_sample_sum[2] * invN;

            if(state->last_ref_g_vector_valid) {
                dotProduct = state->this_batch_g[0] * state->last_ref_g_vector[0] +
                    state->this_batch_g[1] * state->last_ref_g_vector[1] +
                    state->this_batch_g[2] * state->last_ref_g_vector[2];

                if (dotProduct < ANGLE_THRESH) {
                    tilt_detected = true;
                    latch_g_vector = true;
                }
            } else { // reference g vector not valid, first time computing
                latch_g_vector = true;
            }

            if(latch_g_vector) {
                state->last_ref_g_vector_valid = true;
                state->last_ref_g_vector[0] = state->this_batch_g[0];
                state->last_ref_g_vector[1] = state->this_batch_g[1];
                state->last_ref_g_vector[2] = state->this_batch_g[2];
            }

            // Seed the next batch
            state->this_batch_init_ts = sample_ts;
            state->this_batch_num_samples = 0;
            state->this_batch_sample_sum[0] = 0;
            state->this_batch_sample_sum[1] = 0;
            state->this_batch_sample_sum[2] = 0;
        }
    }

    return tilt_detected;
}

// *****************************************************************************

static const struct SensorInfo mSi =
{
    "Tilt Detection",
    NULL,
    SENS_TYPE_TILT,
    NUM_AXIS_EMBEDDED,
    { NANOHUB_INT_WAKEUP }
};

static bool tiltDetectionPower(bool on)
{
    if (on) {
        sensorRequest(mTask.taskId, mTask.accelHandle, ACCEL_MIN_RATE,
                      ACCEL_MAX_LATENCY);
        osEventSubscribe(mTask.taskId, EVT_SENSOR_ACC_DATA_RDY);
    } else {
        sensorRelease(mTask.taskId, mTask.accelHandle);
        osEventUnsubscribe(mTask.taskId, EVT_SENSOR_ACC_DATA_RDY);
    }

    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG,
                            on, 0);
    return true;
}

static bool tiltDetectionSetRate(uint32_t rate, uint64_t latency)
{
    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_RATE_CHG, rate,
                            latency);
    return true;
}

static bool tiltDetectionFirmwareUpload()
{
    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_FW_STATE_CHG,
            1, 0);
    return true;
}

static bool tiltDetectionFlush()
{
    return osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_TILT),
                        SENSOR_DATA_EVENT_FLUSH, NULL);
}

static void tiltDetectionHandleEvent(uint32_t evtType, const void* evtData)
{
    if (evtData == SENSOR_DATA_EVENT_FLUSH)
        return;

    switch (evtType) {
    case EVT_APP_START:
        osLog(LOG_INFO, "[Tilt] idle\n");
        osEventUnsubscribe(mTask.taskId, EVT_APP_START);
        sensorFind(SENS_TYPE_ACCEL, 0, &mTask.accelHandle);
        break;

    case EVT_SENSOR_ACC_DATA_RDY:
        if (algoUpdate((struct TripleAxisDataEvent *)evtData)) {
            union EmbeddedDataPoint sample;
            sample.idata = 1;
            osEnqueueEvt(sensorGetMyEventType(SENS_TYPE_TILT), sample.vptr, NULL);
        }
        break;
    }
}

static const struct SensorOps mSops =
{
    tiltDetectionPower,
    tiltDetectionFirmwareUpload,
    tiltDetectionSetRate,
    tiltDetectionFlush,
    NULL,
};

static bool tiltDetectionStart(uint32_t taskId)
{
    mTask.taskId = taskId;
    mTask.handle = sensorRegister(&mSi, &mSops);
    algoInit();
    osEventSubscribe(taskId, EVT_APP_START);
    return true;
}

static void tiltDetectionEnd()
{
}

INTERNAL_APP_INIT(
        0x0000000000000008ULL,
        tiltDetectionStart,
        tiltDetectionEnd,
        tiltDetectionHandleEvent);

