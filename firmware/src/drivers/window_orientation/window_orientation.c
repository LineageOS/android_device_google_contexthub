#include <stdlib.h>
#include <string.h>
#include <timer.h>
#include <heap.h>
#include <plat/inc/rtc.h>
#include <plat/inc/syscfg.h>
#include <hostIntf.h>

#include <seos.h>
#include <accelerometer.h>

#include <nanohub_math.h>
#include <fusion/fusion.h>
#include <sensors.h>
#include <limits.h>

// all time units in usec, angles in degrees
#define RADIANS_TO_DEGREES                              (180.0f / M_PI)

#define PROPOSAL_SETTLE_TIME                            40000       // 40 ms
#define PROPOSAL_MIN_TIME_SINCE_FLAT_ENDED              500000      // 500 ms
#define PROPOSAL_MIN_TIME_SINCE_SWING_ENDED             300000      // 300 ms
#define PROPOSAL_MIN_TIME_SINCE_ACCELERATION_ENDED      500000      // 500 ms
// #define PROPOSAL_MIN_TIME_SINCE_TOUCH_END               500000   // 500 ms

#define FLAT_ANGLE                      80
#define FLAT_TIME                       1000000     // 1 sec

#define SWING_AWAY_ANGLE_DELTA          20
#define SWING_TIME                      300000      // 300 ms

#define MAX_FILTER_DELTA_TIME           1000000     // 1 sec
#define FILTER_TIME_CONSTANT            200000      // 200 ms

#define NEAR_ZERO_MAGNITUDE             1.0f        // m/s^2
#define ACCELERATION_TOLERANCE          4.0f
#define STANDARD_GRAVITY                9.8f
#define MIN_ACCELERATION_MAGNITUDE  (STANDARD_GRAVITY - ACCELERATION_TOLERANCE)
#define MAX_ACCELERATION_MAGNITUDE  (STANDARD_GRAVITY + ACCELERATION_TOLERANCE)

#define MAX_TILT                        80
#define TILT_OVERHEAD_ENTER             -40
#define TILT_OVERHEAD_EXIT              -15

#define ADJACENT_ORIENTATION_ANGLE_GAP  45

#define TILT_HISTORY_SIZE               200
#define TILT_REFERENCE_PERIOD           1800000000  // 30 min
#define TILT_REFERENCE_BACKOFF          300000000   // 5 min

#define MIN_ACCEL_INTERVAL              33333       // 33.3 ms for 30 Hz

#define EVT_SENSOR_ACC_DATA_RDY sensorGetMyEventType(SENS_TYPE_ACCEL)
#define EVT_SENSOR_WINDOW_ORIENTATION_DATA_RDY sensorGetMyEventType(SENS_TYPE_WINDOW_ORIENTATION)

static int8_t Tilt_Tolerance[4][2] = {
    /* ROTATION_0   */ { -25, 70 },
    /* ROTATION_90  */ { -25, 65 },
    /* ROTATION_180 */ { -25, 60 },
    /* ROTATION_270 */ { -25, 65 }
};

struct ConfigStat {
   uint32_t rate;
   uint64_t latency;
   uint8_t clientId;
   uint8_t enable : 1;
   uint8_t flush : 1;
   uint8_t calibrate : 1;
} __attribute__((packed));

static uint32_t window_orientation_rates[] = {
    SENSOR_HZ(12.5f),
    SENSOR_HZ(25.0f),
    SENSOR_HZ(50.0f),
    SENSOR_HZ(100.0f),
    SENSOR_HZ(200.0f),
    0,
};

struct WindowOrientationTask {
    uint32_t tid;
    uint32_t handle;
    uint32_t rate;
    uint64_t latency;
    static const struct SensorInfo si;

    uint64_t last_filtered_time;
    struct sensor_sample last_filtered_sample;

    uint64_t tilt_reference_time;
    uint64_t accelerating_time;
    uint64_t predicted_rotation_time;
    uint64_t flat_time;
    uint64_t swinging_time;

    uint32_t tilt_history_time[TILT_HISTORY_SIZE];
    int tilt_history_index;
    int8_t tilt_history[TILT_HISTORY_SIZE];

    int8_t current_rotation;
    int8_t prev_valid_rotation;
    int8_t proposed_rotation;
    int8_t predicted_rotation;

    bool flat;
    bool swinging;
    bool accelerating;
    bool overhead;

    bool active;
};

static struct WindowOrientationTask mTask;
static struct ConfigStat mAccelConfig;

static void extDataEvtFree(void *ptr)
{
    struct ext_data_evt *ev = (struct ext_data_evt *)ptr;
    heapFree(ev);
}

static bool isTiltAngleAcceptable(int rotation, int8_t tilt_angle)
{
    return ((tilt_angle >= Tilt_Tolerance[rotation][0])
                && (tilt_angle <= Tilt_Tolerance[rotation][1]));
}

static bool isOrientationAngleAcceptable(int current_rotation, int rotation,
                                            int orientation_angle)
{
    // If there is no current rotation, then there is no gap.
    // The gap is used only to introduce hysteresis among advertised orientation
    // changes to avoid flapping.

    if (current_rotation >= 0) {
        // If the specified rotation is the same or is counter-clockwise
        // adjacent to the current rotation, then we set a lower bound on the
        // orientation angle.
        // For example, if currentRotation is ROTATION_0 and proposed is
        // ROTATION_90, then we want to check orientationAngle > 45 + GAP / 2.
        if ((rotation == current_rotation)
                || (rotation == (current_rotation + 1) % 4)) {
            int lower_bound = rotation * 90 - 45
                    + ADJACENT_ORIENTATION_ANGLE_GAP / 2;
            if (rotation == 0) {
                if ((orientation_angle >= 315)
                        && (orientation_angle < lower_bound + 360)) {
                    return false;
                }
            } else {
                if (orientation_angle < lower_bound) {
                    return false;
                }
            }
        }

        // If the specified rotation is the same or is clockwise adjacent,
        // then we set an upper bound on the orientation angle.
        // For example, if currentRotation is ROTATION_0 and rotation is
        // ROTATION_270, then we want to check orientationAngle < 315 - GAP / 2.
        if ((rotation == current_rotation)
                || (rotation == (current_rotation + 3) % 4)) {
            int upper_bound = rotation * 90 + 45
                    - ADJACENT_ORIENTATION_ANGLE_GAP / 2;
            if (rotation == 0) {
                if ((orientation_angle <= 45)
                        && (orientation_angle > upper_bound)) {
                    return false;
                }
            } else {
                if (orientation_angle > upper_bound) {
                    return false;
                }
            }
        }
    }
    return true;
}

static bool isPredictedRotationAcceptable(uint64_t now)
{
    // The predicted rotation must have settled long enough.
    if (now < mTask.predicted_rotation_time + PROPOSAL_SETTLE_TIME) {
        return false;
    }

    // The last flat state (time since picked up) must have been sufficiently
    // long ago.
    if (now < mTask.flat_time + PROPOSAL_MIN_TIME_SINCE_FLAT_ENDED) {
        return false;
    }

    // The last swing state (time since last movement to put down) must have
    // been sufficiently long ago.
    if (now < mTask.swinging_time + PROPOSAL_MIN_TIME_SINCE_SWING_ENDED) {
        return false;
    }

    // The last acceleration state must have been sufficiently long ago.
    if (now < mTask.accelerating_time
            + PROPOSAL_MIN_TIME_SINCE_ACCELERATION_ENDED) {
        return false;
    }

    // Looks good!
    return true;
}

static void clearPredictedRotation()
{
    mTask.predicted_rotation = -1;
    mTask.predicted_rotation_time = 0;
}

static void clearTiltHistory()
{
    mTask.tilt_history_time[0] = 0;
    mTask.tilt_history_index = 1;
    mTask.tilt_reference_time = 0;
}

static void reset()
{
    mTask.last_filtered_time = 0;
    mTask.proposed_rotation = -1;

    mTask.flat_time = 0;
    mTask.flat = false;

    mTask.swinging_time = 0;
    mTask.swinging = false;

    mTask.accelerating_time = 0;
    mTask.accelerating = false;

    mTask.overhead = false;

    clearPredictedRotation();
    clearTiltHistory();
}

static void updatePredictedRotation(uint64_t now, int rotation)
{
    if (mTask.predicted_rotation != rotation) {
        mTask.predicted_rotation = rotation;
        mTask.predicted_rotation_time = now;
    }
}

static bool isAccelerating(float magnitude)
{
    return ((magnitude < MIN_ACCELERATION_MAGNITUDE)
                || (magnitude > MAX_ACCELERATION_MAGNITUDE));
}

static void addTiltHistoryEntry(uint64_t now, int8_t tilt)
{
    if (mTask.tilt_reference_time == 0) {
        // set reference_time after reset()

        mTask.tilt_reference_time = now - 1;
    } else if (mTask.tilt_reference_time + TILT_REFERENCE_PERIOD < now) {
        // proactively shift reference_time every 30 min,
        // all history entries are within 5 min interval (15Hz x 200 samples)

        uint64_t old_reference_time = mTask.tilt_reference_time;
        mTask.tilt_reference_time = now - TILT_REFERENCE_BACKOFF;

        uint64_t delta = mTask.tilt_reference_time - old_reference_time;
        size_t i;
        for (i = 0; i < TILT_HISTORY_SIZE; ++i) {
            mTask.tilt_history_time[i] = (mTask.tilt_history_time[i] > delta)
                ? (mTask.tilt_history_time[i] - delta) : 0;
        }
    }

    int index = mTask.tilt_history_index;
    mTask.tilt_history[index] = tilt;
    mTask.tilt_history_time[index] = now - mTask.tilt_reference_time;

    index = ((index + 1) == TILT_HISTORY_SIZE) ? 0 : (index + 1);
    mTask.tilt_history_index = index;
    mTask.tilt_history_time[index] = 0;
}

static int nextTiltHistoryIndex(int index)
{
    int next = (index == 0) ? (TILT_HISTORY_SIZE - 1): (index - 1);
    return ((mTask.tilt_history_time[next] != 0) ? next : -1);
}

static bool isFlat(uint64_t now)
{
    int i = mTask.tilt_history_index;
    for (; (i = nextTiltHistoryIndex(i)) >= 0;) {
        if (mTask.tilt_history[i] < FLAT_ANGLE) {
            break;
        }
        if (mTask.tilt_reference_time + mTask.tilt_history_time[i] + FLAT_TIME <= now) {
            // Tilt has remained greater than FLAT_ANGLE for FLAT_TIME.
            return true;
        }
    }
    return false;
}

static bool isSwinging(uint64_t now, int8_t tilt)
{
    int i = mTask.tilt_history_index;
    for (; (i = nextTiltHistoryIndex(i)) >= 0;) {
        if (mTask.tilt_reference_time + mTask.tilt_history_time[i] + SWING_TIME
                < now) {
            break;
        }
        if (mTask.tilt_history[i] + SWING_AWAY_ANGLE_DELTA <= tilt) {
            // Tilted away by SWING_AWAY_ANGLE_DELTA within SWING_TIME.
            return true;
        }
    }
    return false;
}


static bool add_samples(struct data_evt *ev)
{
    float x, y, z;

    int i;
    for (i = 0; i < ev->num_samples; i++) {

        x = ev->samples[i].x;
        y = ev->samples[i].y;
        z = ev->samples[i].z;

        // Apply a low-pass filter to the acceleration up vector in cartesian space.
        // Reset the orientation listener state if the samples are too far apart in time.

        uint64_t now = ev->reference_time + ev->samples[i].delta_time;

        struct sensor_sample *last_sample = &mTask.last_filtered_sample;
        uint64_t then = mTask.last_filtered_time;
        uint64_t time_delta = now - then;

        bool skip_sample;
        if ((now < then) || (now > then + MAX_FILTER_DELTA_TIME)) {
            reset();
            skip_sample = true;
        } else {
            // alpha is the weight on the new sample
            float alpha = (float)time_delta / (FILTER_TIME_CONSTANT + time_delta);
            x = alpha * (x - last_sample->x) + last_sample->x;
            y = alpha * (y - last_sample->y) + last_sample->y;
            z = alpha * (z - last_sample->z) + last_sample->z;

            skip_sample = false;
        }

        // drop samples when input sampling rate is 2x higher than requested
        if (!skip_sample && (time_delta < MIN_ACCEL_INTERVAL)) {
            skip_sample = true;
        } else {
            mTask.last_filtered_time = now;
            mTask.last_filtered_sample.x = x;
            mTask.last_filtered_sample.y = y;
            mTask.last_filtered_sample.z = z;
        }

        bool accelerating = false;
        bool flat = false;
        bool swinging = false;

        if (!skip_sample) {
            // Calculate the magnitude of the acceleration vector.
            float magnitude = sqrtf(x * x + y * y + z * z);

            if (magnitude < NEAR_ZERO_MAGNITUDE) {
                clearPredictedRotation();
            } else {
                // Determine whether the device appears to be undergoing
                // external acceleration.
                if (isAccelerating(magnitude)) {
                    accelerating = true;
                    mTask.accelerating_time = now;
                }

                // Calculate the tilt angle.
                // This is the angle between the up vector and the x-y plane
                // (the plane of the screen) in a range of [-90, 90] degrees.
                //  -90 degrees: screen horizontal and facing the ground (overhead)
                //    0 degrees: screen vertical
                //   90 degrees: screen horizontal and facing the sky (on table)
                int tilt_tmp = (int)(asinf(z / magnitude) * RADIANS_TO_DEGREES);
                tilt_tmp = (tilt_tmp > 127) ? 127 : tilt_tmp;
                tilt_tmp = (tilt_tmp < -128) ? -128 : tilt_tmp;
                int8_t tilt_angle = tilt_tmp;
                addTiltHistoryEntry(now, tilt_angle);

                // Determine whether the device appears to be flat or swinging.
                if (isFlat(now)) {
                    flat = true;
                    mTask.flat_time = now;
                }
                if (isSwinging(now, tilt_angle)) {
                    swinging = true;
                    mTask.swinging_time = now;
                }

                // If the tilt angle is too close to horizontal then we cannot
                // determine the orientation angle of the screen.
                if (tilt_angle <= TILT_OVERHEAD_ENTER) {
                    mTask.overhead = true;
                } else if (tilt_angle >= TILT_OVERHEAD_EXIT) {
                    mTask.overhead = false;
                }

                if (mTask.overhead) {
                    clearPredictedRotation();
                } else if (fabsf(tilt_angle) > MAX_TILT) {
                    clearPredictedRotation();
                } else {
                    // Calculate the orientation angle.
                    // This is the angle between the x-y projection of the up
                    // vector onto the +y-axis, increasing clockwise in a range
                    // of [0, 360] degrees.
                    int orientation_angle = (int)(-atan2f(-x, y) * RADIANS_TO_DEGREES);
                    if (orientation_angle < 0) {
                        // atan2 returns [-180, 180]; normalize to [0, 360]
                        orientation_angle += 360;
                    }

                    // Find the nearest rotation.
                    int nearest_rotation = (orientation_angle + 45) / 90;
                    if (nearest_rotation == 4) {
                        nearest_rotation = 0;
                    }
                    // Determine the predicted orientation.
                    if (isTiltAngleAcceptable(nearest_rotation, tilt_angle)
                        && isOrientationAngleAcceptable(mTask.current_rotation,
                                                           nearest_rotation,
                                                           orientation_angle)) {
                        updatePredictedRotation(now, nearest_rotation);
                    } else {
                        clearPredictedRotation();
                    }
                }
            }
        }

        mTask.flat = flat;
        mTask.swinging = swinging;
        mTask.accelerating = accelerating;

        // Determine new proposed rotation.
        int8_t old_proposed_rotation = mTask.proposed_rotation;
        if ((mTask.predicted_rotation < 0)
                || isPredictedRotationAcceptable(now)) {

            mTask.proposed_rotation = mTask.predicted_rotation;
        }
        int8_t proposed_rotation = mTask.proposed_rotation;

        //dprintf("rotation: current %d, proposed %d, predicted %d: a %d f %d s %d o %d\r\n",
        //        mTask.current_rotation, proposed_rotation, mTask.predicted_rotation,
        //        accelerating, flat, swinging, mTask.overhead);

        if ((proposed_rotation != old_proposed_rotation)
                && (proposed_rotation >= 0)) {
            mTask.current_rotation = proposed_rotation;

            bool change_detected = true;
            change_detected = (proposed_rotation != mTask.prev_valid_rotation);
            mTask.prev_valid_rotation = proposed_rotation;

            if (change_detected) {
                // dprintf("window orientation change: %d -> %d\r\n",
                //         old_proposed_rotation, proposed_rotation);
                return true;
            }
        }
    }

    return false;
}


static bool windowOrientationPower(bool on)
{
    if (on) {
        // if this is power on, we enable the imu when set rate.
        mTask.active = true;

        osEventSubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
    } else {
        mTask.active = false;

        osEventUnsubscribe(mTask.tid, EVT_SENSOR_ACC_DATA_RDY);
        osEnqueueEvt(EVT_SENSOR_ACC_CONFIG, &mAccelConfig, NULL, false);
    }

    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_POWER_STATE_CHG, on, 0);

    return true;
}

static bool windowOrientationSetRate(uint32_t rate, uint64_t latency)
{
    mTask.rate = rate;
    mTask.latency = latency;

    osLog(LOG_INFO, "SENDING TO ACCEL TO ACTIVATE\n");
    osEnqueueEvt(EVT_SENSOR_ACC_CONFIG, &mAccelConfig, NULL, false);

    return true;
}

static bool windowOrientationFirmwareUpload()
{
    sensorSignalInternalEvt(mTask.handle, SENSOR_INTERNAL_EVT_FW_STATE_CHG,
            mTask.rate, mTask.latency);
    return true;
}

static void config(struct ConfigStat *configCmd)
{
    if (configCmd->enable == 0 && mTask.active) {
        // if we are on, cmd is to turn off, release sensor
        sensorRelease(configCmd->clientId, mTask.handle);
    } else if (configCmd->enable == 1 && !mTask.active) {
        // if we are off, cmd is to turn on, request sensor
        sensorRequest(configCmd->clientId, mTask.handle, configCmd->rate, configCmd->latency);
    } else {
        // if we are on, cmd is to turn on, change the config if needed.
        sensorRequestRateChange(configCmd->clientId, mTask.handle, configCmd->rate, configCmd->latency);
    }
}

static void windowOrientationHandleEvent(uint32_t evtType, const void* evtData)
{
    struct data_evt *ev;
    struct ConfigStat *configCmd;

    switch (evtType) {
        case EVT_SENSOR_WINDOW_ORIENTATION_CONFIG:
            osLog(LOG_INFO, "RECEIVED WIN_ORIEN EVT\n");
            configCmd = (struct ConfigStat *)evtData;
            memcpy(&mAccelConfig, evtData, sizeof(mAccelConfig));
            mAccelConfig.clientId = mTask.tid;
            config(configCmd);
            break;
        case EVT_SENSOR_ACC_DATA_RDY:
            ev = (struct data_evt *)evtData;
            bool rotation_changed = add_samples(ev);

            if (rotation_changed) {
                osLog(LOG_INFO, "     ********** rotation changed to ********: %d\n", mTask.proposed_rotation);
                struct ext_data_evt *ext_data_evt = heapAlloc(sizeof(struct ext_data_evt));
                ext_data_evt->size = 250;
                ext_data_evt->data.reference_time = ev->reference_time;
                ext_data_evt->data.type = SENS_TYPE_WINDOW_ORIENTATION;
                ext_data_evt->data.num_samples = 1;
                ext_data_evt->data.samples[0].delta_time = 0;
                ext_data_evt->data.samples[0].x = mTask.proposed_rotation;
                ext_data_evt->data.samples[0].y = 0.0f;
                ext_data_evt->data.samples[0].z = 0.0f;

                osEnqueueEvt(EVT_SENSOR_WINDOW_ORIENTATION_DATA_RDY, ext_data_evt, extDataEvtFree, true);
            }
            break;
        default:
            osLog(LOG_ERROR, "WINDOW ORIENTATION SENSOR: Invalid EvtType: %ld\n", evtType);
            break;
    }
}

static bool window_orientation_start(uint32_t tid)
{
    osLog(LOG_INFO, "        WINDOW ORIENTATION:  %ld\n", tid);

    mTask.tid = tid;

    mTask.si.sensorName = "Window Orientation";
    mTask.si.supportedRates = window_orientation_rates;
    mTask.si.ops.sensorPower = windowOrientationPower;
    mTask.si.ops.sensorFirmwareUpload = windowOrientationFirmwareUpload;
    mTask.si.ops.sensorSetRate = windowOrientationSetRate;
    mTask.si.ops.sensorTriggerOndemand = NULL;
    mTask.si.sensorType = SENS_TYPE_WINDOW_ORIENTATION;
    mTask.si.numAxes = 1;

    mTask.current_rotation = -1;
    mTask.prev_valid_rotation = -1;
    reset();

    mTask.handle = sensorRegister(&mTask.si);

    mTask.rate = SENSOR_HZ(10.0f);
    mTask.active = false;
    mTask.latency = 0;

    osEventSubscribe(mTask.tid, EVT_SENSOR_WINDOW_ORIENTATION_CONFIG);
    return true;
}

static void windowOrientationEnd()
{
}

INTERNAL_APP_INIT(
        0x0000000000000006ULL,
        window_orientation_start,
        windowOrientationEnd,
        windowOrientationHandleEvent);

