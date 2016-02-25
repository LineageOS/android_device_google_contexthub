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

#define LOG_TAG "sensors"
// #defined LOG_NDEBUG  1
#include <utils/Log.h>

#include "sensors.h"

#include "hubconnection.h"

#include <errno.h>
#include <math.h>
#include <media/stagefright/foundation/ADebug.h>
#include <string.h>

using namespace android;

static const int kVersion = 1;

static const float kMinSampleRateHzAccel = 6.250f;
static const float kMaxSampleRateHzAccel = 200.0f;

static const float kMinSampleRateHzGyro = 6.250f;
static const float kMaxSampleRateHzGyro = 200.0f;

static const float kMinSampleRateHzMag = 3.125f;
static const float kMaxSampleRateHzMag = 50.0f;

static const float kMinSampleRateHzPolling = 0.1f;
static const float kMaxSampleRateHzPolling = 25.0f;

static const float kMinSampleRateHzPressure = 0.1f;
static const float kMaxSampleRateHzPressure = 10.0f;

static const float kMinSampleRateHzTemperature = kMinSampleRateHzPolling;
static const float kMaxSampleRateHzTemperature = kMaxSampleRateHzPolling;

static const float kMinSampleRateHzProximity = kMinSampleRateHzPolling;
static const float kMaxSampleRateHzProximity = 5.0;

static const float kMinSampleRateHzLight = kMinSampleRateHzPolling;
static const float kMaxSampleRateHzLight = 5.0;

static const float kMinSampleRateHzOrientation = 12.5f;
static const float kMaxSampleRateHzOrientation = 200.0f;

static const int kMaxOneAxisEventCount = 9000;
static const int kMaxThreeAxisEventCount = 4500;

static const char SENSOR_STRING_TYPE_INTERNAL_TEMPERATURE[] =
        "com.google.sensor.internal_temperature";
static const char SENSOR_STRING_TYPE_SYNC[] =
        "com.google.sensor.sync";
static const char SENSOR_STRING_TYPE_DOUBLE_TWIST[] =
        "com.google.sensor.double_twist";
static const char SENSOR_STRING_TYPE_DOUBLE_TAP[] =
        "com.google.sensor.double_tap";

static struct sensor_t kSensorList[] = {
    {
        // MUST BE THE FIRST ENTRY! The name/vendor will be patched once we
        // get feedback from the hub on which part is installed.

        "Unknown Proximity Sensor",
        "Unknown",
        kVersion,
        COMMS_SENSOR_PROXIMITY,
        SENSOR_TYPE_PROXIMITY,
        5.0f,                                          // maxRange (cm)
        1.0f,                                          // resolution (cm)
        0.0f,                                          // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzProximity), // minDelay
        300,                                           // XXX fifoReservedEventCount
        kMaxOneAxisEventCount,                         // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_PROXIMITY,
        "",                                            // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzProximity),    // maxDelay
        SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_ON_CHANGE_MODE,
        { NULL, NULL }
    },
    {
        "Unknown Light Sensor",
        "Unknown",
        kVersion,
        COMMS_SENSOR_LIGHT,
        SENSOR_TYPE_LIGHT,
        43000.0f,                                  // maxRange (lx)
        10.0f,                                     // XXX resolution (lx)
        0.0f,                                      // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzLight), // minDelay
        0,                                         // XXX fifoReservedEventCount
        0,                                         // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_LIGHT,
        "",                                        // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzLight),    // maxDelay
        SENSOR_FLAG_ON_CHANGE_MODE,
        { NULL, NULL }
    },
    {
        "BMI160 accelerometer",
        "Bosch",
        kVersion,
        COMMS_SENSOR_ACCEL,
        SENSOR_TYPE_ACCELEROMETER,
        GRAVITY_EARTH * 8.0f,                      // maxRange
        GRAVITY_EARTH * 8.0f / 32768.0f,           // resolution
        0.0f,                                      // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzAccel), // minDelay
        3000,                                      // XXX fifoReservedEventCount
        kMaxThreeAxisEventCount,                   // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_ACCELEROMETER,
        "",                                        // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzAccel),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMI160 gyroscope",
        "Bosch",
        kVersion,
        COMMS_SENSOR_GYRO,
        SENSOR_TYPE_GYROSCOPE,
        2000.0f * M_PI / 180.0f,                   // maxRange
        2000.0f * M_PI / (180.0f * 32768.0f),      // resolution
        0.0f,                                      // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzGyro),  // minDelay
        0,                                         // XXX fifoReservedEventCount
        0,                                         // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_GYROSCOPE,
        "",                                        // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzGyro),     // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMI160 gyroscope (uncalibrated)",
        "Bosch",
        kVersion,
        COMMS_SENSOR_GYRO_UNCALIBRATED,
        SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
        2000.0f * M_PI / 180.0f,                   // maxRange
        2000.0f * M_PI / (180.0f * 32768.0f),      // resolution
        0.0f,                                      // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzGyro),  // minDelay
        0,                                         // XXX fifoReservedEventCount
        0,                                         // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_GYROSCOPE_UNCALIBRATED,
        "",                                        // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzGyro),     // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMM150 magnetometer",
        "Bosch",
        kVersion,
        COMMS_SENSOR_MAG,
        SENSOR_TYPE_MAGNETIC_FIELD,
        1300.0f,                                   // XXX maxRange
        0.0f,                                      // XXX resolution
        0.0f,                                      // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzMag),   // minDelay
        0,                                         // XXX fifoReservedEventCount
        0,                                         // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_MAGNETIC_FIELD,
        "",                                        // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzMag),      // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMM150 magnetometer (uncalibrated)",
        "Bosch",
        kVersion,
        COMMS_SENSOR_MAG_UNCALIBRATED,
        SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
        1300.0f,                                        // XXX maxRange
        0.0f,                                           // XXX resolution
        0.0f,                                           // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzMag),        // minDelay
        600,                                            // XXX fifoReservedEventCount
        kMaxThreeAxisEventCount,                        // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
        "",                                             // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzMag),           // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMP280 pressure",
        "Bosch",
        kVersion,
        COMMS_SENSOR_PRESSURE,
        SENSOR_TYPE_PRESSURE,
        1100.0f,                                      // maxRange (hPa)
        0.005f,                                       // resolution (hPa)
        0.0f,                                         // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzPressure), // minDelay
        300,                                          // XXX fifoReservedEventCount
        kMaxOneAxisEventCount,                        // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_PRESSURE,
        "",                                           // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzPressure),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMP280 temperature",
        "Bosch",
        kVersion,
        COMMS_SENSOR_TEMPERATURE,
        SENSOR_TYPE_INTERNAL_TEMPERATURE,
        85.0f,                                           // maxRange (degC)
        0.01,                                            // resolution (degC)
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzTemperature), // minDelay
        0,                                               // XXX fifoReservedEventCount
        0,                                               // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_INTERNAL_TEMPERATURE,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzTemperature),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "Orientation",
        "Google",
        kVersion,
        COMMS_SENSOR_ORIENTATION,
        SENSOR_TYPE_ORIENTATION,
        360.0f,                                          // maxRange (deg)
        1.0f,                                            // XXX resolution (deg)
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzOrientation), // minDelay
        0,                                               // XXX fifoReservedEventCount
        0,                                               // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_ORIENTATION,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzOrientation),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "BMI160 Step detector",
        "Bosch",
        kVersion,
        COMMS_SENSOR_STEP_DETECTOR,
        SENSOR_TYPE_STEP_DETECTOR,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.0f,                                   // XXX power
        0,                                      // minDelay
        100,                                    // XXX fifoReservedEventCount
        kMaxOneAxisEventCount,                  // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_STEP_DETECTOR,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_SPECIAL_REPORTING_MODE,
        { NULL, NULL }
    },
    {
        "BMI160 Step counter",
        "Bosch",
        kVersion,
        COMMS_SENSOR_STEP_COUNTER,
        SENSOR_TYPE_STEP_COUNTER,
        1.0f,                                   // XXX maxRange
        1.0f,                                   // resolution
        0.0f,                                   // XXX power
        0,                                      // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_STEP_COUNTER,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_ON_CHANGE_MODE,
        { NULL, NULL }
    },
    {
        "Significant motion",
        "Google",
        kVersion,
        COMMS_SENSOR_SIGNIFICANT_MOTION,
        SENSOR_TYPE_SIGNIFICANT_MOTION,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.0f,                                   // XXX power
        -1,                                     // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_SIGNIFICANT_MOTION,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_ONE_SHOT_MODE,
        { NULL, NULL }
    },
    {
        "Gravity",
        "Google",
        kVersion,
        COMMS_SENSOR_GRAVITY,
        SENSOR_TYPE_GRAVITY,
        1000.0f,                                         // maxRange
        1.0f,                                            // XXX resolution
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzOrientation), // minDelay
        0,                                               // XXX fifoReservedEventCount
        0,                                               // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_GRAVITY,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzOrientation),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "Linear Acceleration",
        "Google",
        kVersion,
        COMMS_SENSOR_LINEAR_ACCEL,
        SENSOR_TYPE_LINEAR_ACCELERATION,
        1000.0f,                                         // maxRange
        1.0f,                                            // XXX resolution
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzOrientation), // minDelay
        0,                                               // XXX fifoReservedEventCount
        0,                                               // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_LINEAR_ACCELERATION,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzOrientation),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "Rotation Vector",
        "Google",
        kVersion,
        COMMS_SENSOR_ROTATION_VECTOR,
        SENSOR_TYPE_ROTATION_VECTOR,
        1000.0f,                                         // maxRange
        1.0f,                                            // XXX resolution
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzOrientation), // minDelay
        0,                                               // XXX fifoReservedEventCount
        0,                                               // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_ROTATION_VECTOR,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzOrientation),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "Geomagnetic Rotation Vector",
        "Google",
        kVersion,
        COMMS_SENSOR_GEO_MAG,
        SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR,
        1000.0f,                                         // maxRange
        1.0f,                                            // XXX resolution
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzOrientation), // minDelay
        0,                                               // XXX fifoReservedEventCount
        0,                                               // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_GEOMAGNETIC_ROTATION_VECTOR,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzOrientation),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "Game Rotation Vector",
        "Google",
        kVersion,
        COMMS_SENSOR_GAME_ROTATION_VECTOR,
        SENSOR_TYPE_GAME_ROTATION_VECTOR,
        1000.0f,                                         // maxRange
        1.0f,                                            // XXX resolution
        0.0f,                                            // XXX power
        (int32_t)(1.0E6f / kMaxSampleRateHzOrientation), // minDelay
        300,                                             // XXX fifoReservedEventCount
        kMaxThreeAxisEventCount,                         // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_GAME_ROTATION_VECTOR,
        "",                                              // requiredPermission
        (long)(1.0E6f / kMinSampleRateHzOrientation),    // maxDelay
        SENSOR_FLAG_CONTINUOUS_MODE,
        { NULL, NULL }
    },
    {
        "Tilt Detector",
        "Google",
        kVersion,
        COMMS_SENSOR_TILT,
        SENSOR_TYPE_TILT_DETECTOR,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.0f,                                   // XXX power
        0,                                      // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_TILT_DETECTOR,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_SPECIAL_REPORTING_MODE,
        { NULL, NULL }
    },
    {
        "Pickup Gesture",
        "Google",
        kVersion,
        COMMS_SENSOR_GESTURE,
        SENSOR_TYPE_PICK_UP_GESTURE,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.0f,                                   // XXX power
        -1,                                     // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_PICK_UP_GESTURE,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_ONE_SHOT_MODE,
        { NULL, NULL }
    },
    {
        "Sensors Sync",
        "Google",
        kVersion,
        COMMS_SENSOR_SYNC,
        SENSOR_TYPE_SYNC,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.1f,                                   // XXX power
        0,                                      // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_SYNC,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_SPECIAL_REPORTING_MODE,
        { NULL, NULL }
    },
    {
        "Double Twist",
        "Google",
        kVersion,
        COMMS_SENSOR_DOUBLE_TWIST,
        SENSOR_TYPE_DOUBLE_TWIST,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.1f,                                   // XXX power
        0,                                      // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_DOUBLE_TWIST,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_WAKE_UP | SENSOR_FLAG_SPECIAL_REPORTING_MODE,
        { NULL, NULL }
    },
    {
        "Double Tap",
        "Google",
        kVersion,
        COMMS_SENSOR_DOUBLE_TAP,
        SENSOR_TYPE_DOUBLE_TAP,
        1.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.1f,                                   // XXX power
        0,                                      // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_DOUBLE_TAP,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_SPECIAL_REPORTING_MODE,
        { NULL, NULL }
    },
    {
        "Device Orientation",
        "Google",
        kVersion,
        COMMS_SENSOR_WINDOW_ORIENTATION,
        SENSOR_TYPE_DEVICE_ORIENTATION,
        3.0f,                                   // maxRange
        1.0f,                                   // XXX resolution
        0.1f,                                   // XXX power
        0,                                      // minDelay
        0,                                      // XXX fifoReservedEventCount
        0,                                      // XXX fifoMaxEventCount
        SENSOR_STRING_TYPE_DEVICE_ORIENTATION,
        "",                                     // requiredPermission
        0,                                      // maxDelay
        SENSOR_FLAG_ON_CHANGE_MODE,
        { NULL, NULL }
    },
};

////////////////////////////////////////////////////////////////////////////////

SensorContext::SensorContext(const struct hw_module_t *module)
    : mHubConnection(HubConnection::getInstance()), mHubAlive(true) {
    memset(&device, 0, sizeof(device));

    device.common.tag = HARDWARE_DEVICE_TAG;
    device.common.version = SENSORS_DEVICE_API_VERSION_1_3;
    device.common.module = const_cast<hw_module_t *>(module);
    device.common.close = CloseWrapper;
    device.activate = ActivateWrapper;
    device.setDelay = SetDelayWrapper;
    device.poll = PollWrapper;
    device.batch = BatchWrapper;
    device.flush = FlushWrapper;

    if (mHubConnection->initCheck() != (status_t)OK) {
        mHubAlive = false;
    } else {
        if (mHubConnection->getAliveCheck() != (status_t)OK) {
            mHubAlive = false;
        } else {
            HubConnection::ProximitySensorType type;
            mHubConnection->getProximitySensorType(&type);

            if (type == HubConnection::PROXIMITY_ROHM) {
                kSensorList[0].name = "RPR0521 proximity";
                kSensorList[0].vendor = "Rohm";
                kSensorList[1].name = "RPR0521 light";
                kSensorList[1].vendor = "Rohm";
            } else if (type == HubConnection::PROXIMITY_AMS) {
                kSensorList[0].name = "TMD27723 proximity";
                kSensorList[0].vendor = "AMS";
                kSensorList[1].name = "TMD27723 light";
                kSensorList[1].vendor = "AMS";
                kSensorList[1].maxRange = 10000;
            }
        }
    }
}

SensorContext::~SensorContext() {
}

int SensorContext::close() {
    ALOGI("close");

    delete this;

    return 0;
}

int SensorContext::activate(int handle, int enabled) {
    ALOGI("activate");

    mHubConnection->queueActivate(handle, enabled);

    return 0;
}

int SensorContext::setDelay(int handle, int64_t delayNs) {
    ALOGI("setDelay");

    // clamp sample rate based on minDelay and maxDelay defined in kSensorList
    int64_t delayNsClamped = delayNs;
    int num_sensors = sizeof(kSensorList) / sizeof(sensor_t);
    for (int i = 0; i < num_sensors; i++) {
        sensor_t sensor = kSensorList[i];
        if (sensor.handle != handle) {
            continue;
        }

        if ((sensor.flags & 0xE) == SENSOR_FLAG_CONTINUOUS_MODE) {
            if ((delayNs/1000) < sensor.minDelay) {
                delayNsClamped = sensor.minDelay * 1000;
            } else if ((delayNs/1000) > sensor.maxDelay) {
                delayNsClamped = sensor.maxDelay * 1000;
            }
        }

        break;
    }

    mHubConnection->queueSetDelay(handle, delayNsClamped);

    return 0;
}

int SensorContext::poll(sensors_event_t *data, int count) {
    ALOGV("poll");

    ssize_t n = mHubConnection->read(data, count);

    if (n < 0) {
        return -1;
    }

    return n;
}

int SensorContext::batch(
        int handle,
        int flags,
        int64_t sampling_period_ns,
        int64_t max_report_latency_ns) {
    ALOGI("batch");

    // clamp sample rate based on minDelay and maxDelay defined in kSensorList
    int64_t sampling_period_ns_clamped = sampling_period_ns;
    int num_sensors = sizeof(kSensorList) / sizeof(sensor_t);
    for (int i = 0; i < num_sensors; i++) {
        sensor_t sensor = kSensorList[i];
        if (sensor.handle != handle) {
            continue;
        }

        if ((sensor.flags & 0xE) == SENSOR_FLAG_CONTINUOUS_MODE) {
            if ((sampling_period_ns/1000) < sensor.minDelay) {
                sampling_period_ns_clamped = sensor.minDelay * 1000;
            } else if ((sampling_period_ns/1000) > sensor.maxDelay) {
                sampling_period_ns_clamped = sensor.maxDelay * 1000;
            }
        }

        break;
    }

    mHubConnection->queueBatch(
            handle, flags, sampling_period_ns_clamped, max_report_latency_ns);

    return 0;
}

int SensorContext::flush(int handle) {
    ALOGI("flush");

    mHubConnection->queueFlush(handle);
    return 0;
}

// static
int SensorContext::CloseWrapper(struct hw_device_t *dev) {
    return reinterpret_cast<SensorContext *>(dev)->close();
}

// static
int SensorContext::ActivateWrapper(
        struct sensors_poll_device_t *dev, int handle, int enabled) {
    return reinterpret_cast<SensorContext *>(dev)->activate(handle, enabled);
}

// static
int SensorContext::SetDelayWrapper(
        struct sensors_poll_device_t *dev, int handle, int64_t delayNs) {
    return reinterpret_cast<SensorContext *>(dev)->setDelay(handle, delayNs);
}

// static
int SensorContext::PollWrapper(
        struct sensors_poll_device_t *dev, sensors_event_t *data, int count) {
    return reinterpret_cast<SensorContext *>(dev)->poll(data, count);
}

// static
int SensorContext::BatchWrapper(
        struct sensors_poll_device_1 *dev,
        int handle,
        int flags,
        int64_t sampling_period_ns,
        int64_t max_report_latency_ns) {
    return reinterpret_cast<SensorContext *>(dev)->batch(
            handle, flags, sampling_period_ns, max_report_latency_ns);
}

// static
int SensorContext::FlushWrapper(struct sensors_poll_device_1 *dev, int handle) {
    return reinterpret_cast<SensorContext *>(dev)->flush(handle);
}

bool SensorContext::getHubAlive() {
    return mHubAlive;
}

////////////////////////////////////////////////////////////////////////////////

static bool gHubAlive;

static int open_sensors(
        const struct hw_module_t *module,
        const char *,
        struct hw_device_t **dev) {
    ALOGI("open_sensors");

    SensorContext *ctx = new SensorContext(module);

    gHubAlive = ctx->getHubAlive();
    *dev = &ctx->device.common;

    return 0;
}

static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

static int get_sensors_list(
        struct sensors_module_t *,
        struct sensor_t const **list) {
    ALOGI("get_sensors_list");

    if (gHubAlive) {
        *list = kSensorList;
        return sizeof(kSensorList) / sizeof(kSensorList[0]);
    } else {
        *list = {};
        return 0;
    }
}

static int set_operation_mode(unsigned int mode) {
    ALOGI("set_operation_mode");
    return (mode) ? -EINVAL : 0;
}

struct sensors_module_t HAL_MODULE_INFO_SYM = {
        .common = {
                .tag = HARDWARE_MODULE_TAG,
                .version_major = 1,
                .version_minor = 0,
                .id = SENSORS_HARDWARE_MODULE_ID,
                .name = "Google Sensor module",
                .author = "Google",
                .methods = &sensors_module_methods,
                .dso  = NULL,
                .reserved = {0},
        },
        .get_sensors_list = get_sensors_list,
        .set_operation_mode = set_operation_mode,
};
