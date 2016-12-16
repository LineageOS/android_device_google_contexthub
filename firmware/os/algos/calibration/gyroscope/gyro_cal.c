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

#include "calibration/gyroscope/gyro_cal.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "calibration/util/cal_log.h"
#include "common/math/vec.h"

/////// DEFINITIONS AND MACROS ///////////////////////////////////////

// Maximum gyro bias correction (should be set based on expected max bias
// of the given sensor).
#define MAX_GYRO_BIAS (0.096f)  // [rad/sec]

// The time value used to throttle debug messaging.
#define OVERTEMPCAL_WAIT_TIME_NANOS (250000000)

// Converts units of radians to milli-degrees.
#define RAD_TO_MILLI_DEGREES (float)(1e3f * 180.0f / M_PI)

// Unit conversion: m/sec^2 to g's.
#define GRAVITY_TO_G (float)(1e3f * 180.0f / M_PI)

// Unit conversion: nanoseconds to seconds.
#define NANOS_TO_SEC (1.0e-9f)

/////// FORWARD DECLARATIONS /////////////////////////////////////////

static void deviceStillnessCheck(struct GyroCal* gyro_cal,
                                 uint64_t sample_time_nanos);

static void computeGyroCal(struct GyroCal* gyro_cal,
                           uint64_t calibration_time_nanos);

static void checkWatchdog(struct GyroCal* gyro_cal, uint64_t sample_time_nanos);

#ifdef GYRO_CAL_DBG_ENABLED
static void gyroCalUpdateDebug(struct GyroCal* gyro_cal);

/*
 * Updates running calculation of the temperature statistics.
 *
 * Behavior:
 *   1)  If 'debug_temperature' pointer is not NULL then the local calculation
 *       of the temperature statistics are copied, and the function returns.
 *   2)  Else, if 'reset_stats' is 'true' then the local statistics are reset
 *       and the function returns.
 *   3)  Otherwise, the local temperature statistics are updated according to
 *       the input value 'temperature'.
 *
 * INPUTS:
 *   debug_temperature:   Pointer to the temperature stats sturcture to update.
 *   temperature:  Temperature value (Celsius).
 *   reset_stats:  Flag that determines if the local running stats are reset.
 */
static void gyroTempUpdateStats(struct DebugTemperature* debug_temperature,
                                float temperature, bool reset_stats);

/*
 * Updates running calculation of the gyro's mean sampling rate.
 *
 * Behavior:
 *   1)  If 'debug_mean_sampling_rate_hz' pointer is not NULL then the local
 *       calculation of the sampling rate is copied, and the function returns.
 *   2)  Else, if 'reset_stats' is 'true' then the local estimate is reset and
 *       the function returns.
 *   3)  Otherwise, the local estimate of the mean sampling rates is updated.
 *
 * INPUTS:
 *   debug_mean_sampling_rate_hz:   Pointer to the mean sampling rate to update.
 *   timestamp_nanos:  Time stamp (nanoseconds).
 *   reset_stats:  Flag that signals a reset of the sampling rate estimate.
 */
static void gyroSamplingRateUpdate(float* debug_mean_sampling_rate_hz,
                                   uint64_t timestamp_nanos, bool reset_stats);

// Defines the type of debug data to print.
enum DebugPrintData {
  BIAS_CAL = 0,
  CAL_TIME,
  ACCEL_STATS,
  GYRO_STATS,
  MAG_STATS,
  TEMP_STATS,
  STILLNESS_DATA,
  SAMPLING_RATE
};

// Helper function for printing out common debug data.
static void gyroCalDebugPrintData(const struct DebugGyroCal* debug_data,
                                  enum DebugPrintData print_data);

// This conversion function is necessary for Nanohub firmware compilation (i.e.,
// can't cast a uint64_t to a float directly). This conversion function was
// copied from: /third_party/contexthub/firmware/src/floatRt.c
static float floatFromUint64(uint64_t v)
{
    uint32_t hi = v >> 32, lo = v;

    if (!hi) //this is very fast for cases where we fit into a uint32_t
        return(float)lo;
    else {
        return ((float)hi) * 4294967296.0f + (float)lo;
    }
}

#ifdef GYRO_CAL_DBG_TUNE_ENABLED
// Prints debug information useful for tuning the GyroCal parameters.
static void gyroCalTuneDebugPrint(const struct GyroCal* gyro_cal,
                                  uint64_t timestamp_nanos);
#endif  // GYRO_CAL_DBG_TUNE_ENABLED
#endif  // GYRO_CAL_DBG_ENABLED

/////// FUNCTION DEFINITIONS /////////////////////////////////////////

// Initialize the gyro calibration data structure.
void gyroCalInit(struct GyroCal* gyro_cal, uint64_t min_still_duration_nanos,
                 uint64_t max_still_duration_nanos, float bias_x, float bias_y,
                 float bias_z, uint64_t calibration_time_nanos,
                 uint64_t window_time_duration_nanos, float gyro_var_threshold,
                 float gyro_confidence_delta, float accel_var_threshold,
                 float accel_confidence_delta, float mag_var_threshold,
                 float mag_confidence_delta, float stillness_threshold,
                 int remove_bias_enable) {
  // Clear gyro_cal structure memory.
  memset(gyro_cal, 0, sizeof(struct GyroCal));

  // Initialize the stillness detectors.
  // Gyro parameter input units are [rad/sec].
  // Accel parameter input units are [m/sec^2].
  // Magnetometer parameter input units are [uT].
  gyroStillDetInit(&gyro_cal->gyro_stillness_detect, gyro_var_threshold,
                   gyro_confidence_delta);
  gyroStillDetInit(&gyro_cal->accel_stillness_detect, accel_var_threshold,
                   accel_confidence_delta);
  gyroStillDetInit(&gyro_cal->mag_stillness_detect, mag_var_threshold,
                   mag_confidence_delta);

  // Reset stillness flag and start timestamp.
  gyro_cal->prev_still = false;
  gyro_cal->start_still_time_nanos = 0;

  // Set the min and max window stillness duration.
  gyro_cal->min_still_duration_nanos = min_still_duration_nanos;
  gyro_cal->max_still_duration_nanos = max_still_duration_nanos;

  // Sets the duration of the stillness processing windows.
  gyro_cal->window_time_duration_nanos = window_time_duration_nanos;

  // Set the watchdog timeout duration.
  gyro_cal->gyro_watchdog_timeout_duration_nanos =
      2 * window_time_duration_nanos;

  // Load the last valid cal from system memory.
  gyro_cal->bias_x = bias_x;  // [rad/sec]
  gyro_cal->bias_y = bias_y;  // [rad/sec]
  gyro_cal->bias_z = bias_z;  // [rad/sec]
  gyro_cal->calibration_time_nanos = calibration_time_nanos;

  // Set the stillness threshold required for gyro bias calibration.
  gyro_cal->stillness_threshold = stillness_threshold;

  // Current window end time used to assist in keeping sensor data
  // collection in sync. Setting this to zero signals that sensor data
  // will be dropped until a valid end time is set from the first gyro
  // timestamp received.
  gyro_cal->stillness_win_endtime_nanos = 0;

  // Gyro calibrations will be applied (see, gyroCalRemoveBias()).
  gyro_cal->gyro_calibration_enable = (remove_bias_enable > 0);

#ifdef GYRO_CAL_DBG_ENABLED
  CAL_DEBUG_LOG("[GYRO_CAL:MEMORY]", "sizeof(struct GyroCal): %lu",
                (unsigned long int)sizeof(struct GyroCal));

  CAL_DEBUG_LOG("[GYRO_CAL:INIT]",
                "Gyro Bias Calibration [mdps]: %s%d.%06d, %s%d.%06d, %s%d.%06d",
                CAL_ENCODE_FLOAT(gyro_cal->bias_x * RAD_TO_MILLI_DEGREES, 6),
                CAL_ENCODE_FLOAT(gyro_cal->bias_y * RAD_TO_MILLI_DEGREES, 6),
                CAL_ENCODE_FLOAT(gyro_cal->bias_z * RAD_TO_MILLI_DEGREES, 6));

  if (gyro_cal->gyro_calibration_enable) {
    CAL_DEBUG_LOG("[GYRO_CAL:INIT]", "Online gyroscope calibration ENABLED.");
  } else {
    CAL_DEBUG_LOG("[GYRO_CAL:INIT]", "Online gyroscope calibration DISABLED.");
  }

  // Ensures that the running temperature statistics and gyro sampling rate
  // estimate are reset.
  gyroTempUpdateStats(NULL, 0, /*reset_stats=*/true);
  gyroSamplingRateUpdate(NULL, 0, /*reset_stats=*/true);
#endif  // GYRO_CAL_DBG_ENABLED
}

// Void all pointers in the gyro calibration data structure (doesn't do anything
// except prevent compiler warnings).
void gyroCalDestroy(struct GyroCal* gyro_cal) { (void)gyro_cal; }

// Get the most recent bias calibration value.
void gyroCalGetBias(struct GyroCal* gyro_cal, float* bias_x, float* bias_y,
                    float* bias_z) {
  if (gyro_cal->gyro_calibration_enable) {
    *bias_x = gyro_cal->bias_x;
    *bias_y = gyro_cal->bias_y;
    *bias_z = gyro_cal->bias_z;

#ifdef GYRO_CAL_DBG_ENABLED
    CAL_DEBUG_LOG(
        "[GYRO_CAL:STORED]",
        "Gyro Bias Calibration [mdps]: %s%d.%06d, %s%d.%06d, %s%d.%06d",
        CAL_ENCODE_FLOAT(gyro_cal->bias_x * RAD_TO_MILLI_DEGREES, 6),
        CAL_ENCODE_FLOAT(gyro_cal->bias_y * RAD_TO_MILLI_DEGREES, 6),
        CAL_ENCODE_FLOAT(gyro_cal->bias_z * RAD_TO_MILLI_DEGREES, 6));
#endif
  }
}

// Set an initial bias calibration value.
void gyroCalSetBias(struct GyroCal* gyro_cal, float bias_x, float bias_y,
                    float bias_z, uint64_t calibration_time_nanos) {
  gyro_cal->bias_x = bias_x;
  gyro_cal->bias_y = bias_y;
  gyro_cal->bias_z = bias_z;
  gyro_cal->calibration_time_nanos = calibration_time_nanos;

#ifdef GYRO_CAL_DBG_ENABLED
  CAL_DEBUG_LOG("[GYRO_CAL:RECALL]",
                "Gyro Bias Calibration [mdps]: %s%d.%06d, %s%d.%06d, %s%d.%06d",
                CAL_ENCODE_FLOAT(gyro_cal->bias_x * RAD_TO_MILLI_DEGREES, 6),
                CAL_ENCODE_FLOAT(gyro_cal->bias_y * RAD_TO_MILLI_DEGREES, 6),
                CAL_ENCODE_FLOAT(gyro_cal->bias_z * RAD_TO_MILLI_DEGREES, 6));
#endif
}

// Remove bias from a gyro measurement [rad/sec].
void gyroCalRemoveBias(struct GyroCal* gyro_cal, float xi, float yi, float zi,
                       float* xo, float* yo, float* zo) {
  if (gyro_cal->gyro_calibration_enable) {
    *xo = xi - gyro_cal->bias_x;
    *yo = yi - gyro_cal->bias_y;
    *zo = zi - gyro_cal->bias_z;
  }
}

// Returns true when a new gyro calibration is available.
bool gyroCalNewBiasAvailable(struct GyroCal* gyro_cal) {
  bool new_gyro_cal_available =
      (gyro_cal->gyro_calibration_enable && gyro_cal->new_gyro_cal_available);

  // Clear the flag.
  gyro_cal->new_gyro_cal_available = false;

  return new_gyro_cal_available;
}

// Update the gyro calibration with gyro data [rad/sec].
void gyroCalUpdateGyro(struct GyroCal* gyro_cal, uint64_t sample_time_nanos,
                       float x, float y, float z, float temperature) {
  // Make sure that a valid window end time is set,
  // and start the watchdog timer.
  if (gyro_cal->stillness_win_endtime_nanos <= 0) {
    gyro_cal->stillness_win_endtime_nanos =
        sample_time_nanos + gyro_cal->window_time_duration_nanos;

    // Start the watchdog timer.
    gyro_cal->gyro_watchdog_start_nanos = sample_time_nanos;
  }

#ifdef GYRO_CAL_DBG_ENABLED
  // Update the temperature statistics (on temperature change only).
  if (NANO_ABS(gyro_cal->latest_temperature_celcius - temperature) > FLT_MIN) {
    gyroTempUpdateStats(NULL, temperature, /*reset_stats=*/false);
  }

  // Update the gyro sampling rate estimate.
  gyroSamplingRateUpdate(NULL, sample_time_nanos, /*reset_stats=*/false);
#endif  // GYRO_CAL_DBG_ENABLED

  // Record the latest temperture sample.
  gyro_cal->latest_temperature_celcius = temperature;

  // Pass gyro data to stillness detector
  gyroStillDetUpdate(&gyro_cal->gyro_stillness_detect,
                     gyro_cal->stillness_win_endtime_nanos, sample_time_nanos,
                     x, y, z);

  // Perform a device stillness check, set next window end time, and
  // possibly do a gyro bias calibration and stillness detector reset.
  deviceStillnessCheck(gyro_cal, sample_time_nanos);
}

// Update the gyro calibration with mag data [micro Tesla].
void gyroCalUpdateMag(struct GyroCal* gyro_cal, uint64_t sample_time_nanos,
                      float x, float y, float z) {
  // Pass magnetometer data to stillness detector.
  gyroStillDetUpdate(&gyro_cal->mag_stillness_detect,
                     gyro_cal->stillness_win_endtime_nanos, sample_time_nanos,
                     x, y, z);

  // Received a magnetometer sample; incorporate it into detection.
  gyro_cal->using_mag_sensor = true;

  // Perform a device stillness check, set next window end time, and
  // possibly do a gyro bias calibration and stillness detector reset.
  deviceStillnessCheck(gyro_cal, sample_time_nanos);
}

// Update the gyro calibration with accel data [m/sec^2].
void gyroCalUpdateAccel(struct GyroCal* gyro_cal, uint64_t sample_time_nanos,
                        float x, float y, float z) {
  // Pass accelerometer data to stillnesss detector.
  gyroStillDetUpdate(&gyro_cal->accel_stillness_detect,
                     gyro_cal->stillness_win_endtime_nanos, sample_time_nanos,
                     x, y, z);

  // Perform a device stillness check, set next window end time, and
  // possibly do a gyro bias calibration and stillness detector reset.
  deviceStillnessCheck(gyro_cal, sample_time_nanos);
}

// Checks the state of all stillness detectors to determine
// whether the device is "still".
void deviceStillnessCheck(struct GyroCal* gyro_cal,
                          uint64_t sample_time_nanos) {
  bool stillness_duration_exceeded = false;
  bool stillness_duration_too_short = false;
  bool device_is_still = false;
  float conf_not_rot = 0;
  float conf_not_accel = 0;
  float conf_still = 0;

  // Check the watchdog timer.
  checkWatchdog(gyro_cal, sample_time_nanos);

  // Is there enough data to do a stillness calculation?
  if ((!gyro_cal->mag_stillness_detect.stillness_window_ready &&
       gyro_cal->using_mag_sensor) ||
      !gyro_cal->accel_stillness_detect.stillness_window_ready ||
      !gyro_cal->gyro_stillness_detect.stillness_window_ready) {
    return;  // Not yet, wait for more data.
  }

  // Set the next window end time for the stillness detectors.
  gyro_cal->stillness_win_endtime_nanos =
      sample_time_nanos + gyro_cal->window_time_duration_nanos;

  // Update the confidence scores for all sensors.
  gyroStillDetCompute(&gyro_cal->accel_stillness_detect);
  gyroStillDetCompute(&gyro_cal->gyro_stillness_detect);
  if (gyro_cal->using_mag_sensor) {
    gyroStillDetCompute(&gyro_cal->mag_stillness_detect);
  } else {
    // Not using magnetometer, force stillness confidence to 100%.
    gyro_cal->mag_stillness_detect.stillness_confidence = 1.0f;
  }

  // Determine motion confidence scores (rotation, accelerating, and stillness).
  conf_not_rot = gyro_cal->gyro_stillness_detect.stillness_confidence *
                 gyro_cal->mag_stillness_detect.stillness_confidence;
  conf_not_accel = gyro_cal->accel_stillness_detect.stillness_confidence;
  conf_still = conf_not_rot * conf_not_accel;

  // determine if the device is currently still.
  device_is_still = (conf_still > gyro_cal->stillness_threshold);

  if (device_is_still) {
    // Device is still logic:
    // If not previously still, then record the start time.
    // If stillness period is too long, then do a calibration.
    // Otherwise, continue collecting stillness data.

    // If device was not previously still, set new start timestamp.
    if (!gyro_cal->prev_still) {
      // Record the starting timestamp of the current stillness window.
      // This enables the calculation of total duration of the stillness period.
      gyro_cal->start_still_time_nanos =
          gyro_cal->gyro_stillness_detect.window_start_time;
    }

    // Check to see if current stillness period exceeds the desired limit.
    stillness_duration_exceeded =
        ((gyro_cal->gyro_stillness_detect.last_sample_time -
          gyro_cal->start_still_time_nanos) >
         gyro_cal->max_still_duration_nanos);

    if (stillness_duration_exceeded) {
      // The current stillness has gone too long. Do a calibration with the
      // current data and reset.

      // Update the gyro bias estimate with the current window data and
      // reset the stats.
      gyroStillDetReset(&gyro_cal->accel_stillness_detect,
                        /*reset_stats=*/true);
      gyroStillDetReset(&gyro_cal->gyro_stillness_detect, /*reset_stats=*/true);
      gyroStillDetReset(&gyro_cal->mag_stillness_detect, /*reset_stats=*/true);

      // Perform calibration.
      computeGyroCal(gyro_cal,
                     gyro_cal->gyro_stillness_detect.last_sample_time);

#ifdef GYRO_CAL_DBG_ENABLED
      // Reset the temperature statistics and sampling rate estimate.
      gyroTempUpdateStats(NULL, 0, /*reset_stats=*/true);
      gyroSamplingRateUpdate(NULL, sample_time_nanos, /*reset_stats=*/true);
#endif  // GYRO_CAL_DBG_ENABLED

      // Update stillness flag. Force the start of a new stillness period.
      gyro_cal->prev_still = false;
    } else {
      // Continue collecting stillness data.

      // Reset stillness detectors, and extend stillness period.
      gyroStillDetReset(&gyro_cal->accel_stillness_detect,
                        /*reset_stats=*/false);
      gyroStillDetReset(&gyro_cal->gyro_stillness_detect,
                        /*reset_stats=*/false);
      gyroStillDetReset(&gyro_cal->mag_stillness_detect, /*reset_stats=*/false);

      // Update stillness flag.
      gyro_cal->prev_still = true;
    }
  } else {
    // Device is NOT still; motion detected.

    // If device was previously still and the total stillness duration is not
    // "too short", then do a calibration with the data accumulated thus far.
    stillness_duration_too_short =
        ((gyro_cal->gyro_stillness_detect.window_start_time -
          gyro_cal->start_still_time_nanos) <
         gyro_cal->min_still_duration_nanos);

    if (gyro_cal->prev_still && !stillness_duration_too_short) {
      computeGyroCal(gyro_cal,
                     gyro_cal->gyro_stillness_detect.window_start_time);
    }

    // Reset stillness detectors and the stats.
    gyroStillDetReset(&gyro_cal->accel_stillness_detect, /*reset_stats=*/true);
    gyroStillDetReset(&gyro_cal->gyro_stillness_detect, /*reset_stats=*/true);
    gyroStillDetReset(&gyro_cal->mag_stillness_detect, /*reset_stats=*/true);

#ifdef GYRO_CAL_DBG_ENABLED
    // Reset the temperature statistics and sampling rate estimate.
    gyroTempUpdateStats(NULL, 0, /*reset_stats=*/true);
    gyroSamplingRateUpdate(NULL, sample_time_nanos, /*reset_stats=*/true);
#endif  // GYRO_CAL_DBG_ENABLED

    // Update stillness flag.
    gyro_cal->prev_still = false;
  }

  // Reset the watchdog timer after we have processed data.
  gyro_cal->gyro_watchdog_start_nanos = sample_time_nanos;
}

// Calculates a new gyro bias offset calibration value.
void computeGyroCal(struct GyroCal* gyro_cal, uint64_t calibration_time_nanos) {
  // Current calibration duration.
  uint64_t cur_cal_dur_nanos =
      calibration_time_nanos - gyro_cal->start_still_time_nanos;

  // Check to see if new calibration values is within acceptable range.
  if (!(gyro_cal->gyro_stillness_detect.prev_mean_x < MAX_GYRO_BIAS &&
        gyro_cal->gyro_stillness_detect.prev_mean_x > -MAX_GYRO_BIAS &&
        gyro_cal->gyro_stillness_detect.prev_mean_y < MAX_GYRO_BIAS &&
        gyro_cal->gyro_stillness_detect.prev_mean_y > -MAX_GYRO_BIAS &&
        gyro_cal->gyro_stillness_detect.prev_mean_z < MAX_GYRO_BIAS &&
        gyro_cal->gyro_stillness_detect.prev_mean_z > -MAX_GYRO_BIAS)) {
#ifdef GYRO_CAL_DBG_ENABLED
    CAL_DEBUG_LOG(
        "[GYRO_CAL:WARNING]",
        "Rejected Bias Update [mdps]: %s%d.%06d, %s%d.%06d, %s%d.%06d",
        CAL_ENCODE_FLOAT(gyro_cal->bias_x * RAD_TO_MILLI_DEGREES, 6),
        CAL_ENCODE_FLOAT(gyro_cal->bias_y * RAD_TO_MILLI_DEGREES, 6),
        CAL_ENCODE_FLOAT(gyro_cal->bias_z * RAD_TO_MILLI_DEGREES, 6));
#endif  // GYRO_CAL_DBG_ENABLED

    // Outside of range. Ignore, reset, and continue.
    return;
  }

  // Record new gyro bias offset calibration.
  gyro_cal->bias_x = gyro_cal->gyro_stillness_detect.prev_mean_x;
  gyro_cal->bias_y = gyro_cal->gyro_stillness_detect.prev_mean_y;
  gyro_cal->bias_z = gyro_cal->gyro_stillness_detect.prev_mean_z;

  // Record final stillness confidence.
  gyro_cal->stillness_confidence =
      gyro_cal->gyro_stillness_detect.prev_stillness_confidence *
      gyro_cal->accel_stillness_detect.prev_stillness_confidence *
      gyro_cal->mag_stillness_detect.prev_stillness_confidence;

  // Store calibration stillness duration.
  gyro_cal->calibration_time_duration_nanos = cur_cal_dur_nanos;

  // Store calibration time stamp.
  gyro_cal->calibration_time_nanos = calibration_time_nanos;

  // Set flag to indicate a new gyro calibration value is available.
  gyro_cal->new_gyro_cal_available = true;

#ifdef GYRO_CAL_DBG_ENABLED
  // Increment the total count of calibration updates.
  gyro_cal->debug_calibration_count++;

  // Update the calibration debug information.
  gyroCalUpdateDebug(gyro_cal);

  // Trigger a printout of the debug information.
  gyro_cal->debug_print_trigger = true;
#endif
}

// Check for a watchdog timeout condition.
void checkWatchdog(struct GyroCal* gyro_cal, uint64_t sample_time_nanos) {
  bool watchdog_timeout;

  // Check for initialization of the watchdog time (=0).
  if (gyro_cal->gyro_watchdog_start_nanos <= 0) {
    return;
  }

  // Check for timeout condition of watchdog.
  watchdog_timeout =
      ((sample_time_nanos - gyro_cal->gyro_watchdog_start_nanos) >
       gyro_cal->gyro_watchdog_timeout_duration_nanos);

  // If a timeout occurred then reset to known good state.
  if (watchdog_timeout) {
    // Reset stillness detectors and restart data capture.
    gyroStillDetReset(&gyro_cal->accel_stillness_detect, /*reset_stats=*/true);
    gyroStillDetReset(&gyro_cal->gyro_stillness_detect, /*reset_stats=*/true);
    gyroStillDetReset(&gyro_cal->mag_stillness_detect, /*reset_stats=*/true);
    gyro_cal->mag_stillness_detect.stillness_confidence = 0;
    gyro_cal->stillness_win_endtime_nanos = 0;

#ifdef GYRO_CAL_DBG_ENABLED
    // Reset the temperature statistics and sampling rate estimate.
    gyroTempUpdateStats(NULL, 0, /*reset_stats=*/true);
    gyroSamplingRateUpdate(NULL, sample_time_nanos, /*reset_stats=*/true);
#endif  // GYRO_CAL_DBG_ENABLED

    // Force stillness confidence to zero.
    gyro_cal->accel_stillness_detect.prev_stillness_confidence = 0;
    gyro_cal->gyro_stillness_detect.prev_stillness_confidence = 0;
    gyro_cal->mag_stillness_detect.prev_stillness_confidence = 0;
    gyro_cal->stillness_confidence = 0;
    gyro_cal->prev_still = false;

    // If there are no magnetometer samples being received then
    // operate the calibration algorithm without this sensor.
    if (!gyro_cal->mag_stillness_detect.stillness_window_ready &&
        gyro_cal->using_mag_sensor) {
      gyro_cal->using_mag_sensor = false;
    }

    // Assert watchdog timeout flags.
    gyro_cal->gyro_watchdog_timeout |= watchdog_timeout;
    gyro_cal->gyro_watchdog_start_nanos = 0;
#ifdef GYRO_CAL_DBG_ENABLED
    gyro_cal->debug_watchdog_count++;
    CAL_DEBUG_LOG("[GYRO_CAL:WATCHDOG]", "Total#, Timestamp [nsec]: %lu, %llu",
                  (unsigned long int)gyro_cal->debug_watchdog_count,
                  (unsigned long long int)sample_time_nanos);
#endif
  }
}

#ifdef GYRO_CAL_DBG_ENABLED
void gyroCalUpdateDebug(struct GyroCal* gyro_cal) {
  // Probability of stillness (acc, rot, still), duration, timestamp.
  gyro_cal->debug_gyro_cal.accel_stillness_conf =
      gyro_cal->accel_stillness_detect.prev_stillness_confidence;
  gyro_cal->debug_gyro_cal.gyro_stillness_conf =
      gyro_cal->gyro_stillness_detect.prev_stillness_confidence;
  gyro_cal->debug_gyro_cal.mag_stillness_conf =
      gyro_cal->mag_stillness_detect.prev_stillness_confidence;

  // Magnetometer usage.
  gyro_cal->debug_gyro_cal.using_mag_sensor = gyro_cal->using_mag_sensor;

  // Temperature at calibration time.
  gyro_cal->debug_gyro_cal.temperature_celcius =
      gyro_cal->latest_temperature_celcius;

  // Stillness start, stop, and duration times.
  gyro_cal->debug_gyro_cal.start_still_time_nanos =
      gyro_cal->start_still_time_nanos;
  gyro_cal->debug_gyro_cal.end_still_time_nanos =
      gyro_cal->calibration_time_nanos;
  gyro_cal->debug_gyro_cal.stillness_duration_nanos =
      gyro_cal->calibration_time_duration_nanos;

  // Records the current calibration values.
  gyro_cal->debug_gyro_cal.calibration[0] = gyro_cal->bias_x;
  gyro_cal->debug_gyro_cal.calibration[1] = gyro_cal->bias_y;
  gyro_cal->debug_gyro_cal.calibration[2] = gyro_cal->bias_z;

  // Records the complete temperature statistics.
  gyroTempUpdateStats(&gyro_cal->debug_gyro_cal.debug_temperature, 0,
                      /*reset_stats=*/true);
  gyroSamplingRateUpdate(&gyro_cal->debug_gyro_cal.mean_sampling_rate_hz, 0,
                         /*reset_stats=*/true);

  // Records the previous window means.
  gyro_cal->debug_gyro_cal.accel_mean[0] =
      gyro_cal->accel_stillness_detect.prev_mean_x;
  gyro_cal->debug_gyro_cal.accel_mean[1] =
      gyro_cal->accel_stillness_detect.prev_mean_y;
  gyro_cal->debug_gyro_cal.accel_mean[2] =
      gyro_cal->accel_stillness_detect.prev_mean_z;

  gyro_cal->debug_gyro_cal.gyro_mean[0] =
      gyro_cal->gyro_stillness_detect.prev_mean_x;
  gyro_cal->debug_gyro_cal.gyro_mean[1] =
      gyro_cal->gyro_stillness_detect.prev_mean_y;
  gyro_cal->debug_gyro_cal.gyro_mean[2] =
      gyro_cal->gyro_stillness_detect.prev_mean_z;

  gyro_cal->debug_gyro_cal.mag_mean[0] =
      gyro_cal->mag_stillness_detect.prev_mean_x;
  gyro_cal->debug_gyro_cal.mag_mean[1] =
      gyro_cal->mag_stillness_detect.prev_mean_y;
  gyro_cal->debug_gyro_cal.mag_mean[2] =
      gyro_cal->mag_stillness_detect.prev_mean_z;

  // Records the variance data.
  gyro_cal->debug_gyro_cal.accel_var[0] =
      gyro_cal->accel_stillness_detect.win_var_x;
  gyro_cal->debug_gyro_cal.accel_var[1] =
      gyro_cal->accel_stillness_detect.win_var_y;
  gyro_cal->debug_gyro_cal.accel_var[2] =
      gyro_cal->accel_stillness_detect.win_var_z;

  gyro_cal->debug_gyro_cal.gyro_var[0] =
      gyro_cal->gyro_stillness_detect.win_var_x;
  gyro_cal->debug_gyro_cal.gyro_var[1] =
      gyro_cal->gyro_stillness_detect.win_var_y;
  gyro_cal->debug_gyro_cal.gyro_var[2] =
      gyro_cal->gyro_stillness_detect.win_var_z;

  gyro_cal->debug_gyro_cal.mag_var[0] =
      gyro_cal->mag_stillness_detect.win_var_x;
  gyro_cal->debug_gyro_cal.mag_var[1] =
      gyro_cal->mag_stillness_detect.win_var_y;
  gyro_cal->debug_gyro_cal.mag_var[2] =
      gyro_cal->mag_stillness_detect.win_var_z;
}

void gyroTempUpdateStats(struct DebugTemperature* debug_temperature,
                         float temperature, bool reset_stats) {
  // Using the method of the assumed mean to preserve some numerical stability
  // while avoiding per-sample divisions that the more numerically stable
  // Welford method would afford.

  // Reference for the numerical method used below to compute the online mean
  // and variance statistics:
  //   1). en.wikipedia.org/wiki/assumed_mean

  // This is used for local calculations of temperature statistics.
  static struct DebugTemperature local_temperature_stats = {0};
  static bool set_assumed_mean = true;

  // If 'debug_temperature' is not NULL then this function just reads out the
  // current statistics, resets, and returns.
  if (debug_temperature) {
    if (local_temperature_stats.num_temperature_samples > 1) {
      // Computes the final calculation of temperature sensor mean and variance.
      float tmp = local_temperature_stats.temperature_mean_celsius;
      local_temperature_stats.temperature_mean_celsius /=
          local_temperature_stats.num_temperature_samples;
      local_temperature_stats.temperature_var_celsius =
          (local_temperature_stats.temperature_var_celsius -
           local_temperature_stats.temperature_mean_celsius * tmp) /
          (local_temperature_stats.num_temperature_samples - 1);

      // Adds the assumed mean value back to the total mean calculation.
      local_temperature_stats.temperature_mean_celsius +=
          local_temperature_stats.assumed_mean;
    } else {
      // Not enough samples to compute a valid variance. Indicate this with a -1
      // value.
      local_temperature_stats.temperature_var_celsius = -1.0f;
    }

    memcpy(debug_temperature, &local_temperature_stats,
           sizeof(struct DebugTemperature));
    reset_stats = true;
  }

  // Resets the temperature statistics and returns.
  if (reset_stats) {
    local_temperature_stats.num_temperature_samples = 0;
    local_temperature_stats.temperature_mean_celsius = 0.0f;
    local_temperature_stats.temperature_var_celsius = 0.0f;
    set_assumed_mean = true;  // Sets flag.

    // Initialize the min/max temperatures values.
    local_temperature_stats.temperature_min_max_celsius[0] = FLT_MAX;
    local_temperature_stats.temperature_min_max_celsius[1] =
        -1.0f * (FLT_MAX - 1.0f);
    return;
  }

  // The first sample received is taken as the "assumed mean".
  if (set_assumed_mean) {
    local_temperature_stats.assumed_mean = temperature;
    set_assumed_mean = false;  // Resets flag.
  }

  // Increments the number of samples.
  local_temperature_stats.num_temperature_samples++;

  // Online computation of mean and variance for the running stillness period.
  float delta = (temperature - local_temperature_stats.assumed_mean);
  local_temperature_stats.temperature_var_celsius += delta * delta;
  local_temperature_stats.temperature_mean_celsius += delta;

  // Track the min and max temperature values.
  if (local_temperature_stats.temperature_min_max_celsius[0] > temperature) {
    local_temperature_stats.temperature_min_max_celsius[0] = temperature;
  }
  if (local_temperature_stats.temperature_min_max_celsius[1] < temperature) {
    local_temperature_stats.temperature_min_max_celsius[1] = temperature;
  }
}

void gyroSamplingRateUpdate(float* debug_mean_sampling_rate_hz,
                            uint64_t timestamp_nanos, bool reset_stats) {
  // This is used for local calculations of average sampling rate.
  static uint64_t last_timestamp_nanos = 0;
  static uint64_t time_delta_accumulator = 0;
  static size_t num_samples = 0;

  // If 'debug_mean_sampling_rate_hz' is not NULL then this function just reads
  // out the estimate of the sampling rate.
  if (debug_mean_sampling_rate_hz) {
    if (num_samples > 1 && time_delta_accumulator > 0) {
      // Computes the final mean calculation.
      *debug_mean_sampling_rate_hz =
          num_samples /
          (floatFromUint64(time_delta_accumulator) * NANOS_TO_SEC);
    } else {
      // Not enough samples to compute a valid sample rate estimate. Indicate
      // this with a -1 value.
      *debug_mean_sampling_rate_hz = -1.0f;
    }
    reset_stats = true;
  }

  // Resets the sampling rate mean estimator data if:
  //   1. The 'reset_stats' flag is set.
  //   2. A bad timestamp was received (i.e., time not monotonic).
  //   3. 'last_timestamp_nanos' is zero.
  if (reset_stats || (timestamp_nanos <= last_timestamp_nanos) ||
      last_timestamp_nanos == 0) {
    last_timestamp_nanos = timestamp_nanos;
    time_delta_accumulator = 0;
    num_samples = 0;
    return;
  }

  // Increments the number of samples.
  num_samples++;

  // Accumulate the time steps.
  time_delta_accumulator += timestamp_nanos - last_timestamp_nanos;
  last_timestamp_nanos = timestamp_nanos;
}

void gyroCalDebugPrintData(const struct DebugGyroCal* debug_data,
                           enum DebugPrintData print_data) {
  // Prints out the desired debug data.
  float mag_data;
  switch (print_data) {
    case BIAS_CAL:
      CAL_DEBUG_LOG(
          "[GYRO_CAL:BIAS]",
          "Gyro Bias Calibration [mdps]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
          CAL_ENCODE_FLOAT(debug_data->calibration[0] * RAD_TO_MILLI_DEGREES,
                           8),
          CAL_ENCODE_FLOAT(debug_data->calibration[1] * RAD_TO_MILLI_DEGREES,
                           8),
          CAL_ENCODE_FLOAT(debug_data->calibration[2] * RAD_TO_MILLI_DEGREES,
                           8));
      break;

    case CAL_TIME:
      CAL_DEBUG_LOG("[GYRO_CAL:TIME]", "Stillness Start Time [nsec]: %llu",
                    (unsigned long long int)debug_data->start_still_time_nanos);

      CAL_DEBUG_LOG("[GYRO_CAL:TIME]", "Stillness End Time [nsec]: %llu",
                    (unsigned long long int)debug_data->end_still_time_nanos);

      CAL_DEBUG_LOG(
          "[GYRO_CAL:TIME]", "Stillness Duration [nsec]: %llu",
          (unsigned long long int)debug_data->stillness_duration_nanos);
      break;

    case ACCEL_STATS:
      CAL_DEBUG_LOG("[GYRO_CAL:ACCEL_STATS]",
                    "Accel Mean [m/sec^2]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
                    CAL_ENCODE_FLOAT(debug_data->accel_mean[0], 8),
                    CAL_ENCODE_FLOAT(debug_data->accel_mean[1], 8),
                    CAL_ENCODE_FLOAT(debug_data->accel_mean[2], 8));
      CAL_DEBUG_LOG(
          "[GYRO_CAL:ACCEL_STATS]",
          "Accel Variance [(m/sec^2)^2]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
          CAL_ENCODE_FLOAT(debug_data->accel_var[0], 8),
          CAL_ENCODE_FLOAT(debug_data->accel_var[1], 8),
          CAL_ENCODE_FLOAT(debug_data->accel_var[2], 8));
      break;

    case GYRO_STATS:
      CAL_DEBUG_LOG(
          "[GYRO_CAL:GYRO_STATS]",
          "Gyro Mean [mdps]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
          CAL_ENCODE_FLOAT(debug_data->gyro_mean[0] * RAD_TO_MILLI_DEGREES, 8),
          CAL_ENCODE_FLOAT(debug_data->gyro_mean[1] * RAD_TO_MILLI_DEGREES, 8),
          CAL_ENCODE_FLOAT(debug_data->gyro_mean[2] * RAD_TO_MILLI_DEGREES, 8));
      CAL_DEBUG_LOG(
          "[GYRO_CAL:GYRO_STATS]",
          "Gyro Variance [(rad/sec)^2]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
          CAL_ENCODE_FLOAT(debug_data->gyro_var[0], 8),
          CAL_ENCODE_FLOAT(debug_data->gyro_var[1], 8),
          CAL_ENCODE_FLOAT(debug_data->gyro_var[2], 8));
      break;

    case MAG_STATS:
      if (debug_data->using_mag_sensor) {
        CAL_DEBUG_LOG("[GYRO_CAL:MAG_STATS]",
                      "Mag Mean [uT]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
                      CAL_ENCODE_FLOAT(debug_data->mag_mean[0], 8),
                      CAL_ENCODE_FLOAT(debug_data->mag_mean[1], 8),
                      CAL_ENCODE_FLOAT(debug_data->mag_mean[2], 8));
        CAL_DEBUG_LOG("[GYRO_CAL:MAG_STATS]",
                      "Mag Variance [uT^2]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
                      CAL_ENCODE_FLOAT(debug_data->mag_var[0], 8),
                      CAL_ENCODE_FLOAT(debug_data->mag_var[1], 8),
                      CAL_ENCODE_FLOAT(debug_data->mag_var[2], 8));
      } else {
        CAL_DEBUG_LOG("[GYRO_CAL:MAG_STATS]", "Mag Mean [uT]: 0, 0, 0");
        // The -1's indicate that the magnetometer sensor was not used.
        CAL_DEBUG_LOG("[GYRO_CAL:MAG_STATS]",
                      "Mag Variance [uT^2]: -1.0, -1.0, -1.0");
      }
      break;

    case TEMP_STATS:
      CAL_DEBUG_LOG("[GYRO_CAL:TEMP_STATS]",
                    "Latest Temperature [C]: %s%d.%08d",
                    CAL_ENCODE_FLOAT(debug_data->temperature_celcius, 8));
      CAL_DEBUG_LOG(
          "[GYRO_CAL:TEMP_STATS]",
          "Min/Max Temperature [C]: %s%d.%08d, %s%d.%08d",
          CAL_ENCODE_FLOAT(
              debug_data->debug_temperature.temperature_min_max_celsius[0], 8),
          CAL_ENCODE_FLOAT(
              debug_data->debug_temperature.temperature_min_max_celsius[1], 8));
      CAL_DEBUG_LOG(
          "[GYRO_CAL:TEMP_STATS]", "Temperature Mean [C]: %s%d.%08d",
          CAL_ENCODE_FLOAT(
              debug_data->debug_temperature.temperature_mean_celsius, 8));
      CAL_DEBUG_LOG(
          "[GYRO_CAL:TEMP_STATS]", "Temperature Variance [C^2]: %s%d.%08d",
          CAL_ENCODE_FLOAT(
              debug_data->debug_temperature.temperature_var_celsius, 8));
      break;

    case STILLNESS_DATA:
      mag_data = (debug_data->using_mag_sensor)
                     ? debug_data->mag_stillness_conf
                     : -1.0f;  // Signals that magnetometer was not used.
      CAL_DEBUG_LOG("[GYRO_CAL:STILLNESS]",
                    "Stillness [G/A/M]: %s%d.%08d, %s%d.%08d, %s%d.%08d",
                    CAL_ENCODE_FLOAT(debug_data->gyro_stillness_conf, 8),
                    CAL_ENCODE_FLOAT(debug_data->accel_stillness_conf, 8),
                    CAL_ENCODE_FLOAT(mag_data, 8));
      break;

    case SAMPLING_RATE:
      CAL_DEBUG_LOG("[GYRO_CAL:SAMPLING_RATE]",
                    "Gyro Sampling Rate [Hz]: %s%d.%06d",
                    CAL_ENCODE_FLOAT(
                        debug_data->mean_sampling_rate_hz, 6));
      break;

    default:
      break;
  }
}

// Debug printout state enumeration.
enum GyroCalDebugState {
  IDLE = 0,
  WAIT_STATE,
  PRINT_BIAS,
  PRINT_TIME,
  PRINT_TEMP,
  PRINT_ACCEL,
  PRINT_GYRO,
  PRINT_MAG,
  PRINT_STILLNESS,
  PRINT_SAMPLING_RATE
};

void gyroCalDebugPrint(struct GyroCal* gyro_cal, uint64_t timestamp_nanos) {
  static enum GyroCalDebugState debug_state = IDLE;
  static enum GyroCalDebugState next_state = IDLE;
  static uint64_t wait_timer_nanos = 0;

  // This is a state machine that controls the reporting out of debug data.
  switch (debug_state) {
    case IDLE:
      // Wait for a trigger and start the debug printout sequence.
      if (gyro_cal->debug_print_trigger) {
        debug_state = PRINT_BIAS;
        CAL_DEBUG_LOG("[GYRO_CAL]", "");
        gyro_cal->debug_print_trigger = false;  // Resets trigger.
      } else {
        debug_state = IDLE;
      }
      break;

    case WAIT_STATE:
      // This helps throttle the print statements.
      if ((timestamp_nanos - wait_timer_nanos) >= OVERTEMPCAL_WAIT_TIME_NANOS) {
        debug_state = next_state;
      }
      break;

    case PRINT_BIAS:
      CAL_DEBUG_LOG("[GYRO_CAL:BIAS]", "Total # Calibrations: %lu",
                    (unsigned long int)gyro_cal->debug_calibration_count);
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, BIAS_CAL);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_TIME;             // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_TIME:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, CAL_TIME);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_TEMP;             // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_TEMP:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, TEMP_STATS);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_ACCEL;            // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_ACCEL:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, ACCEL_STATS);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_GYRO;             // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_GYRO:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, GYRO_STATS);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_MAG;              // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_MAG:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, MAG_STATS);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_STILLNESS;        // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_STILLNESS:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, STILLNESS_DATA);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_SAMPLING_RATE;    // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_SAMPLING_RATE:
      gyroCalDebugPrintData(&gyro_cal->debug_gyro_cal, SAMPLING_RATE);
      debug_state = IDLE;
      break;

    default:
      // Sends this state machine to its idle state.
      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = IDLE;                   // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
  }

#ifdef GYRO_CAL_DBG_TUNE_ENABLED
  if (debug_state == IDLE) {
    // This check keeps the tuning printout from interleaving with the above
    // debug print data.
    gyroCalTuneDebugPrint(gyro_cal, timestamp_nanos);
  }
#endif  // GYRO_CAL_DBG_TUNE_ENABLED
}

#ifdef GYRO_CAL_DBG_TUNE_ENABLED
void gyroCalTuneDebugPrint(const struct GyroCal* gyro_cal,
                           uint64_t timestamp_nanos) {
  static enum GyroCalDebugState debug_state = IDLE;
  static enum GyroCalDebugState next_state = IDLE;
  static uint64_t wait_timer_nanos = 0;

  // Output sensor variance levels to assist with tuning thresholds.
  //   i.  Within the first 180 seconds of boot: output interval = 5
  //       seconds.
  //   ii. Thereafter: output interval is 60 seconds.
  bool condition_i =
      ((timestamp_nanos <= 180000000000) &&
       ((timestamp_nanos - wait_timer_nanos) > 5000000000));  // nsec
  bool condition_ii = ((timestamp_nanos > 60000000000) &&
                       ((timestamp_nanos - wait_timer_nanos) > 60000000000));

  // This is a state machine that controls the reporting out of debug data.
  switch (debug_state) {
    case IDLE:
      // Wait for a trigger and start the debug printout sequence.
      if (condition_i || condition_ii) {
        debug_state = PRINT_BIAS;
      } else {
        debug_state = IDLE;
      }
      break;

    case WAIT_STATE:
      // This helps throttle the print statements.
      if ((timestamp_nanos - wait_timer_nanos) >= OVERTEMPCAL_WAIT_TIME_NANOS) {
        debug_state = next_state;
      }
      break;

    case PRINT_BIAS:
      CAL_DEBUG_LOG("[GYRO_CAL]", "");
      CAL_DEBUG_LOG(
          "[GYRO_CAL:TUNE]",
          "#%lu Gyro Bias Calibration = {%s%d.%06d, %s%d.%06d, %s%d.%06d} "
          "[mdps]\n",
          (unsigned long int)gyro_cal->debug_calibration_count,
          CAL_ENCODE_FLOAT(gyro_cal->bias_x * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(gyro_cal->bias_y * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(gyro_cal->bias_z * RAD_TO_MILLI_DEGREES, 6));

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_TIME;             // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_TIME:
      CAL_DEBUG_LOG("[GYRO_CAL:TUNE]", "   Timestamp = %llu [nsec]\n",
                    (unsigned long long int)timestamp_nanos);
      CAL_DEBUG_LOG("[GYRO_CAL:TUNE]", "   Total Gyro Calibrations: %lu\n",
                    (unsigned long int)gyro_cal->debug_calibration_count);

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_TEMP;             // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_TEMP:
      CAL_DEBUG_LOG("[GYRO_CAL:TUNE]", "   Temperature = %s%d.%06d [C]\n",
                    CAL_ENCODE_FLOAT(gyro_cal->latest_temperature_celcius, 6));

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_ACCEL;            // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_ACCEL:
      CAL_DEBUG_LOG(
          "[GYRO_CAL:TUNE]",
          "   Accel Variance = {%s%d.%08d, %s%d.%08d, %s%d.%08d} "
          "[m/sec^2]^2\n",
          CAL_ENCODE_FLOAT(gyro_cal->accel_stillness_detect.win_var_x, 8),
          CAL_ENCODE_FLOAT(gyro_cal->accel_stillness_detect.win_var_y, 8),
          CAL_ENCODE_FLOAT(gyro_cal->accel_stillness_detect.win_var_z, 8));

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_GYRO;             // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_GYRO:
      CAL_DEBUG_LOG(
          "[GYRO_CAL:TUNE]",
          "   Gyro Variance = {%s%d.%08d, %s%d.%08d, %s%d.%08d} [rad/sec]^2\n",
          CAL_ENCODE_FLOAT(gyro_cal->gyro_stillness_detect.win_var_x, 8),
          CAL_ENCODE_FLOAT(gyro_cal->gyro_stillness_detect.win_var_y, 8),
          CAL_ENCODE_FLOAT(gyro_cal->gyro_stillness_detect.win_var_z, 8));

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_MAG;              // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    case PRINT_MAG:
      if (gyro_cal->using_mag_sensor) {
        CAL_DEBUG_LOG(
            "[GYRO_CAL:TUNE]",
            "   Mag Variance = {%s%d.%08d, %s%d.%08d, %s%d.%08d} [uT]^2\n",
            CAL_ENCODE_FLOAT(gyro_cal->mag_stillness_detect.win_var_x, 8),
            CAL_ENCODE_FLOAT(gyro_cal->mag_stillness_detect.win_var_y, 8),
            CAL_ENCODE_FLOAT(gyro_cal->mag_stillness_detect.win_var_z, 8));
        CAL_DEBUG_LOG(
            "[GYRO_CAL:TUNE]",
            "   Stillness = {G%s%d.%03d, A%s%d.%03d, M%s%d.%03d}\n",
            CAL_ENCODE_FLOAT(
                gyro_cal->gyro_stillness_detect.stillness_confidence, 3),
            CAL_ENCODE_FLOAT(
                gyro_cal->accel_stillness_detect.stillness_confidence, 3),
            CAL_ENCODE_FLOAT(
                gyro_cal->mag_stillness_detect.stillness_confidence, 3));
      } else {
        CAL_DEBUG_LOG("[GYRO_CAL:TUNE]",
                      "   Mag Variance = {---, ---, ---} [uT]^2\n");
        CAL_DEBUG_LOG(
            "[GYRO_CAL:TUNE]",
            "   Stillness = {G%s%d.%03d, A%s%d.%03d, M---}\n",
            CAL_ENCODE_FLOAT(
                gyro_cal->gyro_stillness_detect.stillness_confidence, 3),
            CAL_ENCODE_FLOAT(
                gyro_cal->accel_stillness_detect.stillness_confidence, 3));
      }

      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = IDLE;                   // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
      break;

    default:
      // Sends this state machine to its idle state.
      wait_timer_nanos = timestamp_nanos;  // Starts the wait timer.
      next_state = IDLE;                   // Sets the next state.
      debug_state = WAIT_STATE;            // First, go to wait state.
  }
}
#endif  // GYRO_CAL_DBG_TUNE_ENABLED
#endif  // GYRO_CAL_DBG_ENABLED
