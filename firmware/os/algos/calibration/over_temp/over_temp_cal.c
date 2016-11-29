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

#include "calibration/over_temp/over_temp_cal.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "calibration/util/cal_log.h"
#include "common/math/vec.h"
#include "util/nano_assert.h"

/////// DEFINITIONS AND MACROS ////////////////////////////////////////////////

// The 'temp_sensitivity' parameters are set to this value to indicate that the
// model is in its initial state.
#define MODEL_INITIAL_STATE (1e6f)

// Rate-limits the search for the nearest offset estimate to every 10 seconds
// when no model has been updated yet.
#define OVERTEMPCAL_NEAREST_NANOS (10000000000)

// Rate-limits the check of old data to every 2 hours.
#define OVERTEMPCAL_STALE_CHECK_TIME_NANOS (7200000000000)

// A common sensor operating temperature at which to start producing the model
// jump-start data.
#define JUMPSTART_START_TEMP_CELSIUS (30.0f)

#ifdef OVERTEMPCAL_DBG_ENABLED
// A debug version label to help with tracking results.
#define OVERTEMPCAL_DEBUG_VERSION_STRING "[Dec 12, 2016]"

// The time value used to throttle debug messaging.
#define OVERTEMPCAL_WAIT_TIME_NANOS (250000000)

// Debug log tag string used to identify debug report output data.
#define OVERTEMPCAL_REPORT_TAG "[OVER_TEMP_CAL:REPORT]"

// Converts units of radians to milli-degrees.
#define RAD_TO_MILLI_DEGREES (float)(1e3f * 180.0f / M_PI)

// Sensor axis label definition with index correspondence: 0=X, 1=Y, 2=Z.
static const char  kDebugAxisLabel[3] = "XYZ";
#endif  // OVERTEMPCAL_DBG_ENABLED

/////// FORWARD DECLARATIONS //////////////////////////////////////////////////

// Updates the most recent model estimate data.
static void setLatestEstimate(struct OverTempCal *over_temp_cal,
                              const float *offset, float offset_temp,
                              uint64_t timestamp_nanos);

/*
 * Determines if a new over-temperature model fit should be performed, and then
 * updates the model as needed.
 *
 * INPUTS:
 *   over_temp_cal:    Over-temp data structure.
 *   timestamp_nanos:  Current timestamp for the model update.
 */
static void computeModelUpdate(struct OverTempCal *over_temp_cal,
                               uint64_t timestamp_nanos);

/*
 * Searches 'model_data' for the sensor offset estimate closest to the current
 * temperature. Sets the 'latest_offset' pointer to the result.
 */
static void setNearestEstimate(struct OverTempCal *over_temp_cal);

/*
 * Removes the "old" offset estimates from 'model_data' (i.e., eliminates the
 * drift-compromised data). Returns 'true' if any data was removed.
 */
static bool removeStaleModelData(struct OverTempCal *over_temp_cal,
                                 uint64_t timestamp_nanos);

/*
 * Removes the offset estimates from 'model_data' at index, 'model_index'.
 * Returns 'true' if data was removed.
 */
static bool removeModelDataByIndex(struct OverTempCal *over_temp_cal,
                                   size_t model_index);

/*
 * Since it may take a while for an empty model to build up enough data to start
 * producing new model parameter updates, the model collection can be
 * jump-started by using the new model parameters to insert fake data in place
 * of actual sensor offset data.
 */
static bool jumpStartModelData(struct OverTempCal *over_temp_cal);

/*
 * Provides updated model parameters for the over-temperature model data.
 *
 * INPUTS:
 *   over_temp_cal:    Over-temp data structure.
 * OUTPUTS:
 *   temp_sensitivity: Updated modeled temperature sensitivity (array).
 *   sensor_intercept: Updated model intercept (array).
 *
 * NOTE: Arrays are all 3-dimensional with indices: 0=x, 1=y, 2=z.
 *
 * Reference: "Comparing two ways to fit a line to data", John D. Cook.
 * http://www.johndcook.com/blog/2008/10/20/comparing-two-ways-to-fit-a-line-to-data/
 */
static void updateModel(const struct OverTempCal *over_temp_cal,
                        float *temp_sensitivity, float *sensor_intercept);

/*
 * Removes the over-temp compensated offset from the input sensor data.
 *
 * INPUTS:
 *   over_temp_cal:    Over-temp data structure.
 *   vi:               Single axis sensor data to be compensated.
 *   index:            Index for model parameter compensation (0=x, 1=y, 2=z).
 * OUTPUTS:
 *   vo:               Single axis sensor data that has been compensated.
 */
static void removeSensorOffset(const struct OverTempCal *over_temp_cal,
                               float vi, size_t index, float *vo);

/*
 * Computes the over-temperature compensated sensor offset estimate based on the
 * input model parameters. Note, this is a single axis calculation.
 *   comp_offset = (temp_sensitivity * temperature + sensor_intercept)
 *
 * INPUTS:
 *   temperature:      Temperature value at which to compute the estimate.
 *   temp_sensitivity: Temperature sensitivity model parameter.
 *   sensor_intercept: Sensor intercept model parameter.
 * RETURNS:
 *   comp_offset:      Over-temperature compensated sensor offset estimate.
 */
static float getCompensatedOffset(float temperature, float temp_sensitivity,
                                  float sensor_intercept);

/////// FUNCTION DEFINITIONS //////////////////////////////////////////////////

void overTempCalInit(struct OverTempCal *over_temp_cal,
                     size_t min_num_model_pts,
                     uint64_t min_update_interval_nanos,
                     float delta_temp_per_bin, float max_error_limit,
                     uint64_t age_limit_nanos, float temp_sensitivity_limit,
                     float sensor_intercept_limit, bool over_temp_enable) {
  ASSERT_NOT_NULL(over_temp_cal);

  // Clears OverTempCal memory.
  memset(over_temp_cal, 0, sizeof(struct OverTempCal));

  // Initializes the pointer to the most recent sensor offset estimate. Sets it
  // as the first element in 'model_data'.
  over_temp_cal->latest_offset = &over_temp_cal->model_data[0];

  // Sets the temperature sensitivity model parameters to MODEL_INITIAL_STATE to
  // indicate that the model is in an "initial" state.
  over_temp_cal->temp_sensitivity[0] = MODEL_INITIAL_STATE;
  over_temp_cal->temp_sensitivity[1] = MODEL_INITIAL_STATE;
  over_temp_cal->temp_sensitivity[2] = MODEL_INITIAL_STATE;

  // Initializes the model identification parameters.
  over_temp_cal->new_overtemp_model_available = false;
  over_temp_cal->min_num_model_pts = min_num_model_pts;
  over_temp_cal->min_update_interval_nanos = min_update_interval_nanos;
  over_temp_cal->delta_temp_per_bin = delta_temp_per_bin;
  over_temp_cal->max_error_limit = max_error_limit;
  over_temp_cal->age_limit_nanos = age_limit_nanos;
  over_temp_cal->temp_sensitivity_limit = temp_sensitivity_limit;
  over_temp_cal->sensor_intercept_limit = sensor_intercept_limit;
  over_temp_cal->over_temp_enable = over_temp_enable;

#ifdef OVERTEMPCAL_DBG_ENABLED
  CAL_DEBUG_LOG("[OVER_TEMP_CAL:MEMORY]", "sizeof(struct OverTempCal): %lu",
                (unsigned long int)sizeof(struct OverTempCal));
  CAL_DEBUG_LOG("[OVER_TEMP_CAL:INIT]", "Over-Temp Cal: Initialized.");

#endif  // OVERTEMPCAL_DBG_ENABLED
}

void overTempCalSetModel(struct OverTempCal *over_temp_cal, const float *offset,
                         float offset_temp, uint64_t timestamp_nanos,
                         const float *temp_sensitivity,
                         const float *sensor_intercept, bool jump_start_model) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(offset);
  ASSERT_NOT_NULL(temp_sensitivity);
  ASSERT_NOT_NULL(sensor_intercept);

  // Sets the model parameters if they are within the acceptable limits.
  size_t i;
  for (i = 0; i < 3; i++) {
    if (NANO_ABS(temp_sensitivity[i]) < over_temp_cal->temp_sensitivity_limit &&
        NANO_ABS(sensor_intercept[i]) < over_temp_cal->sensor_intercept_limit) {
      over_temp_cal->temp_sensitivity[i] = temp_sensitivity[i];
      over_temp_cal->sensor_intercept[i] = sensor_intercept[i];
    }
  }

  // Sets the model update time to the current timestamp.
  over_temp_cal->modelupdate_timestamp_nanos = timestamp_nanos;

  // Model "Jump-Start".
  const bool model_jump_started =
      (jump_start_model) ? jumpStartModelData(over_temp_cal) : false;

  if (!model_jump_started) {
    // Sets the initial over-temp calibration estimate and model data.
    setLatestEstimate(over_temp_cal, offset, offset_temp, timestamp_nanos);

    // Now there is one offset estimate in the model.
    over_temp_cal->num_model_pts = 1;
  }

#ifdef OVERTEMPCAL_DBG_ENABLED
  // Prints the updated model data.
  CAL_DEBUG_LOG("[OVER_TEMP_CAL:RECALL]",
                "Model parameters recalled from memory.");

  // Trigger a debug print out to view the new model parameters.
  over_temp_cal->debug_print_trigger = true;
#endif  // OVERTEMPCAL_DBG_ENABLED
}

void overTempCalGetModel(struct OverTempCal *over_temp_cal, float *offset,
                         float *offset_temp, uint64_t *timestamp_nanos,
                         float *temp_sensitivity, float *sensor_intercept) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(over_temp_cal->latest_offset);
  ASSERT_NOT_NULL(offset);
  ASSERT_NOT_NULL(offset_temp);
  ASSERT_NOT_NULL(timestamp_nanos);
  ASSERT_NOT_NULL(temp_sensitivity);
  ASSERT_NOT_NULL(sensor_intercept);

  // Updates the latest offset so that it is the one nearest to the current
  // temperature.
  setNearestEstimate(over_temp_cal);

  // Gets the over-temp calibration estimate and model data.
  memcpy(offset, over_temp_cal->latest_offset->offset, 3 * sizeof(float));
  memcpy(temp_sensitivity, over_temp_cal->temp_sensitivity, 3 * sizeof(float));
  memcpy(sensor_intercept, over_temp_cal->sensor_intercept, 3 * sizeof(float));
  *offset_temp = over_temp_cal->latest_offset->offset_temp;
  *timestamp_nanos = over_temp_cal->latest_offset->timestamp_nanos;
}

void overTempCalRemoveOffset(struct OverTempCal *over_temp_cal,
                             uint64_t timestamp_nanos, float xi, float yi,
                             float zi, float *xo, float *yo, float *zo) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(over_temp_cal->latest_offset);
  ASSERT_NOT_NULL(xo);
  ASSERT_NOT_NULL(yo);
  ASSERT_NOT_NULL(zo);

  // Determines whether over-temp compensation will be applied.
  if (!over_temp_cal->over_temp_enable) {
    return;
  }

  // Removes very old data from the collected model estimates (eliminates
  // drift-compromised data). Only does this when there is more than one
  // estimate in the model (i.e., don't want to remove all data, even if it is
  // very old [something is likely better than nothing]).
  if ((timestamp_nanos - over_temp_cal->stale_data_timer) >=
          OVERTEMPCAL_STALE_CHECK_TIME_NANOS &&
      over_temp_cal->num_model_pts > 1) {
    over_temp_cal->stale_data_timer = timestamp_nanos;  // Resets timer.

    if (removeStaleModelData(over_temp_cal, timestamp_nanos)) {
      // If anything was removed, then this attempts to recompute the model.
      computeModelUpdate(over_temp_cal, timestamp_nanos);
    }
  }

  // Removes the over-temperature compensated offset from the input sensor data.
  removeSensorOffset(over_temp_cal, xi, 0, xo);
  removeSensorOffset(over_temp_cal, yi, 1, yo);
  removeSensorOffset(over_temp_cal, zi, 2, zo);
}

bool overTempCalNewModelUpdateAvailable(struct OverTempCal *over_temp_cal) {
  ASSERT_NOT_NULL(over_temp_cal);
  const bool update_available = over_temp_cal->new_overtemp_model_available &&
                                over_temp_cal->over_temp_enable;

  // The 'new_overtemp_model_available' flag is reset when it is read here.
  over_temp_cal->new_overtemp_model_available = false;

  return update_available;
}

void overTempCalUpdateSensorEstimate(struct OverTempCal *over_temp_cal,
                                     uint64_t timestamp_nanos,
                                     const float *offset,
                                     float temperature_celsius) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(over_temp_cal->latest_offset);
  ASSERT_NOT_NULL(offset);
  ASSERT(over_temp_cal->delta_temp_per_bin > 0);

  // Prevent a divide by zero below.
  if (over_temp_cal->delta_temp_per_bin <= 0) {
    return;
  }

#ifdef OVERTEMPCAL_DBG_ENABLED
  // Updates the total count of offset estimates.
  over_temp_cal->debug_num_estimates++;

  // If there are fewer than the minimum number of points to produce a model,
  // then trigger a debug printout to view the model building process.
  over_temp_cal->debug_print_trigger |=
      (over_temp_cal->num_model_pts <= over_temp_cal->min_num_model_pts);
#endif  // OVERTEMPCAL_DBG_ENABLED

  // Provides an early escape if this is the first model estimate.
  if (over_temp_cal->num_model_pts == 0) {
    setLatestEstimate(over_temp_cal, offset, temperature_celsius,
                      timestamp_nanos);
    over_temp_cal->num_model_pts = 1;  // one estimate was added above.
    return;
  }

  // Computes the temperature bin range data.
  const int32_t bin_num =
      CAL_FLOOR(temperature_celsius / over_temp_cal->delta_temp_per_bin);
  const float temp_lo_check = bin_num * over_temp_cal->delta_temp_per_bin;
  const float temp_hi_check = (bin_num + 1) * over_temp_cal->delta_temp_per_bin;

  // The rules for accepting new offset estimates into the 'model_data'
  // collection:
  //    1) The temperature domain is divided into bins each spanning
  //       'delta_temp_per_bin'.
  //    2) Find and replace the i'th 'model_data' estimate data if:
  //          Let, bin_num = floor(temperature_celsius / delta_temp_per_bin)
  //          temp_lo_check = bin_num * delta_temp_per_bin
  //          temp_hi_check = (bin_num + 1) * delta_temp_per_bin
  //          Check condition:
  //            temp_lo_check <= model_data[i].offset_temp < temp_hi_check
  bool replaced_one = false;
  size_t i = 0;
  for (i = 0; i < over_temp_cal->num_model_pts; i++) {
    if (over_temp_cal->model_data[i].offset_temp < temp_hi_check &&
        over_temp_cal->model_data[i].offset_temp >= temp_lo_check) {
      // NOTE - the pointer to the estimate that is getting replaced is set
      // here; the offset values are set below in the call to
      // 'setLatestEstimate'.
      over_temp_cal->latest_offset = &over_temp_cal->model_data[i];
      replaced_one = true;
      break;
    }
  }

  //    3) If nothing was replaced, and the 'model_data' buffer is not full
  //       then add the estimate data to the array.
  //    4) Otherwise (nothing was replaced and buffer is full), replace the
  //       'latest_offset' with the incoming one. This is done below.
  if (!replaced_one && over_temp_cal->num_model_pts < OVERTEMPCAL_MODEL_SIZE) {
    // NOTE - the pointer to the next available array location is set here;
    // the offset values are set below in the call to 'setLatestEstimate'.
    over_temp_cal->latest_offset =
        &over_temp_cal->model_data[over_temp_cal->num_model_pts];
    over_temp_cal->num_model_pts++;
  }

  // Updates the latest model estimate data.
  setLatestEstimate(over_temp_cal, offset, temperature_celsius,
                    timestamp_nanos);

  // Conditionally updates the over-temp model. See 'computeModelUpdate' for
  // update conditions.
  computeModelUpdate(over_temp_cal, timestamp_nanos);

}

void overTempCalSetTemperature(struct OverTempCal *over_temp_cal,
                               uint64_t timestamp_nanos,
                               float temperature_celsius) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(over_temp_cal->latest_offset);

#ifdef OVERTEMPCAL_DBG_ENABLED
#ifdef OVERTEMPCAL_DBG_LOG_TEMP
  static uint64_t wait_timer = 0;
  // Prints the sensor temperature trajectory for debugging purposes.
  // This throttles the print statements.
  if ((timestamp_nanos - wait_timer) >= 1000000000) {
    wait_timer = timestamp_nanos;  // Starts the wait timer.

    // Prints out temperature and the current timestamp.
    CAL_DEBUG_LOG("[OVER_TEMP_CAL:TEMP]",
                  "Temperature|Time [C|nsec] = %s%d.%06d, %llu",
                  CAL_ENCODE_FLOAT(temperature_celsius, 6),
                  (unsigned long long int)timestamp_nanos);
  }
#endif  // OVERTEMPCAL_DBG_LOG_TEMP
#endif  // OVERTEMPCAL_DBG_ENABLED

  // Updates the sensor temperature.
  over_temp_cal->temperature_celsius = temperature_celsius;

  // If any of the models for the sensor axes are in an initial state, then
  // this searches for the sensor offset estimate closest to the current
  // temperature. A timer is used to limit the rate at which this search is
  // performed.
  if (over_temp_cal->num_model_pts > 1 &&
      (over_temp_cal->temp_sensitivity[0] >= MODEL_INITIAL_STATE ||
       over_temp_cal->temp_sensitivity[1] >= MODEL_INITIAL_STATE ||
       over_temp_cal->temp_sensitivity[2] >= MODEL_INITIAL_STATE) &&
      (timestamp_nanos - over_temp_cal->nearest_search_timer) >=
          OVERTEMPCAL_NEAREST_NANOS) {
    setNearestEstimate(over_temp_cal);
    over_temp_cal->nearest_search_timer = timestamp_nanos;  // Reset timer.
  }
}

void getModelError(const struct OverTempCal *over_temp_cal,
                   const float *temp_sensitivity, const float *sensor_intercept,
                   float *max_error) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(temp_sensitivity);
  ASSERT_NOT_NULL(sensor_intercept);
  ASSERT_NOT_NULL(max_error);

  size_t i;
  size_t j;
  float max_error_test;
  memset(max_error, 0, 3 * sizeof(float));

  for (i = 0; i < over_temp_cal->num_model_pts; i++) {
    for (j = 0; j < 3; j++) {
      max_error_test =
          NANO_ABS(over_temp_cal->model_data[i].offset[j] -
              getCompensatedOffset(over_temp_cal->model_data[i].offset_temp,
                                   temp_sensitivity[j], sensor_intercept[j]));
      if (max_error_test > max_error[j]) {
        max_error[j] = max_error_test;
      }
    }
  }
}

/////// LOCAL HELPER FUNCTION DEFINITIONS /////////////////////////////////////

void setLatestEstimate(struct OverTempCal *over_temp_cal, const float *offset,
                       float offset_temp, uint64_t timestamp_nanos) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(offset);
  ASSERT_NOT_NULL(over_temp_cal->latest_offset);

  // Sets the latest over-temp calibration estimate.
  over_temp_cal->latest_offset->offset[0] = offset[0];
  over_temp_cal->latest_offset->offset[1] = offset[1];
  over_temp_cal->latest_offset->offset[2] = offset[2];
  over_temp_cal->latest_offset->offset_temp = offset_temp;
  over_temp_cal->latest_offset->timestamp_nanos = timestamp_nanos;
}

void computeModelUpdate(struct OverTempCal *over_temp_cal,
                        uint64_t timestamp_nanos) {
  ASSERT_NOT_NULL(over_temp_cal);

  // The rules for determining whether a new model fit is computed are:
  //    1) A minimum number of data points must have been collected:
  //          num_model_pts >= min_num_model_pts
  //       NOTE: Collecting 'num_model_pts' and given that only one point is
  //       kept per temperature bin (spanning a thermal range specified by
  //       'delta_temp_per_bin'), implies that model data covers at least,
  //          model_temp_span >= 'num_model_pts' * delta_temp_per_bin
  //    2) New model updates will not occur for intervals less than:
  //          (current_timestamp_nanos - modelupdate_timestamp_nanos) <
  //            min_update_interval_nanos
  if (over_temp_cal->num_model_pts < over_temp_cal->min_num_model_pts ||
      (timestamp_nanos - over_temp_cal->modelupdate_timestamp_nanos) <
          over_temp_cal->min_update_interval_nanos) {
    return;
  }

  // Updates the linear model fit.
  float temp_sensitivity[3];
  float sensor_intercept[3];
  updateModel(over_temp_cal, temp_sensitivity, sensor_intercept);

  // Computes the maximum error over all of the model data.
  float max_error[3];
  getModelError(over_temp_cal, temp_sensitivity, sensor_intercept, max_error);

  //    3) A new set of model parameters are accepted if:
  //         i.  The model fit error is less than, 'max_error_limit'. See
  //             getModelError() for error metric description.
  //         ii. The model fit parameters must be within certain absolute
  //             bounds:
  //               a. NANO_ABS(temp_sensitivity) < temp_sensitivity_limit
  //               b. NANO_ABS(sensor_intercept) < sensor_intercept_limit
  size_t i;
  for (i = 0; i < 3; i++) {
    if (max_error[i] < over_temp_cal->max_error_limit &&
        NANO_ABS(temp_sensitivity[i]) < over_temp_cal->temp_sensitivity_limit &&
        NANO_ABS(sensor_intercept[i]) < over_temp_cal->sensor_intercept_limit) {
      over_temp_cal->temp_sensitivity[i] = temp_sensitivity[i];
      over_temp_cal->sensor_intercept[i] = sensor_intercept[i];
    } else {
#ifdef OVERTEMPCAL_DBG_ENABLED
      CAL_DEBUG_LOG(
          "[OVER_TEMP_CAL:REJECT]",
          "Rejected %c-Axis Parameters|Max Error [mdps/C|mdps|mdps] = "
          "%s%d.%06d, %s%d.%06d, %s%d.%06d",
          kDebugAxisLabel[i],
          CAL_ENCODE_FLOAT(temp_sensitivity[i] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(sensor_intercept[i] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(max_error[i] * RAD_TO_MILLI_DEGREES, 6));
#endif  // OVERTEMPCAL_DBG_ENABLED
    }
  }

  // Resets the timer and sets the update flag.
  over_temp_cal->modelupdate_timestamp_nanos = timestamp_nanos;
  over_temp_cal->new_overtemp_model_available = true;

  // Track the total number of model updates, and set trigger to print log data.
#ifdef OVERTEMPCAL_DBG_ENABLED
  over_temp_cal->debug_num_model_updates++;
  over_temp_cal->debug_print_trigger |= true;
#endif  // OVERTEMPCAL_DBG_ENABLED
}

void setNearestEstimate(struct OverTempCal *over_temp_cal) {
  ASSERT_NOT_NULL(over_temp_cal);

  size_t i = 0;
  float dtemp_new = 0.0f;
  float dtemp_old = FLT_MAX;
  struct OverTempCalDataPt *nearest_estimate = &over_temp_cal->model_data[0];
  for (i = 1; i < over_temp_cal->num_model_pts; i++) {
    dtemp_new = NANO_ABS(over_temp_cal->model_data[i].offset_temp -
                    over_temp_cal->temperature_celsius);
    if (dtemp_new < dtemp_old) {
      nearest_estimate = &over_temp_cal->model_data[i];
      dtemp_old = dtemp_new;
    }
  }

  // Set the 'latest_offset' to the estimate nearest the current temperature.
  over_temp_cal->latest_offset = nearest_estimate;
}

bool removeStaleModelData(struct OverTempCal *over_temp_cal,
                          uint64_t timestamp_nanos) {
  ASSERT_NOT_NULL(over_temp_cal);

  size_t i;
  bool removed_one = false;
  for (i = 0; i < over_temp_cal->num_model_pts; i++) {
    if ((timestamp_nanos - over_temp_cal->model_data[i].timestamp_nanos) >
        over_temp_cal->age_limit_nanos) {
      removed_one |= removeModelDataByIndex(over_temp_cal, i);
    }
  }

  // Updates the latest offset so that it is the one nearest to the current
  // temperature.
  setNearestEstimate(over_temp_cal);

  return removed_one;
}

bool removeModelDataByIndex(struct OverTempCal *over_temp_cal,
                            size_t model_index) {
  ASSERT_NOT_NULL(over_temp_cal);

  // This function will not remove all of the model data. At least one model
  // sample will be left.
  if (over_temp_cal->num_model_pts <= 1) {
    return false;
  }

  // Remove the model data at 'model_index'.
  size_t i;
  for (i = model_index; i < over_temp_cal->num_model_pts - 1; i++) {
    memcpy(&over_temp_cal->model_data[i], &over_temp_cal->model_data[i + 1],
           sizeof(struct OverTempCalDataPt));
  }
  over_temp_cal->num_model_pts--;

#ifdef OVERTEMPCAL_DBG_ENABLED
  CAL_DEBUG_LOG("[OVER_TEMP_CAL:REMOVE]",
                "Removed Stale Data:  Model Index = %lu",
                (unsigned long int)model_index);
#endif  // OVERTEMPCAL_DBG_ENABLED

  return true;
}

bool jumpStartModelData(struct OverTempCal *over_temp_cal) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT(over_temp_cal->delta_temp_per_bin > 0);

  // Prevent a divide by zero below.
  if (over_temp_cal->delta_temp_per_bin <= 0) {
    return false;
  }

  // In normal operation the offset estimates enter into the 'model_data' array
  // complete (i.e., x, y, z values are all provided). Therefore, the jumpstart
  // data produced here requires that the model parameters have all been fully
  // defined (i.e., no models in an initial state) and are all within the valid
  // range (this is assumed to have been checked prior to this function). There
  // must also be no preexisting model data; that is, this function will not
  // replace any actual offset estimates already buffered.
  if (over_temp_cal->num_model_pts > 0 ||
      over_temp_cal->temp_sensitivity[0] >= MODEL_INITIAL_STATE ||
      over_temp_cal->temp_sensitivity[1] >= MODEL_INITIAL_STATE ||
      over_temp_cal->temp_sensitivity[2] >= MODEL_INITIAL_STATE) {
    return false;
  }

  // This defines the minimum contiguous set of points to allow a model update
  // when the next offset estimate is received. They are placed at a common
  // temperature range that is likely to get replaced with actual data soon.
  const int32_t start_bin_num = CAL_FLOOR(JUMPSTART_START_TEMP_CELSIUS /
                                          over_temp_cal->delta_temp_per_bin);
  float offset_temp =
      start_bin_num * (1.5f * over_temp_cal->delta_temp_per_bin);

  size_t i;
  size_t j;
  for (i = 0; i < over_temp_cal->min_num_model_pts; i++) {
    float offset[3];
    const uint64_t timestamp_nanos = over_temp_cal->modelupdate_timestamp_nanos;
    for (j = 0; j < 3; j++) {
      offset[j] =
          getCompensatedOffset(offset_temp, over_temp_cal->temp_sensitivity[j],
                               over_temp_cal->sensor_intercept[j]);
    }
    over_temp_cal->latest_offset = &over_temp_cal->model_data[i];
    setLatestEstimate(over_temp_cal, offset, offset_temp, timestamp_nanos);
    offset_temp += over_temp_cal->delta_temp_per_bin;
    over_temp_cal->num_model_pts++;
  }

#ifdef OVERTEMPCAL_DBG_ENABLED
  if (over_temp_cal->min_num_model_pts > 0) {
    CAL_DEBUG_LOG("[OVER_TEMP_CAL]", "Jump-Started Model:  #Points = %lu.",
                  (unsigned long int)over_temp_cal->min_num_model_pts);
  }
#endif  // OVERTEMPCAL_DBG_ENABLED

  return (over_temp_cal->min_num_model_pts > 0);
}

void updateModel(const struct OverTempCal *over_temp_cal,
                 float *temp_sensitivity, float *sensor_intercept) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(temp_sensitivity);
  ASSERT_NOT_NULL(sensor_intercept);
  ASSERT(over_temp_cal->num_model_pts > 0);

  float st = 0.0f, stt = 0.0f;
  float sx = 0.0f, stsx = 0.0f;
  float sy = 0.0f, stsy = 0.0f;
  float sz = 0.0f, stsz = 0.0f;
  const size_t n = over_temp_cal->num_model_pts;
  size_t i = 0;

  // First pass computes the mean values.
  for (i = 0; i < n; ++i) {
    st += over_temp_cal->model_data[i].offset_temp;
    sx += over_temp_cal->model_data[i].offset[0];
    sy += over_temp_cal->model_data[i].offset[1];
    sz += over_temp_cal->model_data[i].offset[2];
  }

  // Second pass computes the mean corrected second moment values.
  const float inv_n = 1.0f / n;
  for (i = 0; i < n; ++i) {
    const float t = over_temp_cal->model_data[i].offset_temp - st * inv_n;
    stt += t * t;
    stsx += t * over_temp_cal->model_data[i].offset[0];
    stsy += t * over_temp_cal->model_data[i].offset[1];
    stsz += t * over_temp_cal->model_data[i].offset[2];
  }

  // Calculates the linear model fit parameters.
  ASSERT(stt > 0);
  const float inv_stt = 1.0f / stt;
  temp_sensitivity[0] = stsx * inv_stt;
  sensor_intercept[0] = (sx - st * temp_sensitivity[0]) * inv_n;
  temp_sensitivity[1] = stsy * inv_stt;
  sensor_intercept[1] = (sy - st * temp_sensitivity[1]) * inv_n;
  temp_sensitivity[2] = stsz * inv_stt;
  sensor_intercept[2] = (sz - st * temp_sensitivity[2]) * inv_n;
}

void removeSensorOffset(const struct OverTempCal *over_temp_cal, float vi,
                        size_t index, float *vo) {
  // Removes the over-temperature compensated offset from the input sensor data.
  if (over_temp_cal->temp_sensitivity[index] >= MODEL_INITIAL_STATE) {
    // If this axis is in its initial state, use the nearest estimate to perform
    // the compensation (in this case the latest estimate will be the nearest):
    // sensor_out = sensor_in - nearest_offset
    *vo = vi - over_temp_cal->latest_offset->offset[index];
  } else {
    // sensor_out = sensor_in - compensated_offset
    // Where,
    //  compensated_offset = (temp_sensitivity * temp_meas + sensor_intercept)
    *vo = vi - getCompensatedOffset(over_temp_cal->temperature_celsius,
                                    over_temp_cal->temp_sensitivity[index],
                                    over_temp_cal->sensor_intercept[index]);
  }
}

float getCompensatedOffset(float temperature, float temp_sensitivity,
                           float sensor_intercept) {
  return temp_sensitivity * temperature + sensor_intercept;
}

/////// DEBUG FUNCTION DEFINITIONS ////////////////////////////////////////////

#ifdef OVERTEMPCAL_DBG_ENABLED
// Debug printout state enumeration.
enum DebugState {
  IDLE = 0,
  WAIT_STATE,
  PRINT_HEADER,
  PRINT_OFFSET,
  PRINT_SENSITIVITY,
  PRINT_INTERCEPT,
  PRINT_ERROR,
  PRINT_MODEL_PTS,
  PRINT_MODEL_DATA
};

void overTempCalDebugPrint(struct OverTempCal *over_temp_cal,
                           uint64_t timestamp_nanos) {
  ASSERT_NOT_NULL(over_temp_cal);
  ASSERT_NOT_NULL(over_temp_cal->latest_offset);

  static enum DebugState debug_state = IDLE;
  static enum DebugState next_state = 0;
  static uint64_t wait_timer = 0;
  static size_t i = 0;  // Counter.

  // NOTE - The un-initialized model state is indicated by
  // temp_sensitivity=MODEL_INITIAL_STATE. The following filters out this
  // condition for the data printout below.
  float compensated_offset[3];
  float temp_sensitivity[3];
  float max_error[3];
  size_t j;
  for (j = 0; j < 3; j++) {
    temp_sensitivity[j] =
        (over_temp_cal->temp_sensitivity[j] >= MODEL_INITIAL_STATE)
            ? 0.0f
            : over_temp_cal->temp_sensitivity[j];
  }

  // This is a state machine that controls the reporting out of debug data.
  switch (debug_state) {
    case IDLE:
      // Wait for a trigger and start the debug printout sequence.
      if (over_temp_cal->debug_print_trigger) {
        debug_state = PRINT_HEADER;
        CAL_DEBUG_LOG(OVERTEMPCAL_REPORT_TAG, "");
        over_temp_cal->debug_print_trigger = false;  // Resets trigger.
      } else {
        debug_state = IDLE;
      }
      break;

    case PRINT_HEADER:
      // Print out header.
      CAL_DEBUG_LOG(OVERTEMPCAL_REPORT_TAG, "Debug Version: %s",
                    OVERTEMPCAL_DEBUG_VERSION_STRING);

      // Prints out number of offsets.
      CAL_DEBUG_LOG(OVERTEMPCAL_REPORT_TAG, "Total Offsets = %lu",
                    (unsigned long int)over_temp_cal->debug_num_estimates);

      wait_timer = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_OFFSET;     // Sets the next state.
      debug_state = WAIT_STATE;      // First, go to wait state.
      break;

    case PRINT_OFFSET:
      // Computes the compensated sensor offset based on the current
      // temperature.
      for (j = 0; j < 3; j++) {
        compensated_offset[j] =
            (over_temp_cal->temp_sensitivity[j] >= MODEL_INITIAL_STATE)
                ? over_temp_cal->latest_offset->offset[j]
                : getCompensatedOffset(over_temp_cal->temperature_celsius,
                                       over_temp_cal->temp_sensitivity[j],
                                       over_temp_cal->sensor_intercept[j]);
      }

      CAL_DEBUG_LOG(
          OVERTEMPCAL_REPORT_TAG,
          "Offset|Temp|Time [mdps|C|nsec] = %s%d.%06d, %s%d.%06d, "
          "%s%d.%06d, %s%d.%06d, %llu",
          CAL_ENCODE_FLOAT(compensated_offset[0] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(compensated_offset[1] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(compensated_offset[2] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(over_temp_cal->temperature_celsius, 6),
          (unsigned long long int)
            over_temp_cal->latest_offset->timestamp_nanos);

      wait_timer = timestamp_nanos;    // Starts the wait timer.
      next_state = PRINT_SENSITIVITY;  // Sets the next state.
      debug_state = WAIT_STATE;        // First, go to wait state.
      break;

    case WAIT_STATE:
      // This helps throttle the print statements.
      if ((timestamp_nanos - wait_timer) >= OVERTEMPCAL_WAIT_TIME_NANOS) {
        debug_state = next_state;
      }
      break;

    case PRINT_SENSITIVITY:
      // Prints out the modeled temperature sensitivity.
      CAL_DEBUG_LOG(
          OVERTEMPCAL_REPORT_TAG,
          "Modeled Temperature Sensitivity [mdps/C] = %s%d.%06d, %s%d.%06d, "
          "%s%d.%06d",
          CAL_ENCODE_FLOAT(temp_sensitivity[0] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(temp_sensitivity[1] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(temp_sensitivity[2] * RAD_TO_MILLI_DEGREES, 6));

      wait_timer = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_INTERCEPT;  // Sets the next state.
      debug_state = WAIT_STATE;      // First, go to wait state.
      break;

    case PRINT_INTERCEPT:
      // Prints out the modeled temperature intercept.
      CAL_DEBUG_LOG(
          OVERTEMPCAL_REPORT_TAG,
          "Modeled Temperature Intercept [mdps] = %s%d.%06d, %s%d.%06d, "
          "%s%d.%06d",
          CAL_ENCODE_FLOAT(
              over_temp_cal->sensor_intercept[0] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(
              over_temp_cal->sensor_intercept[1] * RAD_TO_MILLI_DEGREES, 6),
          CAL_ENCODE_FLOAT(
              over_temp_cal->sensor_intercept[2] * RAD_TO_MILLI_DEGREES, 6));

      wait_timer = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_ERROR;      // Sets the next state.
      debug_state = WAIT_STATE;      // First, go to wait state.
      break;

    case PRINT_ERROR:
      // Computes the maximum error over all of the model data.
      if (over_temp_cal->num_model_pts > 0) {
        getModelError(over_temp_cal, temp_sensitivity,
                      over_temp_cal->sensor_intercept, max_error);

        // Reports the resulting model error.
        CAL_DEBUG_LOG(
            OVERTEMPCAL_REPORT_TAG,
            "Model Error [mdps] = %s%d.%06d, %s%d.%06d, %s%d.%06d",
            CAL_ENCODE_FLOAT(max_error[0] * RAD_TO_MILLI_DEGREES, 6),
            CAL_ENCODE_FLOAT(max_error[1] * RAD_TO_MILLI_DEGREES, 6),
            CAL_ENCODE_FLOAT(max_error[2] * RAD_TO_MILLI_DEGREES, 6));
      }

      wait_timer = timestamp_nanos;  // Starts the wait timer.
      next_state = PRINT_MODEL_PTS;  // Sets the next state.
      debug_state = WAIT_STATE;      // First, go to wait state.
      break;

    case PRINT_MODEL_PTS:
      // Prints out the number of model points/updates.
      CAL_DEBUG_LOG(OVERTEMPCAL_REPORT_TAG, "Number of Model Points = %lu",
                    (unsigned long int)over_temp_cal->num_model_pts);

      CAL_DEBUG_LOG(OVERTEMPCAL_REPORT_TAG, "Number of Model Updates = %lu",
                    (unsigned long int)over_temp_cal->debug_num_model_updates);

      i = 0;                          // Resets the counter.
      wait_timer = timestamp_nanos;   // Starts the wait timer.
      next_state = PRINT_MODEL_DATA;  // Sets the next state.
      debug_state = WAIT_STATE;       // First, go to wait state.
      break;

    case PRINT_MODEL_DATA:
      // Prints out all of the model data.
      if (i < over_temp_cal->num_model_pts) {
        CAL_DEBUG_LOG(
            OVERTEMPCAL_REPORT_TAG,
            "  Model[%lu] [mdps|C] = %s%d.%06d, %s%d.%06d, %s%d.%06d, "
            "%s%d.%03d ",
            (unsigned long int)i,
            CAL_ENCODE_FLOAT(
                over_temp_cal->model_data[i].offset[0] * RAD_TO_MILLI_DEGREES,
                6),
            CAL_ENCODE_FLOAT(
                over_temp_cal->model_data[i].offset[1] * RAD_TO_MILLI_DEGREES,
                6),
            CAL_ENCODE_FLOAT(
                over_temp_cal->model_data[i].offset[2] * RAD_TO_MILLI_DEGREES,
                6),
            CAL_ENCODE_FLOAT(over_temp_cal->model_data[i].offset_temp, 3));

        i++;
        wait_timer = timestamp_nanos;   // Starts the wait timer.
        next_state = PRINT_MODEL_DATA;  // Sets the next state.
        debug_state = WAIT_STATE;       // First, go to wait state.
      } else {
        debug_state = IDLE;             // Goes to idle state.
        CAL_DEBUG_LOG(
            OVERTEMPCAL_REPORT_TAG, "Last Model Update [nsec] = %llu",
            (unsigned long long int)over_temp_cal->modelupdate_timestamp_nanos);
      }
      break;

    default:
      // Sends this state machine to its idle state.
      wait_timer = timestamp_nanos;  // Starts the wait timer.
      next_state = IDLE;             // Sets the next state.
      debug_state = WAIT_STATE;      // First, go to wait state.
  }
}

#endif  // OVERTEMPCAL_DBG_ENABLED
