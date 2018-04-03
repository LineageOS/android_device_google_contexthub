/*
 * This module contains the algorithms for producing a magnetometer offset
 * calibration. The algorithm is based on the paper “Adaptive Estimation of
 * Measurement Bias in Three-Dimensional Field Sensors with Angular-Rate
 * Sensors: Theory and Comparative Experimental Evaluation” by G. Troni and L.
 * Whitcomb and uses gyroscope data as well as magnetometer data. We use a batch
 * method, resulting in a Linear Least-Squares Fit (Section III B of the paper).
 * We define the following mathematical setup:
 *
 * wi (wx,wy,wx) -> are the gyroscope readings [RAD/s]
 * xi            -> are the magnetometer readings [uT]
 * xi_dot        -> is the derivative of xi
 *
 * Form here we can define:
 *
 *       yi = xi_dot + (wi x xi) (x denotes the cross product)
 *
 * and
 *
 *           /  0   -wz   wy \
 *       Wi =|  wz   0    -wx |
 *           \ -wy  wx     0  /
 *
 * which we can use to define, what we call first sum and second sum
 *
 *       firstSum = sum( Wi^2)
 *
 * and
 *
 *       secondSum = sum(Wi yi)
 *
 * The bias is calculated the following way
 *
 *       b = inverse(firstSum) * secondSum
 *
 * Notice that we use the diagonal of the matrix “inverse(firstSum)” as a
 * confidence metric, to make a accuracy assessment.
 *
 * The algorithm expects time synchronized magnetometer and gyro data, any delay
 * shift between those two greater than 5-10ms will result in poor accuracy.
 * Furthermore, the minimum sample rate of the sensors should be higher than 50
 * Hz.
 */

#ifndef LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_MAGNETOMETER_MAG_GYRO_CAL_H_
#define LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_MAGNETOMETER_MAG_GYRO_CAL_H_

#ifdef SPHERE_FIT_ENABLED
#ifndef DIVERSITY_CHECK_ENABLED
#define DIVERSITY_CHECK_ENABLED
#endif
#endif

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>

#include "calibration/magnetometer/mag_cal.h"
#include "common/math/mat.h"
#include "common/math/vec.h"

#ifdef __cplusplus
extern "C" {
#endif

// Algo Batch Data Struct.
struct MagGyroBatchCal {
  // Batch Start time.
  uint64_t start_time_ns;

  // Time Stamp Buffer. Variable used to calculate 1/deltaT = odr_in_hz, where
  // 1/deltaT is used to calculate the derivative of the mag signal.
  uint64_t timestamp_buffer_ns;

  // First sample in batch indicator.
  bool first_sample;

  // Matrix and vector definition.
  struct Mat33 first_sum_update;
  struct Vec3 last_mag_sample;
  struct Vec3 second_sum_update;
};

// Algorithm tuning Parameter struct.
struct MagGyroCalParameters {
  // Batch window size.
  uint64_t batch_window_in_ns;

  // Algo resets after max_run_time.
  uint64_t max_run_time_in_ns;

  // Confidence Threshold.
  // The confidence metric is explained in go/magcalp design document.
  // Furthermore more details are given as well on how to tune it. Confidence
  // Threshold high accuracy.
  float confidence_threshold_high;

  // Confidence Threshold medium accuracy.
  float confidence_threshold_medium;

  // Confidence Threshold start. Don't want to start with a bad matrix.
  float confidence_threshold_start;
};

// Data Struct for the Batch average function.
struct MagGyroAverageCal {
  // Matrix and vector definition.
  struct Mat33 first_sum_average;
  struct Vec3 second_sum_average;

  // Medium accuracy bias update.
  bool medium_accuracy_bias_update;

  // Confidence Metric.
  struct Vec3 confidence_metric;

  // Running biases update.
  struct Vec3 bias_running;
};

// Main algo data Struct.
struct MagGyroCal {
  // Algorithm tuning parameters.
  struct MagGyroCalParameters algo_parameters;

  // Single batch struct. We use batches, which in turn get again averaged over
  // time. The single batch is the amount of data that we do the Linear
  // Least-Squares fit on and decide if we will add this to the averaging based
  // on the confidence metric. Notice that we do on every batch a matrix
  // inversion, hence in order to save power it is beneficial to make the batch
  // size reasonably large. This is controlled by the “batch_window_in_ns”.
  struct MagGyroBatchCal mgbc;

  // Algo average struct.
  struct MagGyroAverageCal mgac;

  // New bias update.
  float x_bias;
  float y_bias;
  float z_bias;
};

// Algorithm initialization.
void magGyroCalInit(struct MagGyroCal *mgc,
                    const struct MagGyroCalParameters *params);

// Resets the batch struct.
void magGyroCalReset(struct MagGyroCal *mgc);

// Gets the current MagGyro bias estimate.
void magGyroCalGetBias(struct MagGyroCal *mgc, float *x, float *y, float *z);

// Main algorithm runner function.
// sample_time_ns in [ns].
// mag_x_ut, mag_y_ut, mag_z_ut are in [uT].
// gyro_x_rps, gyro_y_rps, gyro_z_rps are in [RAD/s].
// odr_hz is in [Hz].
enum MagUpdate magGyroCalRun(struct MagGyroCal *mgc, uint64_t sample_time_ns,
                             float mag_x_ut, float mag_y_ut, float mag_z_ut,
                             float gyro_x_rps, float gyro_y_rps,
                             float gyro_z_rps);

//////// Function exposed for testing ///////////////////////////////////
// Updating sensor data into the batch.
// sample_time_ns in [ns].
// mag_x_ut, mag_y_ut, mag_z_ut are in [uT].
// gyro_x_rps, gyro_y_rps, gyro_z_rps are in [RAD/s].
// odr_hz is in [Hz].
void magGyroCalBatchUpdate(struct MagGyroBatchCal *mgbc,
                           uint64_t sample_time_ns, float mag_x_ut,
                           float mag_y_ut, float mag_z_ut, float gyro_x_rps,
                           float gyro_y_rps, float gyro_z_rps);

// Checks metrics and classifies bias update into either:
// NO_UPDATE and no bias is updated
// UPDATE_BIAS_MEDIUM bias is update with medium accuracy.
// UPDATE_BIAS_HIGH bias is update with high accuracy.
enum MagUpdate magGyroCalBiasUpdate(struct MagGyroCal *mgc,
                                    struct Mat33 *first_sum_inverse);

// Resetting the batch struct.
void magGyroCalBatchReset(struct MagGyroBatchCal *mgbc);

#ifdef __cplusplus
}
#endif

#endif  // LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_MAGNETOMETER_MAG_GYRO_CAL_H_
