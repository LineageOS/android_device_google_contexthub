#include "calibration/magnetometer/mag_gyro_cal.h"

#include <string.h>

#include "calibration/util/cal_log.h"
#include "common/math/macros.h"

#define SINGULAR_TOL (1e-4f)

static bool allDiagonalsLessThan(struct Mat33 *matrix, float threshold) {
  return matrix->elem[0][0] < threshold && matrix->elem[1][1] < threshold &&
         matrix->elem[2][2] < threshold;
}

static void calculatingAndSettingBias(struct MagGyroCal *mgc,
                                      struct Mat33 *first_sum_inverse,
                                      struct Vec3 *bias) {
  // Calculate bias, since we have high trust into this estimates.
  mat33Apply(bias, first_sum_inverse, &mgc->mgac.second_sum_average);

  // Setting bias update.
  mgc->x_bias = bias->x;
  mgc->y_bias = bias->y;
  mgc->z_bias = bias->z;
}

void magGyroCalBatchReset(struct MagGyroBatchCal *mgbc) {
  mgbc->first_sample = true;
  mgbc->start_time_ns = 0;
  mgbc->timestamp_buffer_ns = 0;
  initZeroMatrix(&mgbc->first_sum_update);
  initVec3(&mgbc->last_mag_sample, 0.0f, 0.0f, 0.0f);
  initVec3(&mgbc->second_sum_update, 0.0f, 0.0f, 0.0f);
}

static void magGyroCalAverageReset(struct MagGyroAverageCal *mgac) {
  initZeroMatrix(&mgac->first_sum_average);
  initVec3(&mgac->second_sum_average, 0.0f, 0.0f, 0.0f);
  initVec3(&mgac->confidence_metric, 0.0f, 0.0f, 0.0f);
  mgac->medium_accuracy_bias_update = false;
  initVec3(&mgac->bias_running, 0.0f, 0.0f, 0.0f);
}

void magGyroCalReset(struct MagGyroCal *mgc) {
  // Reset Batch Cal.
  magGyroCalBatchReset(&mgc->mgbc);

  // Reset Avarge Cal.
  magGyroCalAverageReset(&mgc->mgac);

  initVec3(&mgc->mgac.confidence_metric,
           mgc->algo_parameters.confidence_threshold_start,
           mgc->algo_parameters.confidence_threshold_start,
           mgc->algo_parameters.confidence_threshold_start);
}

void magGyroCalGetBias(struct MagGyroCal *mgc, float *x, float *y, float *z) {
  *x = mgc->x_bias;
  *y = mgc->y_bias;
  *z = mgc->z_bias;
}

void magGyroCalInit(struct MagGyroCal *mgc,
                    const struct MagGyroCalParameters *params) {
  // Tuning parameters.
  memcpy(&mgc->algo_parameters, params, sizeof(mgc->algo_parameters));

  // Bias Updates set to zero.
  mgc->x_bias = 0.0f;
  mgc->y_bias = 0.0f;
  mgc->z_bias = 0.0f;

  // Reseting Algo.
  magGyroCalReset(mgc);
}

void magGyroCalBatchUpdate(struct MagGyroBatchCal *mgbc,
                           uint64_t sample_time_ns, float mag_x_ut,
                           float mag_y_ut, float mag_z_ut, float gyro_x_rps,
                           float gyro_y_rps, float gyro_z_rps) {
  // local variables, only needed for math on single samples.
  struct Vec3 mag_data;
  struct Vec3 gyro_data;
  struct Vec3 mag_data_dot;
  struct Vec3 cross_mag_gyro;
  struct Vec3 second_sum;
  struct Vec3 yi;
  struct Mat33 first_sum;
  struct Mat33 w;
  struct Mat33 w_trans;
  float odr_hz_int;
  // Creating gyro and mag vector.
  initVec3(&mag_data, mag_x_ut, mag_y_ut, mag_z_ut);
  initVec3(&gyro_data, gyro_x_rps, gyro_y_rps, gyro_z_rps);

  // Saving first sample in a batch and skipping the rest, this is done in order
  // to do the discrete time derivative. x_dot = x(n) - x(n-1).
  if (mgbc->first_sample) {
    initVec3(&mgbc->last_mag_sample, mag_x_ut, mag_y_ut, mag_z_ut);
    mgbc->first_sample = false;
    mgbc->start_time_ns = sample_time_ns;
    mgbc->timestamp_buffer_ns = sample_time_ns;
    return;
  }
  // checking if delta time can become zero or negative.
  if (sample_time_ns <= mgbc->timestamp_buffer_ns) {
    // Timestamp is not monotonic, resetting batch.
    magGyroCalBatchReset(mgbc);
    return;
  }

  // Calculating time delta between this sample and the sample before.
  float delta_time_ns = (float)(sample_time_ns - mgbc->timestamp_buffer_ns);

  // Calculating 1/deltaT = ODR [Hz], for time derivative.
  odr_hz_int = 1 / (delta_time_ns * NANOS_TO_SEC);
  mgbc->timestamp_buffer_ns = sample_time_ns;

  // creating mag_dot.
  // mag_data_dot =  (mag_data - last_mag_sample) * odr.
  vec3SubVecs(&mag_data_dot, &mag_data, &mgbc->last_mag_sample);
  vec3ScalarMul(&mag_data_dot, odr_hz_int);

  // Cross product (gyro x mag).
  vec3Cross(&cross_mag_gyro, &gyro_data, &mag_data);

  // Saving data into memory, for next derivative.
  initVec3(&mgbc->last_mag_sample, mag_x_ut, mag_y_ut, mag_z_ut);

  // calculating yi = mag_dot + (gyro x mag).
  vec3AddVecs(&yi, &mag_data_dot, &cross_mag_gyro);

  // creating skew-symmetric matrix.
  //     /   0, -w3,  w2 \.
  // w = |  w3,   0, -w1 |.
  //     \ -w2,  w1,  0  /.
  w.elem[0][0] = 0;
  w.elem[0][1] = -gyro_z_rps;
  w.elem[0][2] = gyro_y_rps;
  w.elem[1][0] = gyro_z_rps;
  w.elem[1][1] = 0;
  w.elem[1][2] = -gyro_x_rps;
  w.elem[2][0] = -gyro_y_rps;
  w.elem[2][1] = gyro_x_rps;
  w.elem[2][2] = 0;

  // calculating second_sum:
  // second_sum = w^T * yi.
  mat33Transpose(&w_trans, &w);
  mat33Apply(&second_sum, &w_trans, &yi);

  // calculating first sum:
  // first_sum = w^T * w.
  mat33MultiplyTransposed(&first_sum, &w, &w);

  // Updating sums.
  mat33Add(&mgbc->first_sum_update, &first_sum);
  vec3Add(&mgbc->second_sum_update, &second_sum);
}

enum MagUpdate magGyroCalBiasUpdate(struct MagGyroCal *mgc,
                                    struct Mat33 *first_sum_inverse) {
  struct Vec3 bias;
  // Testing for medium accuracy.
  if (allDiagonalsLessThan(first_sum_inverse,
                           mgc->algo_parameters.confidence_threshold_medium)) {
    // Testing for high accuracy, will leave the function here if true.
    if (allDiagonalsLessThan(first_sum_inverse,
                             mgc->algo_parameters.confidence_threshold_high)) {
      // Calculate bias, since we have high trust into this estimates.
      calculatingAndSettingBias(mgc, first_sum_inverse, &bias);

      // Resetting everything since we have found the bias.
      magGyroCalReset(mgc);

      // Return high accuracy indicator.
      return UPDATE_BIAS_MAGGYRO_HIGH;
    }
    // Only will reach this point if accuracy is between high and medium, now
    // we test for if we already have a medium update.
    if (mgc->mgac.medium_accuracy_bias_update == false) {
      // Calculate bias, since we have high trust into the estimates.
      calculatingAndSettingBias(mgc, first_sum_inverse, &bias);

      // Resetting Batch, since we are not done yet.
      magGyroCalBatchReset(&mgc->mgbc);

      // Setting medium_accuracy_bias_update to "true".
      mgc->mgac.medium_accuracy_bias_update = true;

      // Return medium accuracy indicator.
      return UPDATE_BIAS_MAGGYRO_MEDIUM;

      // already have an medium update, still calculate the actual offset, can
      // be used for log files and manual bias extractions.
    } else {
      // Calculate bias, since we have high trust into the estimates.
      mat33Apply(&bias, first_sum_inverse, &mgc->mgac.second_sum_average);

      // Setting bias update.
      mgc->mgac.bias_running = bias;

      // Resetting Batch, since we are not done yet.
      magGyroCalBatchReset(&mgc->mgbc);

      // Return no update indicator.
      return NO_UPDATE;
    }
  }
  // Have not gotten to medium accuracy, reset the batch.
  magGyroCalBatchReset(&mgc->mgbc);

  // Return no update indicator.
  return NO_UPDATE;
}

enum MagUpdate magGyroCalRun(struct MagGyroCal *mgc, uint64_t sample_time_ns,
                             float mag_x_ut, float mag_y_ut, float mag_z_ut,
                             float gyro_x_rps, float gyro_y_rps,
                             float gyro_z_rps) {
  struct Mat33 first_sum_inverse;
  // Running samples into the batch function.
  magGyroCalBatchUpdate(&mgc->mgbc, sample_time_ns, mag_x_ut, mag_y_ut,
                        mag_z_ut, gyro_x_rps, gyro_y_rps, gyro_z_rps);

  // Timeout, reset and start over, return MAGGYRO_TIMEOUT.
  if ((sample_time_ns - mgc->mgbc.start_time_ns) >=
      mgc->algo_parameters.max_run_time_in_ns) {
    magGyroCalReset(mgc);
    return MAGGYRO_TIMEOUT;
  }

  // Early return if batch not complete.
  if ((sample_time_ns - mgc->mgbc.start_time_ns) <=
      mgc->algo_parameters.batch_window_in_ns) {
    return NO_UPDATE;
  }

  // Updating the average matrix.
  mat33Add(&mgc->mgac.first_sum_average, &mgc->mgbc.first_sum_update);

  // Testing for singular matrix.
  if (fabsf(mat33Determinant(&mgc->mgac.first_sum_average)) > SINGULAR_TOL) {
    // Inverse of the average system matrix.
    mat33Invert(&first_sum_inverse, &mgc->mgac.first_sum_average);

    // Testing if the confidence metric has been improved.
    if (first_sum_inverse.elem[0][0] < mgc->mgac.confidence_metric.x &&
        first_sum_inverse.elem[1][1] < mgc->mgac.confidence_metric.y &&
        first_sum_inverse.elem[2][2] < mgc->mgac.confidence_metric.z) {
      // Updating the second_sum vector.
      vec3Add(&mgc->mgac.second_sum_average, &mgc->mgbc.second_sum_update);

      // Updating the confidence metric.
      initVec3(&mgc->mgac.confidence_metric, first_sum_inverse.elem[0][0],
               first_sum_inverse.elem[1][1], first_sum_inverse.elem[2][2]);

      // Testing for Bias update and leaving the function.
      return magGyroCalBiasUpdate(mgc, &first_sum_inverse);
    }
    // No improvement, removing the last batch from the average.
    mat33Sub(&mgc->mgac.first_sum_average, &mgc->mgbc.first_sum_update);

    // Resetting Batch, since we are not done yet.
    magGyroCalBatchReset(&mgc->mgbc);

    // Returning no bias update.
    return NO_UPDATE;
  }
  return NO_UPDATE;
}
