#ifndef LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_OVER_TEMP_OVER_TEMP_MODEL_H_
#define LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_OVER_TEMP_OVER_TEMP_MODEL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Over-temperature data structures that contain a modeled sensor offset
 * estimate, an associated temperature, and the age of the data point since it
 * first entered the model.
 */

struct OverTempModelThreeAxis {
  // Sensor offset estimate, temperature, and age timestamp.
  uint64_t offset_age_nanos;  // [nanoseconds]
  float offset_temp_celsius;  // [Celsius]

  // Three-axis offset estimate (indices: 0=x, 1=y, 2=z).
  float offset[3];
};

struct OverTempModelSingleAxis {
  // Sensor offset estimate, temperature, and age timestamp.
  uint64_t offset_age_nanos;  // [nanoseconds]
  float offset_temp_celsius;  // [Celsius]

  // Single-axis offset estimate.
  float offset;
};

#ifdef __cplusplus
}
#endif

#endif  // LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_OVER_TEMP_OVER_TEMP_MODEL_H_
