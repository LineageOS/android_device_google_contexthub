#ifndef LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_GYROSCOPE_GYRO_OFFSET_OVER_TEMP_CAL_GYRO_OFFSET_OVER_TEMP_CAL_H_
#define LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_GYROSCOPE_GYRO_OFFSET_OVER_TEMP_CAL_GYRO_OFFSET_OVER_TEMP_CAL_H_

#include "calibration/gyroscope/gyro_cal.h"
#include "calibration/online_calibration/common_data/calibration_callback.h"
#include "calibration/online_calibration/common_data/calibration_data.h"
#include "calibration/online_calibration/common_data/online_calibration.h"
#include "calibration/online_calibration/common_data/sensor_data.h"
#include "calibration/over_temp/over_temp_cal.h"

namespace online_calibration {

/*
 * This class is a wrapper for the gyroscope offset calibration with
 * over-temperature compensation (OTC).
 */
class GyroOffsetOtcCal final
    : public OnlineCalibration<CalibrationDataThreeAxis> {
 public:
  // Default constructor.
  GyroOffsetOtcCal() = default;

  // Creates an GyroOffsetOtcCal with specified algorithm parameters.
  GyroOffsetOtcCal(const GyroCalParameters& gyro_cal_parameters,
                   const OverTempCalParameters& otc_parameters) {
    Initialize(gyro_cal_parameters, otc_parameters);
  }

  // Initializes with specified algorithm parameters, and resets the calibration
  // data.
  void Initialize(const GyroCalParameters& gyro_cal_parameters,
                  const OverTempCalParameters& otc_parameters);

  // Sends new sensor data to the calibration algorithm, and returns the state
  // of the calibration update flags, 'cal_update_polling_flags_'.
  CalibrationTypeFlags SetMeasurement(const SensorData& sample) final;

  // Sets the initial calibration data of the calibration algorithm. Returns
  // true if set successfully.
  bool SetInitialCalibration(
      const CalibrationDataThreeAxis& input_cal_data) final;

  // Returns the calibration sensor type.
  SensorType get_sensor_type() const final {
    return SensorType::kGyroscopeRps;
  };

 private:
  // GyroCal algorithm data structure.
  GyroCal gyro_cal_;

  // Over-temperature offset compensation algorithm data structure.
  OverTempCal over_temp_cal_;
};

}  // namespace online_calibration

#endif  // LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_GYROSCOPE_GYRO_OFFSET_OVER_TEMP_CAL_GYRO_OFFSET_OVER_TEMP_CAL_H_
