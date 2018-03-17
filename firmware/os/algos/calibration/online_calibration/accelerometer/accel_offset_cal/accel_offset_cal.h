#ifndef LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_ACCELEROMETER_ACCEL_OFFSET_CAL_ACCEL_OFFSET_CAL_H_
#define LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_ACCELEROMETER_ACCEL_OFFSET_CAL_ACCEL_OFFSET_CAL_H_

#include "calibration/accelerometer/accel_cal.h"
#include "calibration/online_calibration/common_data/calibration_callback.h"
#include "calibration/online_calibration/common_data/calibration_data.h"
#include "calibration/online_calibration/common_data/online_calibration.h"
#include "calibration/online_calibration/common_data/sensor_data.h"

namespace online_calibration {

/*
 * This class is a wrapper for accelerometer offset calibration.
 */
class AccelOffsetCal final
    : public OnlineCalibration<CalibrationDataThreeAxis> {
 public:
  // Default constructor.
  AccelOffsetCal() = default;

  // Creates an AccelOffsetCal with specified algorithm parameters.
  explicit AccelOffsetCal(const AccelCalParameters& accel_cal_parameters) {
    Initialize(accel_cal_parameters);
  }

  // Initializes with specified algorithm parameters, and resets the calibration
  // data.
  void Initialize(const AccelCalParameters& accel_cal_parameters);

  // Sends new sensor data to the calibration algorithm, and returns the state
  // of the calibration update flags, 'cal_update_polling_flags_'.
  CalibrationTypeFlags SetMeasurement(const SensorData& sample) final;

  // Sets the initial calibration data of the calibration algorithm. Returns
  // true if set successfully.
  bool SetInitialCalibration(
      const CalibrationDataThreeAxis& input_cal_data) final;

  // Returns the calibration sensor type.
  SensorType get_sensor_type() const final {
    return SensorType::kAccelerometerMps2;
  };

 private:
  // Accelerometer offset calibration algorithm data structure.
  AccelCal accel_cal_;
};

}  // namespace online_calibration

#endif  // LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_ACCELEROMETER_ACCEL_OFFSET_CAL_ACCEL_OFFSET_CAL_H_
