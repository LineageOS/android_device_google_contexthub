#ifndef LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_MAGNETOMETER_MAG_DIVERSE_CAL_MAG_DIVERSE_CAL_H_
#define LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_MAGNETOMETER_MAG_DIVERSE_CAL_MAG_DIVERSE_CAL_H_

#include "calibration/diversity_checker/diversity_checker.h"
#include "calibration/magnetometer/mag_cal.h"
#include "calibration/online_calibration/common_data/calibration_callback.h"
#include "calibration/online_calibration/common_data/calibration_data.h"
#include "calibration/online_calibration/common_data/online_calibration.h"
#include "calibration/online_calibration/common_data/sensor_data.h"

namespace online_calibration {

/*
 * This class is a wrapper for the magnetometer offset calibration with
 * diversity checking.
 */
class MagDiverseCal final : public OnlineCalibration<CalibrationDataThreeAxis> {
 public:
  MagDiverseCal() = default;

  // Creates an MagDiverseCal with specified algorithm parameters.
  MagDiverseCal(const MagCalParameters& mag_cal_parameters,
                const DiversityCheckerParameters& diversity_parameters) {
    Initialize(mag_cal_parameters, diversity_parameters);
  }

  // Initializes with specified algorithm parameters.
  void Initialize(const MagCalParameters& mag_cal_parameters,
                  const DiversityCheckerParameters& diversity_parameters);

  // Sends new sensor data to the calibration algorithm, and returns the state
  // of the calibration update flags, 'cal_update_polling_flags_'.
  CalibrationTypeFlags SetMeasurement(const SensorData& sample) final;

  // Sets the initial calibration data of the calibration algorithm. Returns
  // true if set successfully.
  bool SetInitialCalibration(
      const CalibrationDataThreeAxis& input_cal_data) final;

  // Returns the calibration sensor type.
  SensorType get_sensor_type() const final {
    return SensorType::kMagnetometerUt;
  };

 private:
  // MagCal algorithm data structure.
  MagCal mag_cal_;
};

}  // namespace online_calibration

#endif  // LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_ONLINE_CALIBRATION_MAGNETOMETER_MAG_DIVERSE_CAL_MAG_DIVERSE_CAL_H_
