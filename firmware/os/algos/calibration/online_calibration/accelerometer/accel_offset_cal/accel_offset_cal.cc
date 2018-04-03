#include "calibration/online_calibration/accelerometer/accel_offset_cal/accel_offset_cal.h"

#include "calibration/util/cal_log.h"

namespace online_calibration {

void AccelOffsetCal::Initialize(
    const AccelCalParameters& accel_cal_parameters) {
  accelCalInit(&accel_cal_, &accel_cal_parameters);
  InitializeCalData();
}

CalibrationTypeFlags AccelOffsetCal::SetMeasurement(const SensorData& sample) {
  // Routes the input sensor sample to the calibration algorithm.
  switch (sample.type) {
    case SensorType::kAccelerometerMps2:
      accelCalRun(&accel_cal_, sample.timestamp_nanos,
                  sample.data[SensorIndex::kXAxis],
                  sample.data[SensorIndex::kYAxis],
                  sample.data[SensorIndex::kZAxis],  // [m/sec^2]
                  temperature_celsius_);

#ifdef ACCEL_CAL_DBG_ENABLED
      // Prints debug data report.
      accelCalDebPrint(&accel_cal_, temperature_celsius_);
#endif
      break;

    case SensorType::kTemperatureCelsius:
      temperature_celsius_ = sample.data[SensorIndex::kSingleAxis];
      break;

    default:
      // This sample is not required.
      return cal_update_polling_flags_;
  }

  // Checks for a new offset estimate, and updates the calibration data.
  if (accelCalNewBiasAvailable(&accel_cal_)) {
    accelCalUpdateBias(&accel_cal_, &cal_data_.offset[0], &cal_data_.offset[1],
                       &cal_data_.offset[2]);

    cal_data_.offset_temp_celsius = temperature_celsius_;
    cal_data_.cal_update_time_nanos = sample.timestamp_nanos;
    cal_update_polling_flags_ = CalibrationTypeFlags::BIAS;
    OnNotifyCalibrationUpdate(CalibrationTypeFlags::BIAS);
  }

  return cal_update_polling_flags_;
}

bool AccelOffsetCal::SetInitialCalibration(
    const CalibrationDataThreeAxis& input_cal_data) {
  // Checks that the input calibration type matches the algorithm type.
  if (input_cal_data.type != get_sensor_type()) {
    CAL_DEBUG_LOG("[AccelOffsetCal]",
                  "SetInitialCalibration failed due to wrong sensor type.");
    return false;
  }

  // Sets the accelerometer algorithm's calibration data.
  accelCalBiasSet(&accel_cal_, input_cal_data.offset[0],
                  input_cal_data.offset[1], input_cal_data.offset[2]);

  // Sync's all initial calibration data.
  cal_data_ = input_cal_data;

  return true;
}

}  // namespace online_calibration
