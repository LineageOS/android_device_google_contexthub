/*
 * This is a sensor data synchronizer module that interpolates buffers (queues)
 * of streamed measurements with respect to a user-selected input master stream
 * to set the timebase. To perform synchronization, a sensor (sensor_data.h)
 * is selected to define the resulting synchronized time axis. The second
 * selected sensor is interpolated to the first sensor's timescale. This
 * produces a synchronized queue of measurement pairs.
 */

#ifndef LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_SYNCHRONIZER_SYNCHRONIZER_H_
#define LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_SYNCHRONIZER_SYNCHRONIZER_H_

#include "calibration/online_calibration/common_data/sensor_data.h"
#include "chre/util/array_queue.h"

namespace synchronizer {

/*
 * Buffers measurements and synchronizes them using interpolation.
 *
 * Processes input streams of sensors and queues up output streams.
 */
class Synchronizer {
 public:
  // Input buffer size = 400 gives 1 second of buffer for a input sensor data
  // rate of 400 Hz.
  static constexpr int kInputBufferSize = 400;
  // The output buffer size can store 10 sample, hist code has to ensure to
  // grab the data before overflow.
  static constexpr int kOutputBufferSize = 10;

  Synchronizer();

  Synchronizer(const online_calibration::SensorType& sensor_to_interpolate_to,
               const online_calibration::SensorType& sensor_to_interpolate);

  // Sets the sensor (first) to which the second sensor is interpolated to.
  // Hence this sensor timebase is the master of the synchronized data stream.
  // Notice that "sensor_to_interpolate_to_" cannot be equal
  // "sensor_to_interpolate_", so this function cannot synchronize two accel's
  // to each other.
  // TODO(b/70861106): Change interface to a master slave approach.
  // Sensors available (sensor_data.h).
  bool SetDataToInterpolateToSensorType(
      const online_calibration::SensorType& sensor_type);

  // Sets the sensor (second) which will be interpolated. Notice that
  // "sensor_to_interpolate_to_" cannot be equal "sensor_to_interpolate_".
  // Sensors available (sensor_data.h).
  bool SetDataToInterpolateSensorType(
      const online_calibration::SensorType& sensor_type);

  // Adds measurements. Uses the type in SensorData to identify
  // sensor_to_interpolate and sensor_to_interpolate_to sensors.
  void AddMeasurement(const online_calibration::SensorData& meas);

  // Processes the synchronization queues in chronological order. Processing
  // involves popping off each queue element and passing it to the specified
  // function target of corresponding sensor type.
  //
  // Returns true if at least one element is in the sensor queue, and populates
  // the synchronzied sensor data structs.
  bool ConsumeSynchronizedMeasurementQueues(
      /*sensor_to_interpolate_to*/
      online_calibration::SensorData* sensor_to_interpolate_to,
      /*sensor_to_interpolate*/
      online_calibration::SensorData* sensor_to_interpolate);

 private:
  // Attempts to pair up "sensor_to_interpolate_to" (first) and
  // "sensor_to_interpolate" (second)  measurements buffered by the
  // AddMeasurement() function, interpolating one using the other as a
  // data_to_interpolate_to timescale if necessary.
  //
  // If interpolated measurements are generated, they are queued in this
  // function.

  void ProcessMeasurementQueues();

  // Queue of synchronized measurements.
  chre::ArrayQueue<
      std::pair<online_calibration::SensorData, online_calibration::SensorData>,
      kOutputBufferSize>
      synchronized_first_and_second_queues_;

  // Enums set which measurement defines the data_to_interpolate_to (first)
  // timebase, against which sensor_to_interpolate is interpolated (second).
  online_calibration::SensorType sensor_to_interpolate_to_;
  online_calibration::SensorType sensor_to_interpolate_;

  // Queues of time-ordered measurements for interpolation within this class.
  chre::ArrayQueue<online_calibration::SensorData, kInputBufferSize>
      input_sensor_to_interpolate_to_;
  chre::ArrayQueue<online_calibration::SensorData, kInputBufferSize>
      input_sensor_to_interpolate_;
};

}  // namespace synchronizer

#endif  // LOCATION_LBS_CONTEXTHUB_NANOAPPS_CALIBRATION_SYNCHRONIZER_SYNCHRONIZER_H_
