#include "calibration/synchronizer/synchronizer.h"

#include "calibration/util/cal_log.h"

namespace synchronizer {
namespace {

// Alias to simplify the access to SensorIndex.
using SensorIndex = online_calibration::SensorIndex;

// Seeks through data_to_interpolate_to and data_to_interpolate queues so that
// first two entries of data_to_interpolate have timestamps that straddle the
// timestamp of the first entry of data_to_interpolate_to. Leading queue items
// are popped and discarded to perform seeking.
//
// Assumes both data_to_interpolate_to and data_to_interpolate sorted by
// strictly increasing timestamp.
//
// Returns True if seeking was achieved. Returns false if seeking could not
// be achieved, such is if either pointer was null or they contain data
// streams which cannot be made to straddle in time.

bool SeekStraddle(
    chre::ArrayQueue<online_calibration::SensorData,
                     Synchronizer::kInputBufferSize>* data_to_interpolate_to,
    chre::ArrayQueue<online_calibration::SensorData,
                     Synchronizer::kInputBufferSize>* data_to_interpolate) {
  // If either is a null pointer, return false
  if (data_to_interpolate_to == nullptr || data_to_interpolate == nullptr) {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Unexpected null pointer during seek!");
#endif
    return false;
  }

  // If data_to_interpolate_to and data_to_interpolate are the same, no seeking.
  if (reinterpret_cast<void*>(data_to_interpolate_to) ==
      reinterpret_cast<void*>(data_to_interpolate)) {
    return true;
  }

  // Check that queues are not empty and time min/max aren't impossible.
  if (data_to_interpolate_to->empty() || data_to_interpolate->empty() ||
      data_to_interpolate->front().timestamp_nanos >
          data_to_interpolate_to->back().timestamp_nanos ||
      data_to_interpolate->back().timestamp_nanos <
          data_to_interpolate_to->front().timestamp_nanos) {
    return false;
  }

  // Seek in data_to_interpolate_to until first entry of data_to_interpolate
  // precedes first of data_to_interpolate_to.
  while (!data_to_interpolate_to->empty() &&
         data_to_interpolate->front().timestamp_nanos >
             data_to_interpolate_to->front().timestamp_nanos) {
    data_to_interpolate_to->pop();
  }

  // Check that data_to_interpolate_to still has at least one element.
  if (data_to_interpolate_to->empty()) {
    return false;
  }

  // Seek in data_to_interpolate until 1st and 2nd entries straddle first
  // data_to_interpolate_to entry.
  while (data_to_interpolate->size() > 1 &&
         !((*data_to_interpolate)[0].timestamp_nanos <=
               data_to_interpolate_to->front().timestamp_nanos &&
           (*data_to_interpolate)[1].timestamp_nanos >=
               data_to_interpolate_to->front().timestamp_nanos)) {
    data_to_interpolate->pop();
  }

  // data_to_interpolate must either match exactly, or have >=2 elements to
  // interpolate
  if (data_to_interpolate->front().timestamp_nanos ==
      data_to_interpolate_to->front().timestamp_nanos) {
    return true;
  } else if (data_to_interpolate->size() < 2) {
    return false;
  }

  // Check that data_to_interpolate still has at least two elements and they
  // straddle the first entry of data_to_interpolate_to.
  if ((*data_to_interpolate)[0].timestamp_nanos >
          data_to_interpolate_to->front().timestamp_nanos ||
      (*data_to_interpolate)[1].timestamp_nanos <
          data_to_interpolate_to->front().timestamp_nanos) {
    return false;
  }

  return true;
}
// Performs linear interpolation for a single axis.
float SingleAxisLinearInterpolation(float tb_minus_ta, float tc_minus_ta,
                                    float va, float vc) {
  return va + (vc - va) * (tb_minus_ta / tc_minus_ta);
}

// Interpolates data_to_interpolate using timestamps in
// data_to_interpolate_to, writing to interpolated the resulting 3-axis
// measurement vector and timestamp that matches a timestamp in
// data_to_interpolate_to. For this to be possible, data_to_interpolate
// must have at least two timestamps which straddle a timestamp in
// data_to_interpolate_to.
//
// The input queues are assumed to have strictly increasing timestamps.
//
// First, this uses SeekStraddle() to pop off entries of each queue until the
// first two entries of data_to_interpolate have timestamps that straddle the
// timestamp of the first entry of data_to_interpolate_to. Then linear
// interpolation proceeds and results are written to interpolated.
//
// Returns true if interpolation was possible. Returns false if any of the
// inputs are nullptr or if interpolation was not possible given the
// timestamps in the data queues.
bool SeekAndInterpolateLinear(
    chre::ArrayQueue<online_calibration::SensorData,
                     Synchronizer::kInputBufferSize>* data_to_interpolate_to,
    chre::ArrayQueue<online_calibration::SensorData,
                     Synchronizer::kInputBufferSize>* data_to_interpolate,
    online_calibration::SensorData* interpolated) {
  // Seek until first two entries of data_to_interpolate straddle first entry of
  // data_to_interpolate_to. Seek function also checks that
  // data_to_interpolate_to and data_to_interpolate are not nullptr.
  if (!SeekStraddle(data_to_interpolate_to, data_to_interpolate) ||
      interpolated == nullptr) {
    return false;
  }

  // Use some local variables to avoid repeated pointer dereferencing.
  // data_to_interpolate_to and data_to_interpolate queues guaranteed by seek
  // function to be non-empty.
  online_calibration::SensorData data_to_interpolate_to_0 =
      data_to_interpolate_to->front();
  const online_calibration::SensorData data_to_interpolate_0 =
      data_to_interpolate->front();
  online_calibration::SensorData data_to_interpolate_1;

  // Avoid floating point math if possible: check if timestamps match exactly.
  if (data_to_interpolate_to_0.timestamp_nanos ==
      data_to_interpolate_0.timestamp_nanos) {
    *interpolated = data_to_interpolate_0;
    return true;
  } else if (data_to_interpolate->size() > 1) {
    data_to_interpolate_1 = (*data_to_interpolate)[1];
    if (data_to_interpolate_to_0.timestamp_nanos ==
        data_to_interpolate_1.timestamp_nanos) {
      *interpolated = data_to_interpolate_1;
      return true;
    }
  } else {
// Size was 1 and first entries didn't match, so interpolation impossible!
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Unexpected queue state while attempting interpolation!");
#endif
    return false;
  }

  // Linearly interpolate data_to_interpolate at data_to_interpolate_to's 0th
  // entry timepoint. Interpolate vector vb between va, vc at time tb where
  // ta < tb < tc.
  const float tb_minus_ta =
      static_cast<float>(data_to_interpolate_to_0.timestamp_nanos -
                         data_to_interpolate_0.timestamp_nanos);
  const float tc_minus_ta =
      static_cast<float>(data_to_interpolate_1.timestamp_nanos -
                         data_to_interpolate_0.timestamp_nanos);

  if (data_to_interpolate_0.type ==
          online_calibration::SensorType::kAccelerometerMps2 ||
      data_to_interpolate_0.type ==
          online_calibration::SensorType::kGyroscopeRps ||
      data_to_interpolate_0.type ==
          online_calibration::SensorType::kMagnetometerUt) {
    // Perform linear interpolation for 3-axis sensor and populate the result
    // struct.
    interpolated->data[SensorIndex::kXAxis] = SingleAxisLinearInterpolation(
        tb_minus_ta, tc_minus_ta,
        data_to_interpolate_0.data[SensorIndex::kXAxis],
        data_to_interpolate_1.data[SensorIndex::kXAxis]);
    interpolated->data[SensorIndex::kYAxis] = SingleAxisLinearInterpolation(
        tb_minus_ta, tc_minus_ta,
        data_to_interpolate_0.data[SensorIndex::kYAxis],
        data_to_interpolate_1.data[SensorIndex::kYAxis]);
    interpolated->data[SensorIndex::kZAxis] = SingleAxisLinearInterpolation(
        tb_minus_ta, tc_minus_ta,
        data_to_interpolate_0.data[SensorIndex::kZAxis],
        data_to_interpolate_1.data[SensorIndex::kZAxis]);
    interpolated->timestamp_nanos = data_to_interpolate_to_0.timestamp_nanos;
    interpolated->type = data_to_interpolate_0.type;
    return true;
  } else if (data_to_interpolate_0.type ==
                 online_calibration::SensorType::kTemperatureCelsius ||
             data_to_interpolate_0.type ==
                 online_calibration::SensorType::kBarometerHpa) {
    // Perform linear interpolation for 1-axis sensor and populate the result
    // struct.
    interpolated->data[SensorIndex::kSingleAxis] =
        SingleAxisLinearInterpolation(
            tb_minus_ta, tc_minus_ta,
            data_to_interpolate_0.data[SensorIndex::kSingleAxis],
            data_to_interpolate_1.data[SensorIndex::kSingleAxis]);
    interpolated->timestamp_nanos = data_to_interpolate_to_0.timestamp_nanos;
    interpolated->type = data_to_interpolate_0.type;
    return true;
  } else {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Invalid sensor_type in SeekAndInterpolateLinear");
#endif
    return false;
  }
}

// Attempts to add a SensorData measurement to a deque of them, returning
// whether the new measurement was strictly greater in time than the last
// entry in the queue. If it was not, measurement is not added and false is
// returned. This function is primarily for code compactness: several queues
// of measurements are appended-to in this same manner.
bool AddMeasurementToDeque(
    const online_calibration::SensorData& measurement,
    chre::ArrayQueue<online_calibration::SensorData,
                     Synchronizer::kInputBufferSize>* mutable_deque) {
  // Push measurement onto the measurement queue if sequential.
  if (mutable_deque->empty() ||
      (!mutable_deque->empty() &&
       measurement.timestamp_nanos > mutable_deque->back().timestamp_nanos)) {
    mutable_deque->push(measurement);
    return true;
  } else {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG(
        "[SYNCHRONIZER WARNING]",
        "measurement timestamp: %lu was not strictly greater than existing"
        "queued timestamps.",
        measurement.timestamp_nanos);
#endif
    return false;
  }
}
}  // namespace

Synchronizer::Synchronizer()
    : Synchronizer(online_calibration::SensorType::kGyroscopeRps,
                   online_calibration::SensorType::kAccelerometerMps2) {}

Synchronizer::Synchronizer(
    const online_calibration::SensorType& sensor_to_interpolate_to,
    const online_calibration::SensorType& sensor_to_interpolate) {
  if (sensor_to_interpolate_to == sensor_to_interpolate) {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Sensor_types are equal! Using default settings!");
#endif
    sensor_to_interpolate_to_ = online_calibration::SensorType::kGyroscopeRps;
    sensor_to_interpolate_ = online_calibration::SensorType::kAccelerometerMps2;
  } else {
    // Setting sensors.
    sensor_to_interpolate_to_ = sensor_to_interpolate_to;
    sensor_to_interpolate_ = sensor_to_interpolate;
  }
}

bool Synchronizer::SetDataToInterpolateToSensorType(
    const online_calibration::SensorType& sensor_type) {
  if (sensor_type == sensor_to_interpolate_) {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Sensor_type in SetDataToInterpolateToSensorType is "
                  "already used!");
#endif
    return false;
  }
  if (sensor_type == online_calibration::SensorType::kUndefined) {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Invalid sensor_type in SetDataToInterpolateToSensorType");
#endif
    return false;
  }
  sensor_to_interpolate_to_ = sensor_type;
  return true;
}

bool Synchronizer::SetDataToInterpolateSensorType(
    const online_calibration::SensorType& sensor_type) {
  if (sensor_type == sensor_to_interpolate_to_) {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG(
        "[SYNCHRONIZER WARNING]",
        "Sensor_type in SetDataToInterpolateSensorType is already used!");
#endif
    return false;
  }
  if (sensor_type == online_calibration::SensorType::kUndefined) {
#ifdef SYNCHRONIZER_DBG_ENABLED
    CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                  "Invalid sensor_type in SetDataToInterpolateSensorType");
#endif
    return false;
  }
  sensor_to_interpolate_ = sensor_type;
  return true;
}

void Synchronizer::AddMeasurement(const online_calibration::SensorData& meas) {
  online_calibration::SensorType sensor_type = meas.type;
  if (sensor_type == sensor_to_interpolate_to_) {
    if (input_sensor_to_interpolate_to_.full()) {
      CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                    "input_sensor_to_interpolate_to queue is full! "
                    "Removing front element from queue");
      input_sensor_to_interpolate_to_.pop();
    }
    if (AddMeasurementToDeque(meas, &input_sensor_to_interpolate_to_)) {
      // Add succeeded. Process applicable measurement queue.
      ProcessMeasurementQueues();
    }
  }
  if (sensor_type == sensor_to_interpolate_) {
    if (input_sensor_to_interpolate_.full()) {
      CAL_DEBUG_LOG("[SYNCHRONIZER WARNING]",
                    "input_sensor_to_interpolate queue is full !"
                    "Removing front element from queue");
      input_sensor_to_interpolate_.pop();
    }
    if (AddMeasurementToDeque(meas, &input_sensor_to_interpolate_)) {
      // Add succeeded. Process applicable measurement queue.
      ProcessMeasurementQueues();
    }
  }
}

bool Synchronizer::ConsumeSynchronizedMeasurementQueues(
    /*first sensor*/ online_calibration::SensorData* sensor_to_interpolate_to,
    /*second sensor*/ online_calibration::SensorData* sensor_to_interpolate) {
  if (!synchronized_first_and_second_queues_.empty()) {
    *sensor_to_interpolate_to =
        synchronized_first_and_second_queues_.front().first;
    *sensor_to_interpolate =
        synchronized_first_and_second_queues_.front().second;
    synchronized_first_and_second_queues_.pop();
    return true;
  }
  return false;
}

void Synchronizer::ProcessMeasurementQueues() {
  // Results of interpolation written here.
  online_calibration::SensorData interpolated;

  // Process the queues, interpolating and queuing up measurements.
  while (SeekAndInterpolateLinear(&input_sensor_to_interpolate_to_,
                                  &input_sensor_to_interpolate_,
                                  &interpolated)) {
    synchronized_first_and_second_queues_.push(
        std::pair<online_calibration::SensorData,
                  online_calibration::SensorData>(
            input_sensor_to_interpolate_to_.front(), interpolated));

    // Pop off data_to_interpolate_to measurement used to create this sample.
    // data_to_interpolate will be cleaned up next seek.
    input_sensor_to_interpolate_to_.pop();
  }
}

}  // namespace synchronizer
