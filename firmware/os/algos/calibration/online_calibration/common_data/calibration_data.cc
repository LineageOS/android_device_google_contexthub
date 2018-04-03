#include "calibration/online_calibration/common_data/calibration_data.h"

namespace online_calibration {

CalibrationTypeFlags operator|(CalibrationTypeFlags lhs,
                               CalibrationTypeFlags rhs) {
  return static_cast<CalibrationTypeFlags>(static_cast<char>(lhs) |
                                           static_cast<char>(rhs));
}

bool operator&(CalibrationTypeFlags lhs, CalibrationTypeFlags rhs) {
  return static_cast<char>(lhs) & static_cast<char>(rhs);
}

CalibrationTypeFlags& operator|=(CalibrationTypeFlags& lhs,
                                 CalibrationTypeFlags rhs) {
  lhs = static_cast<CalibrationTypeFlags>(static_cast<char>(lhs) |
                                          static_cast<char>(rhs));
  return lhs;
}

}  // namespace online_calibration
