#include "none_move_base_common/status_codec.h"

namespace none_move_base_common {

std::string trackingStatusToPhase(int tracking_status) {
  switch (tracking_status) {
  case 1:
    return "tracking";
  case 2:
    return "near_goal";
  case 3:
    return "done";
  case 5:
    return "error";
  case 6:
    return "error";
  case 7:
    return "error";
  default:
    return "planning";
  }
}

} // namespace none_move_base_common
