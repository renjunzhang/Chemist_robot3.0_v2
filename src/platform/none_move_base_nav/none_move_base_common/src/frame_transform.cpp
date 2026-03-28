#include "none_move_base_common/frame_transform.h"

#include "none_move_base_common/pose_utils.h"

namespace none_move_base_common {

geometry_msgs::PoseStamped toPoseStamped(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {
  return poseStampedFromCovariance(msg);
}

} // namespace none_move_base_common
