#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace none_move_base_common {

geometry_msgs::PoseStamped toPoseStamped(
    const geometry_msgs::PoseWithCovarianceStamped &msg);

} // namespace none_move_base_common
