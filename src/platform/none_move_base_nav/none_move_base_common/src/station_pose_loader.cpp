#include "none_move_base_common/station_pose_loader.h"

namespace none_move_base_common {

bool StationPoseLoader::loadFromFile(const std::string & /*file_path*/) {
  station_map_.clear();
  return false;
}

bool StationPoseLoader::hasStation(const std::string &station_name) const {
  return station_map_.find(station_name) != station_map_.end();
}

bool StationPoseLoader::getStationPose(const std::string &station_name,
                                       geometry_msgs::PoseStamped *pose) const {
  const auto iter = station_map_.find(station_name);
  if (iter == station_map_.end() || pose == nullptr) {
    return false;
  }
  *pose = iter->second;
  return true;
}

std::size_t StationPoseLoader::stationCount() const { return station_map_.size(); }

} // namespace none_move_base_common
