#pragma once

#include <map>
#include <string>

#include <geometry_msgs/PoseStamped.h>

namespace none_move_base_common {

class StationPoseLoader {
public:
  bool loadFromFile(const std::string &file_path);
  bool hasStation(const std::string &station_name) const;
  bool getStationPose(const std::string &station_name,
                      geometry_msgs::PoseStamped *pose) const;
  std::size_t stationCount() const;

private:
  std::map<std::string, geometry_msgs::PoseStamped> station_map_;
};

} // namespace none_move_base_common
