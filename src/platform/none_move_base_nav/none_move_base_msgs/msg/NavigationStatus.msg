std_msgs/Header header

string task_id
string exper_no
string action
string state
string phase
string detail
string current_station

geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped target_pose

float32 distance_to_goal
float32 yaw_error

bool is_tracking
bool is_charging
int32 error_code
