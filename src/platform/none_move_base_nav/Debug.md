## 协作原则

1. 不要假设我清楚自己想要什么。动机或目标不清晰时，停下来讨论。
2. 目标清晰但路径不是最短的，直接告诉我并建议更好的办法。
3. 遇到问题追根因，不打补丁。每个决策都要能回答“为什么”。
4. 输出说重点，砍掉一切不改变决策的信息。
5. 修改日志记录在 “NoneMoveBasePlanLog.md”，目录更新原则和方案落实情况由 `README.md` 负责维护，`Debug.md` 只记录调试流程。

# Debug

## 文档用途

1. 只记录新导航链的调试方法。
2. 只记录真实可执行的步骤，不写方案讨论。
3. 每次增加新的调试入口或排障路径时，直接更新本文件。

## 当前阶段调试目标

1. 验证阶段 1 的内部导航闭环是否能独立跑通。
2. 验证 `/map`、`/amcl_pose_tf`、`/scan_full_filtered` 是否能稳定输入新链。
3. 验证 `/hf_platform/nav_vel` 是否能正确输出到底盘控制链。

## 当前调试入口

1. 先使用旧底层链提供传感器、定位和底盘能力。
2. 新链优先走 `parallel_debug` 模式，不直接接 DMS。
3. 手工发内部目标，先验证“能走、能到、能停住”。

## 编译准备

1. `cd /data/a/chemist_robot3.0`
2. `source /opt/ros/noetic/setup.bash`
3. `catkin build`
4. `source /data/a/chemist_robot3.0/devel/setup.bash`

## 最小启动命令

1. 只启动新导航核心，不拉旧定位链：
   `export ROS_HOME=/tmp/ros && mkdir -p /tmp/ros`
2. `source /data/a/chemist_robot3.0/devel/setup.bash`
3. `roslaunch none_move_base_bringup parallel_debug.launch start_localization:=false start_debug_tools:=false`
4. 当前已验证：
   上述 launch 可以正常拉起 `/none_move_base_global`、`/none_move_base_local`、`/none_move_base_navigation` 三个节点。

## 实物联调

1. 明天现场联调只做阶段 1，不接 DMS，不启动旧 `task_communication`，不启动旧 `hf_navigation.launch`。
2. 推荐分终端启动，不要把底层、定位和新导航全部塞进一个 launch 里。
3. 第一轮务必使用低速参数：
   `local_config:=/data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/config/local_controller_hw_debug.yaml`
4. 现场必须保证随手可按的物理急停。

## 实物启动顺序

1. 终端 A，启动底层和激光：
   `cd /data/a/chemist_robot3.0`
2. `source devel/setup.bash`
3. `roslaunch hf_bringup hf_bringup.launch robot_name:=hf_0`
4. 先确认：
   `rostopic hz /scan_full_filtered`
5. 再确认：
   `rostopic echo -n1 /odom`
6. 终端 B，启动阶段 1 新链和定位：
   `cd /data/a/chemist_robot3.0`
7. `source devel/setup.bash`
8. `roslaunch none_move_base_bringup parallel_debug.launch local_config:=/data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/config/local_controller_hw_debug.yaml start_localization:=true start_debug_tools:=false`
9. 这个入口默认会加载：
   `/home/a/speed_develop/config/current_pose.yaml`
10. 如果当前机器人停放位置和这个初始位姿不一致，先不要发导航目标，先修正初始位姿或重新摆车。

## 实物观察窗口

1. 终端 C 看定位：
   `rostopic echo /amcl_pose_tf`
2. 终端 D 看路径跟踪状态：
   `rostopic echo /none_move_base/path_tracking_state`
3. 终端 E 看总状态：
   `rostopic echo /none_move_base/status`
4. 终端 F 看导航速度输出：
   `rostopic echo /hf_platform/nav_vel`

## 实物第一步验证

1. 先不要跑大目标。
2. 第一轮只发一个相对前进 `0.30 m` 的小目标。
3. 新终端执行：
   `cd /data/a/chemist_robot3.0`
4. `source devel/setup.bash`
5. `python3 /data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/scripts/send_relative_goal.py --dx 0.30`
6. 预期现象：
   - `/none_move_base/global_path` 非空
   - `/hf_platform/nav_vel` 有低速输出
   - 机器人缓慢前进
   - 到点后速度归零
   - `/none_move_base/status` 进入 `done`
7. 第一轮不要先测横移，不要先测转向，不要先测远点。

## 实物第二步验证

1. 如果第一步稳定，再测：
   `python3 /data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/scripts/send_relative_goal.py --dx 0.50`
2. 如果前进稳定，再测横移：
   `python3 /data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/scripts/send_relative_goal.py --dy 0.20`
3. 如果平移稳定，再测小角度：
   `python3 /data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/scripts/send_relative_goal.py --dx 0.30 --dyaw 0.35 --need-final-yaw`

## 现场停机

1. 第一优先级始终是物理急停。
2. 软件取消当前任务：
   `rostopic pub -1 /none_move_base/navigate_task/cancel actionlib_msgs/GoalID "{}"`
3. 软件清空当前路径：
   `rostopic pub -1 /none_move_base/clear_path std_msgs/Empty "{}"`
4. 必要时直接停掉新导航：
   `rosnode kill /none_move_base_local /none_move_base_navigation /none_move_base_global`

## 实物失败先查什么

1. `amcl_pose_tf` 不更新：
   先停在定位层，不要查导航。
2. `global_path` 为空：
   先查 `/map`、`/amcl_pose_tf` 和目标点是否在地图内。
3. `global_path` 有，但 `/hf_platform/nav_vel` 一直是零：
   先查 `/none_move_base/path_tracking_state` 的 `detail`，重点看 `blocked_by_scan`、`no_pose`、`path_timeout`。
4. `nav_vel` 有，但机器人不动：
   先查底层链，不要改导航算法。
5. 机器人动了但不收敛：
   先保留日志，再回头调 `local_controller_hw_debug.yaml`，不要直接改代码。

## 标准烟雾测试

1. 先启动最小 launch：
   `roslaunch none_move_base_bringup parallel_debug.launch start_localization:=false start_debug_tools:=false`
2. 再执行闭环烟雾测试脚本：
   `python3 /data/a/chemist_robot3.0/src/platform/none_move_base_nav/none_move_base_bringup/scripts/phase1_smoke_test.py`
3. 当前已验证结果：
   在空白地图、模拟 `/amcl_pose_tf`、模拟 `/odom`、模拟 `/scan_full_filtered` 条件下，action goal 可收敛为 `goal_reached`。

## Ridgeback 仿真联调

1. 可选清理：
   `killall -9 roscore rosout roslaunch gzserver gzclient rosmaster robot_state_publisher rviz || true`
2. 启动仿真后端：
   `source /data/a/chemist_robot3.0/devel/setup.bash`
3. `roslaunch ridgeback_test ridgeback_world.launch gui:=false joystick:=false`
4. 如需 Gazebo 前端，在新终端运行：
   `gzclient`
5. 在新终端启动阶段 1 仿真联调入口：
   `source /data/a/chemist_robot3.0/devel/setup.bash`
6. `roslaunch none_move_base_bringup ridgeback_sim_debug.launch`
7. 这个联调入口已做好的仿真适配：
   - 局部控制输出 `/cmd_vel`
   - 里程计输入 `/odometry/filtered`
   - 定位输入 `/amcl_pose_tf`
   - 激光输入 `/scan_full_filtered`
8. 注意：
   不要再额外启动 `ridgeback_navigation/launch/carto_localization_demo.launch`，因为它会把旧 `move_base` 一起拉起来。

## 仿真键盘控制

1. 这套 ridgeback 仿真默认没有自动启动键盘控制节点。
2. 当前机器已安装 `teleop_twist_keyboard`，可直接用它给仿真底盘发 `/cmd_vel`。
3. 新终端执行：
   `cd /data/a/chemist_robot3.0`
4. `source devel/setup.bash`
5. `export ROS_HOME=/tmp/ros && mkdir -p /tmp/ros`
6. `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel`
7. 这个命令适合：
   - Cartographer 建图时手工开车
   - AMCL 或 Carto 定位联调时手工挪车
8. 如果要确认键盘控制生效：
   `rostopic echo /cmd_vel`

## Ridgeback 仿真 AMCL 联调

1. 如果你已经有现成 `pgm + yaml` 地图，不需要重新建图。
2. 当前项目已内置：
   - `src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.pgm`
   - `src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.yaml`
3. 启动仿真后端：
   `roslaunch ridgeback_test ridgeback_world.launch gui:=false joystick:=false`
4. 新终端启动 AMCL 版阶段 1 联调入口：
   `source /data/a/chemist_robot3.0/devel/setup.bash`
5. `roslaunch none_move_base_bringup ridgeback_sim_amcl_debug.launch`
6. 这个入口会启动：
   - `map_server`
   - `amcl`
   - `lidar_odometry_rg_single_node`
   - `none_move_base_global`
   - `none_move_base_local`
   - `none_move_base_navigation`
7. 这个入口适合直接验证“现成地图 + 仿真定位 + 阶段 1 新导航”。

## Ridgeback 仿真 Cartographer 建图

1. 启动仿真后端：
   `roslaunch ridgeback_test ridgeback_world.launch gui:=false joystick:=false`
2. 新终端启动 Cartographer 建图入口：
   `source /data/a/chemist_robot3.0/devel/setup.bash`
3. `roslaunch none_move_base_bringup ridgeback_sim_carto_mapping.launch`
4. 这个入口只会启动：
   - `cartographer_node`
   - `cartographer_occupancy_grid_node`
5. 这个入口不会启动旧 `move_base`，也不会启动阶段 1 新导航核心。
6. 建图时需要你在仿真里把车开动，至少覆盖主要走廊和转角。
7. 如果你不开 `gzclient`，也可以临时用 `rostopic pub -r` 给 `/cmd_vel` 连续发速度做简单巡航。
8. 如果启动后立刻出现 `Lookup would require extrapolation ... into the past`：
   先看它是不是只在启动最初几秒出现，若随后消失，可以忽略。
9. 如果这类报错持续刷屏：
   先关闭当前 Cartographer 建图节点，再重新启动仿真后端和 `ridgeback_sim_carto_mapping.launch`，不要让 Gazebo 已经跑了很久之后才启动建图。

## Cartographer 存图

1. 先结束当前轨迹：
   `rosservice call /finish_trajectory "{trajectory_id: 0}"`
2. 保存 `pbstream`：
   `rosservice call /write_state "{filename: '/data/a/chemist_robot3.0/src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.pbstream', include_unfinished_submaps: true}"`
3. 保存占据栅格地图：
   `rosrun map_server map_saver -f /data/a/chemist_robot3.0/src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map map:=/map`
4. 存完之后，再用：
   `roslaunch none_move_base_bringup ridgeback_sim_debug.launch map_file:=/data/a/chemist_robot3.0/src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.yaml load_state_filename:=/data/a/chemist_robot3.0/src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.pbstream`
5. 如果你只想用 `pgm + yaml`，不用 `pbstream`，则改用：
   `roslaunch none_move_base_bringup ridgeback_sim_amcl_debug.launch map_file:=/data/a/chemist_robot3.0/src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.yaml`

## 常用检查命令

1. 看核心节点：
   `rosnode list | grep none_move_base`
2. 看全局路径：
   `rostopic echo /none_move_base/global_path`
3. 看局部跟踪状态：
   `rostopic echo /none_move_base/path_tracking_state`
4. 看任务状态：
   `rostopic echo /none_move_base/status`
5. 看底盘速度输出：
   `rostopic echo /hf_platform/nav_vel`
6. 看 action 结果：
   `rostopic echo /none_move_base/navigate_task/result`
7. 仿真里看真正驱动车体的速度：
   `rostopic echo /cmd_vel`

## 手工发目标

1. 只验证全局规划和局部跟踪链：
   `rostopic pub -1 /none_move_base/global_goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"`
2. 验证完整 action 入口：
   `rostopic pub -1 /none_move_base/navigate_task/goal none_move_base_msgs/NavigateTaskActionGoal "{goal: {id: 'debug_goal_1', exper_no: '', action: 'move', destination: '', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, use_named_station: false, need_final_yaw: true, enable_terminal_adjust: false, terminal_adjust_source: '', xy_goal_tolerance: 0.12, yaw_goal_tolerance: 0.17}}"`

## 已知边界

1. 阶段 1 只支持 `map` 坐标系的 `target_pose`。
2. 阶段 1 不支持命名站点目标。
3. 阶段 1 不接入 `tf_flash` 终点微调，只保留接口。
4. 阶段 1 的最小启动验证和闭环烟雾测试都是在 `start_localization:=false` 下完成的，旧定位链联调仍待现场验证。

## 常见问题

1. `roslaunch` 直接报找不到包：
   先执行 `source /data/a/chemist_robot3.0/devel/setup.bash`。
2. `rospack` 报 `/home/a/.ros` 只读：
   调试前先设置 `export ROS_HOME=/tmp/ros`。
3. 新链有目标但不出速度：
   先查 `/amcl_pose_tf`、`/none_move_base/global_path`、`/scan_full_filtered` 是否存在，再看 `/none_move_base/path_tracking_state` 的 `detail`。
