## 协作原则

1. 不要假设我清楚自己想要什么。动机或目标不清晰时，停下来讨论。
2. 目标清晰但路径不是最短的，直接告诉我并建议更好的办法。
3. 遇到问题追根因，不打补丁。每个决策都要能回答“为什么”。
4. 输出说重点，砍掉一切不改变决策的信息。
5. 修改日志记录在 “NoneMoveBasePlanLog.md”，目录更新原则和方案落实情况由 `README.md` 负责维护，`Debug.md` 只记录调试流程。

# none_move_base_nav

## 文档职责

1. `README.md` 负责总结项目、记录方案落实情况、维护当前目录结构和目录更新原则。
2. `NoneMoveBasePlanLog.md` 只记录目录、文档、接口和实现的修改流水。
3. `Debug.md` 只记录调试入口、调试步骤、常见故障和验证路径。

## 目录更新原则

1. 新增或删除目录时，先更新本文件，再做后续实现。
2. `README.md` 只保留当前有效结构，不保留废弃目录说明。
3. 目录说明必须标明是“阶段 1 落地”还是“阶段 2 预留”。
4. 阶段 2 预留目录可以先存在，但不能在 README 里写成“已实现”。
5. 目录变化必须同步追加到 `NoneMoveBasePlanLog.md`。

## 当前目录结构

1. `none_move_base_bringup/`
   当前阶段：阶段 1
   用途：放新导航链自己的 launch 入口、参数文件、烟雾测试脚本和仿真调试入口。
2. `none_move_base_msgs/`
   当前阶段：阶段 1
   用途：放内部 `msg/action/srv` 定义。
3. `none_move_base_common/`
   当前阶段：阶段 1
   用途：放公共数据结构、工具函数和共享配置。
4. `none_move_base_global/`
   当前阶段：阶段 1
   用途：放 A* 全局规划实现。
5. `none_move_base_local/`
   当前阶段：阶段 1
   用途：放轻量 Holonomic Path Tracker。
6. `none_move_base_navigation/`
   当前阶段：阶段 1
   用途：放内部导航任务执行和状态编排。
7. `none_move_base_adapter/`
   当前阶段：阶段 2 预留
   用途：后续对接旧 `/obsNavigation_*` 业务接口。

## 当前落实情况

1. 已完成阶段 1 六个核心包和一个阶段 2 预留包的 catkin 骨架。
2. 已完成 `none_move_base_msgs` 的 `NavigateTask.action`、`NavigationStatus.msg`、`PathTrackingState.msg`、`BehaviorState.msg`、`ReloadStationPoses.srv`。
3. 已完成 `none_move_base_global` 的二维占据栅格 A*、路径裁剪、重采样和 yaw 补齐。
4. 已完成 `none_move_base_local` 的轻量 Holonomic Path Tracker，包含预瞄跟踪、近终点减速、最终朝向对齐、激光停障和速度限幅。
5. 已完成 `none_move_base_navigation` 的单层 `NavigateTask.action` 服务端，负责目标下发、运行状态汇总和超时控制。
6. 已完成 `none_move_base_bringup` 的 `parallel_debug.launch`、`navigation_core.launch` 和参数文件。
7. 已完成全工作区编译验证：
   使用当前默认 profile 执行 `catkin build`，结果为 `55/55` 成功，`web_video_server` 按既有 skiplist 跳过。
8. 已完成最小启动验证：
   在 `start_localization:=false` 条件下，`none_move_base_global`、`none_move_base_local`、`none_move_base_navigation` 可被 `parallel_debug.launch` 正常拉起。
9. 已完成最小闭环烟雾测试：
   使用空白 `/map`、模拟 `/amcl_pose_tf`、模拟 `/scan_full_filtered` 和 `NavigateTask` action goal，验证新链可从目标下发收敛到 `goal_reached`。
10. 已完成 ridgeback 仿真调试入口：
   新增 `ridgeback_sim_debug.launch`，将仿真定位链和新导航核心拼接到一起，并将阶段 1 输出从 `/hf_platform/nav_vel` 切到仿真使用的 `/cmd_vel`。
11. 已完成 ridgeback AMCL 仿真调试入口：
   新增 `ridgeback_sim_amcl_debug.launch`，支持直接使用 `pgm + yaml` 地图进行仿真定位，不依赖 `pbstream`。
12. 已完成 ridgeback Cartographer 仿真建图入口：
   新增 `ridgeback_sim_carto_mapping.launch`，只启动 Cartographer 建图相关节点，不夹带旧 `move_base`。

## 当前阶段边界

1. 阶段 1 只打通新导航链内部闭环，不接入旧 DMS 和旧 `task_communication`。
2. 阶段 1 只支持 `map` 坐标系下的 `target_pose` 目标，不支持命名站点。
3. 阶段 1 默认关闭终点 `tf_flash` 微调，只保留接口。
4. 阶段 1 保持旧底盘链、旧定位链和旧激光链不动，只复用其输入输出 topic。

## 仿真环境启动步骤
为避免 GUI 和后端在虚拟机或远程环境中同时启动导致崩溃（如界面卡死或黑屏），推荐将 Gazebo 服务端和客户端分开启动：

1. **清理遗留进程**（可选，环境异常时使用）：
   ```bash
   killall -9 roscore rosout roslaunch gzserver gzclient rosmaster robot_state_publisher rviz || true
   ```
2. **启动仿真后端**（无 GUI 模式）：
   ```bash
   source devel/setup.bash
   roslaunch ridgeback_test ridgeback_world.launch gui:=false joystick:=false
   ```
3. **启动仿真前端**（待后端启动稳定后，在新终端中运行）：
   ```bash
   gzclient
   ```

## 仿真联调入口

1. 启动 ridgeback 仿真后端后，在新终端运行：
   ```bash
   source /data/a/chemist_robot3.0/devel/setup.bash
   roslaunch none_move_base_bringup ridgeback_sim_debug.launch
   ```
2. 这个入口会启动：
   - `map_server`
   - `cartographer_node`
   - `lidar_odometry_rg_single_node`
   - `none_move_base_global`
   - `none_move_base_local`
   - `none_move_base_navigation`
3. 这个入口不会启动旧 `move_base`，也不会启动旧 DMS 业务链。
4. 仿真模式下阶段 1 的局部控制输出改为 `/cmd_vel`，odom 输入改为 `/odometry/filtered`。

## 仿真地图

1. 如果已经有现成 `pgm + yaml` 地图，不需要先重新建图。
2. 已将外部地图复制到：
   - `src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.pgm`
   - `src/platform/ridgeback/ridgeback_navigation/maps/ridgeback_map.yaml`
3. 如果你要走 `pgm + yaml` 地图定位，优先使用：
   `roslaunch none_move_base_bringup ridgeback_sim_amcl_debug.launch`
4. 如果你要走 `cartographer + pbstream`，继续使用：
   `roslaunch none_move_base_bringup ridgeback_sim_debug.launch`
5. 如果你要先在仿真里重新建一张 Cartographer 地图，使用：
   `roslaunch none_move_base_bringup ridgeback_sim_carto_mapping.launch`
