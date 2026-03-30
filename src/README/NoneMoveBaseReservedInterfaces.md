# NoneMoveBase 接口保留文档

## 说明

这份文档只记录两阶段方案里需要保留下来的接口边界。

目的只有一个：

第一阶段先让机器人动起来，第二阶段继续完善时，尽量不再改接口名和接口语义。

补充原则：

1. 新实现默认放在 `src/platform/none_move_base_nav/`
2. 旧目录默认只读
3. 没有明确确认前，不通过修改旧代码来接新接口

## 1. 外部保留接口

这些接口是为了后续接回旧业务链保留的。

第一阶段可以不全部启用，但名字和语义先固定。

| 接口 | 类型 | 第一阶段 | 第二阶段 | 说明 |
|---|---|---|---|---|
| `/obsNavigation_in` | `std_msgs/String` | 不接 | 接入 | DMS 下发导航命令 |
| `/obsNavigation_out` | `std_msgs/String` | 不接 | 接入 | 导航反馈 |
| `/inquiryNavigation_out` | `aichem_msg_srv/DmsService` | 不接 | 接入 | 状态查询 |
| `/hf_platform/nav_vel` | `geometry_msgs/Twist` | 启用 | 启用 | 导航速度输出 |
| `/hf_platform/crg_vel` | `geometry_msgs/Twist` | 不接 | 启用 | 充电保持输出 |
| `/voide_broadcast_msg` | `std_msgs/String` | 不接 | 接入 | 语音播报 |

## 2. 输入依赖接口

这些接口来自当前旧系统，后续不建议改名。

| 接口 | 类型 | 用途 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 全局规划输入 |
| `/odom` | `nav_msgs/Odometry` | 局部控制输入 |
| `/amcl_pose_tf` | `geometry_msgs/PoseWithCovarianceStamped` | 当前位姿输入 |
| `/scan_full_filtered` | `sensor_msgs/LaserScan` | 障碍感知输入 |
| `/hf_platform/joy` | `sensor_msgs/Joy` | 最终速度门控输入（RB 按住才运行） |
| `/tf_flash` | `std_msgs/String` | 终点微调可选输入，第一阶段不接，后续按需启用 |

## 3. 新内部标准接口

这些接口是新链内部标准接口。

第一阶段就应尽量固定，后续不要轻易改名。

| 接口 | 类型 | 第一阶段 | 第二阶段 | 说明 |
|---|---|---|---|---|
| `/none_move_base/navigate_task` | `NavigateTaskAction` | 启用 | 启用 | internal task entry |
| `/none_move_base/global_goal` | `geometry_msgs/PoseStamped` | 启用 | 启用 | navigation 发给 global |
| `/none_move_base/global_path` | `nav_msgs/Path` | 启用 | 启用 | global 输出路径 |
| `/none_move_base/path_tracking_state` | `PathTrackingState` | 启用 | 启用 | local 状态反馈 |
| `/none_move_base/local_planner_cmd` | `geometry_msgs/Twist` | 预留 | 启用 | local_planner 输出给 local 执行层 |
| `/none_move_base/local_planner_state` | `std_msgs/String`（暂定） | 预留 | 启用 | local_planner 诊断状态 |
| `/none_move_base/local_obstacles` | 自定义 msg（预留） | 不接 | 可选 | 局部障碍集合（后续按需启用） |
| `/none_move_base/status` | `NavigationStatus` | 可选 | 启用 | internal status 广播 |
| `/none_move_base/behavior_state` | `BehaviorState` | 可选 | 启用 | behavior 状态广播 |
| `/none_move_base/reload_station_poses` | `ReloadStationPoses.srv` | 不接 | 启用 | 站点表热更新 |

## 4. 第一阶段必须实现的最小接口

为了满足“3 天能动起来”，第一阶段最小只需要这 5 个：

1. `/map`
2. `/amcl_pose_tf`
3. `/scan_full_filtered`
4. `/none_move_base/navigate_task`
5. `/hf_platform/nav_vel`

加上中间链路：

1. `/none_move_base/global_goal`
2. `/none_move_base/global_path`
3. `/none_move_base/path_tracking_state`

第一阶段当前基线还依赖：

1. `/odom`
2. `/hf_platform/joy`

第一阶段如果进入 MPC 试运行，新增最小中间接口：

1. `/none_move_base/local_planner_cmd`
2. `/none_move_base/local_planner_state`

## 5. 第二阶段再启用的接口

这些接口留名不留实现，第二阶段再接：

1. `/obsNavigation_in`
2. `/obsNavigation_out`
3. `/inquiryNavigation_out`
4. `/hf_platform/crg_vel`
5. `/voide_broadcast_msg`
6. `/none_move_base/status`
7. `/none_move_base/behavior_state`
8. `/none_move_base/reload_station_poses`
9. `/tf_flash`
10. `/none_move_base/local_planner_cmd`
11. `/none_move_base/local_planner_state`
12. `/none_move_base/local_obstacles`

## 6. `geometry_msgs/Twist` 语义约束

这个约束必须从第一阶段就固定下来。

| 字段 | 语义 |
|---|---|
| `linear.x` | 前后速度 |
| `linear.y` | 横移速度 |
| `angular.z` | 旋转速度 |
| `angular.x` | 继续保留给充电控制位 |
| `angular.y` | 继续保留给里程计清零控制位 |

要求：

1. 第一阶段和第二阶段都不要让 local controller 占用 `angular.x`
2. 第一阶段和第二阶段都不要让 local controller 占用 `angular.y`

## 7. `NavigateTask.action` 保留约束

内部任务接口的最小语义现在先固定为：

1. `action`
   - `move`
   - `charge`
   - `uncharge`
2. `destination`
   - 站点名模式
3. `target_pose`
   - 直接位姿模式
4. `xy_goal_tolerance`
5. `yaw_goal_tolerance`
6. `enable_terminal_adjust`
   - 是否在 near-goal 后进入终点微调
7. `terminal_adjust_source`
   - 后续第一个保留值为 `tf_flash`

要求：

1. 第一阶段即使只真正实现 `move`
2. 字段也不要删
3. 第二阶段继续沿用同一个 action 定义
4. 第一阶段默认 `enable_terminal_adjust=false`
5. 后续如果启用终点微调，优先先走 `/tf_flash`

## 8. 两阶段边界

### 第一阶段

目标：

1. 先让机器人动起来

范围：

1. `parallel_debug.launch`
2. 手工发内部目标
3. 全局路径生成
4. 局部路径跟踪
5. 输出 `/hf_platform/nav_vel`
6. 不接 `/tf_flash`

### 第二阶段

目标：

1. 替掉旧导航业务链

范围：

1. 接 DMS
2. 接 query service
3. 接 charge / uncharge / rotate / backoff / charge-approach
4. 接 `/hf_platform/crg_vel`
5. 按需接入 `/tf_flash` 终点微调

## 9. 非侵入实现边界

这套方案默认按“旧目录只读、新目录实现”的方式推进。

默认不改的目录：

1. `src/platform/control/`
2. `src/platform/hf_mapping/`
3. `src/platform/hf_nav/hf_navigation/`
4. `src/platform/hf_nav/task_communication/`
5. `src/robot_message_bridge/`

默认新增的目录：

1. `src/platform/none_move_base_nav/`

允许做的事情：

1. 读取旧代码
2. 复用旧话题、旧底盘链、旧定位链
3. 通过新 launch 决定起哪个节点

默认不做的事情：

1. 修改旧源码
2. 修改旧 launch
3. 修改旧 yaml
4. 把新逻辑散落到旧目录里

## 10. 不建议后续再改的内容

后续如果没有明确必要，不建议再改下面这些：

1. `/none_move_base/*` 命名空间前缀
2. `/hf_platform/nav_vel` 作为导航最终输出
3. `NavigateTask.action` 的主字段集合
4. `PathTrackingState.msg` 作为 local -> navigation 的反馈通道
5. `NavigationStatus.msg` 作为 internal status 广播通道
6. `none_move_base_local` 作为最终执行与安全门控层的角色边界
7. `/none_move_base/local_planner_cmd` 作为 local_planner -> local 执行层接口名
