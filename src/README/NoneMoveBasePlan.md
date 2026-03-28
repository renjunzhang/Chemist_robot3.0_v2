# NoneMoveBase 脱离方案

## 协作原则

1. 不要假设我清楚自己想要什么。动机或目标不清晰时，停下来讨论。
2. 目标清晰但路径不是最短的，直接告诉我并建议更好的办法。
3. 遇到问题追根因，不打补丁。每个决策都要能回答“为什么”。
4. 输出说重点，砍掉一切不改变决策的信息。
5. 修改日志记录在 “NoneMoveBasePlanLog.md”

## 目录

1. 协作原则
2. 修改方案摘要
3. 目标
4. 核心结论
5. 现状约束
6. 保留边界
7. 替换边界
8. 推荐目录结构
9. 包结构
10. Launch 设计
11. 外部接口表
12. 内部接口表
13. 依赖关系表
14. 算法选型结论
15. 新链路拓扑
16. 充电链迁移原则
17. 实施顺序
18. 已确认决策
19. 待确认问题
20. 第一阶段文件清单
21. NavigateTask.action 草案
22. NavigationStatus.msg 草案
23. PathTrackingState.msg 草案
24. BehaviorState.msg 草案
25. ReloadStationPoses.srv 草案
26. parallel_debug 最小启动图
27. 第一阶段验收标准

## 修改方案摘要

这份方案已经按“彻底脱离 `move_base` 体系”的原则重写：

1. 总体落地拆成两个阶段：
   - 第一阶段：3 天内先让机器人动起来
   - 第二阶段：再补齐旧业务接口和特殊流程
2. 不再保留 `/move_base` 接口，不再兼容 `move_base_msgs`。
3. 不再使用 `move_base`、MBF、`nav_core` 插件框架。
4. 外部只保留旧业务接口：
   - `/obsNavigation_in`
   - `/obsNavigation_out`
   - `/inquiryNavigation_out`
5. 内部改成全新的导航执行结构：
   - `adapter`
   - `navigation`
   - `global`
   - `local`
   - `common`
   - `bringup`
6. 第一阶段只做最小内部闭环，不接 DMS，不接 charge 全流程。
7. 第一阶段的算法已经固定为：
   - global: A*
   - local: 轻量 Holonomic Path Tracker
8. 第二阶段只保留接口和扩展位，不在本轮文档里锁死最终局部算法实现。
9. `task_communication` 不再作为运行链的一部分保留。
10. `charge` 业务语义保留，但旧代码里依赖 `MoveBaseGoal` 的执行部分要迁到新导航链里。
11. 底层仍然保持：
   - `/hf_platform/nav_vel -> twist_mux -> rs485_control`
   - `/hf_platform/crg_vel -> twist_mux -> rs485_control`
12. 方案取舍原因、MBF 取舍和工期估算单独记录在 [此方案的原因.md](/data/a/chemist_robot3.0/src/README/此方案的原因.md)。
13. 默认采用**非侵入式并行开发**：
   - 不改旧包源码
   - 不改旧 launch
   - 不改旧 config
   - 新代码全部放进 `src/platform/none_move_base_nav/`

## 1. 目标

这次方案的目标已经变成：

在保留外部业务接口和底层控制链的前提下，完全移除 `move_base` 体系，建立一套独立的全局规划 + 局部控制 + 任务执行结构。

这里的“完全移除”包括：

1. 不运行原 `move_base`
2. 不保留 `/move_base` action 兼容层
3. 不依赖 MBF
4. 不依赖 `nav_core` 插件式编排
5. 不让业务层继续直接操作 `MoveBaseGoal`

## 2. 核心结论

如果目标是“彻底脱离 `move_base` 体系”，那就不能再走“只替换 `/move_base`”路线。

正确方案必须同时替掉两层：

1. 旧导航执行内核
2. 旧 `task_communication` 中对 `MoveBaseAction` 的直接依赖

因此最终结构应该是：

1. 对外保留旧业务接口
2. 对内重建新的导航任务接口
3. 新导航直接输出 `/hf_platform/nav_vel`
4. 充电保持线程和底层驱动链继续复用

### 2.1 两阶段划分

#### 第一阶段：3 天能动版

只做：

1. `parallel_debug.launch`
2. `none_move_base_global`
3. `none_move_base_local`
4. `none_move_base_navigation`
5. 手工发内部目标
6. 输出 `/hf_platform/nav_vel`

先不做：

1. `/obsNavigation_in`
2. `/obsNavigation_out`
3. `/inquiryNavigation_out`
4. `charge / uncharge`
5. `charge-approach`
6. `dms_to_robot.py` 对接

#### 第二阶段：替旧链版

再做：

1. `none_move_base_adapter`
2. DMS 接口对接
3. 旧业务语义迁移
4. `charge / uncharge / back-off / rotate / charge-approach`
5. `cutover_compat.launch`

## 3. 现状约束

当前仓库里，旧业务层和 `move_base` 的耦合是硬编码的。

关键位置：

1. [platform_communication.h](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/include/platform_communication.h#L20) 直接包含 `move_base_msgs/MoveBaseAction.h`
2. [platform_communication.h](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/include/platform_communication.h#L48) 定义了 `SimpleActionClient<move_base_msgs::MoveBaseAction>`
3. [platform_communication.cpp](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/src/platform_communication.cpp#L148) 构造时直接连接 `"move_base"`
4. [platform_communication.cpp](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/src/platform_communication.cpp#L328) `StrollToGoal()` 内部直接 `sendGoal`
5. [platform_communication.cpp](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/src/platform_communication.cpp#L390) `RotateInPlace()` 内部直接 `sendGoal`
6. [platform_communication.cpp](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/src/platform_communication.cpp#L661) `ReCharge()` 内部先走 `MoveBaseGoal`
7. [platform_communication.cpp](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication/src/platform_communication.cpp#L737) `StrollToCharge()` 仍然直接 `sendGoal`

所以不能说“旧 `task_communication` 不动，只换 planner/controller”。

一旦目标是彻底脱离，这部分必须替掉。

## 4. 保留边界

下面这些链路继续保留：

1. [communication_rs485](/data/a/chemist_robot3.0/src/platform/control/communication_rs485)
2. [hf_bringup](/data/a/chemist_robot3.0/src/platform/control/hf_bringup)
3. [sick_safetyscanners](/data/a/chemist_robot3.0/src/platform/control/sick_safetyscanners)
4. [twist_mux](/data/a/chemist_robot3.0/src/platform/control/twist_mux)
5. [hf_karto](/data/a/chemist_robot3.0/src/platform/hf_mapping/hf_karto)
6. 地图服务、定位链、`/amcl_pose_tf`
7. `dms_to_robot.py` 这一层看到的业务接口名字
8. `/hf_platform/nav_vel`
9. `/hf_platform/crg_vel`
10. `/scan_full_filtered`
11. `/map`
12. `/odom`

### 4.1 非侵入原则

这一版方案默认按**非侵入式**执行。

明确要求：

1. 第一阶段**不修改**项目原有代码
2. 第二阶段默认也**不修改**项目原有代码
3. 新方案通过新增包、新增 launch、新增配置完成切换
4. 旧包只作为被复用的运行资源，不作为本轮修改目标

这里的“项目原有代码”包括但不限于：

1. [platform/control](/data/a/chemist_robot3.0/src/platform/control)
2. [platform/hf_mapping](/data/a/chemist_robot3.0/src/platform/hf_mapping)
3. [platform/hf_nav/hf_navigation](/data/a/chemist_robot3.0/src/platform/hf_nav/hf_navigation)
4. [platform/hf_nav/task_communication](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication)
5. [robot_message_bridge](/data/a/chemist_robot3.0/src/robot_message_bridge)

允许的动作只有：

1. 读取旧代码
2. 复用旧话题名
3. 复用旧底层链和定位链
4. 在新 launch 里决定“起不起旧节点”

默认不允许的动作：

1. 改旧 C++/Python 源码
2. 改旧 launch
3. 改旧 yaml 配置
4. 改旧消息和服务定义

例外规则：

1. 如果后面出现“必须改旧代码才能上线”的情况
2. 先停下来单独讨论
3. 没有明确确认前，不进入旧目录做修改

### 4.2 独立目录原则

新方案的所有新增实现统一放到下面这个独立目录：

`/data/a/chemist_robot3.0/src/platform/none_move_base_nav`

要求：

1. 新增包只放这个目录下
2. 新消息、新服务、新 action 也只放这个目录下
3. 新 launch、新 yaml、新 rviz 配置也只放这个目录下
4. 不把新逻辑拆散到旧目录中

这样做的原因：

1. 回退简单
2. 对比新旧链简单
3. 不污染原仓库既有结构
4. 上线切换时只需要切启动入口，不需要回收零散改动

## 5. 替换边界

下面这些不再进入新运行链：

1. [task_communication](/data/a/chemist_robot3.0/src/platform/hf_nav/task_communication)
2. 原 `move_base`
3. `move_base_msgs`
4. MBF
5. `/move_base`
6. `nav_core` 驱动的 planner/controller 插件式耦合

这里的意思不是删源码，而是：

1. 新链路不依赖它们
2. 切换到新链时不启动它们
3. 也不需要为了新链去修改它们的源码

## 6. 推荐目录结构

新目录根固定为：

`/data/a/chemist_robot3.0/src/platform/none_move_base_nav`

推荐保持 7 包结构：

```text
src/platform/none_move_base_nav/
├── none_move_base_bringup/
├── none_move_base_msgs/
├── none_move_base_common/
├── none_move_base_adapter/
├── none_move_base_global/
├── none_move_base_local/
└── none_move_base_navigation/
```

原因：

1. 现在不是“只换内核”，而是“外部接口保留、内部结构重做”
2. `adapter` 和 `navigation` 必须分开
3. `global` 和 `local` 必须分开
4. `msgs` 单独拆出后，后面换实现不会反复动业务接口
5. 新代码集中放一个目录，最符合当前“尽量零侵入旧项目”的原则

## 7. 包结构

### 7.1 `none_move_base_bringup`

职责：

1. 启动新链
2. 管理 debug / compat 模式
3. 管理参数、rviz、launch include

### 7.2 `none_move_base_msgs`

职责：

1. 定义内部 action / msg / srv
2. 不承载业务逻辑

建议接口：

1. `NavigateTask.action`
2. `NavigationStatus.msg`
3. `PathTrackingState.msg`
4. `BehaviorState.msg`
5. `ReloadStationPoses.srv`

说明：

1. 第一阶段内部只保留 1 层 action：`NavigateTask.action`
2. 不再保留 `NavigatePose.action`

### 7.3 `none_move_base_common`

职责：

1. 站点表加载
2. DMS JSON 编解码
3. TF 转换
4. 到站判定
5. 通用参数和状态工具

### 7.4 `none_move_base_adapter`

职责：

1. 替代旧 `task_communication` 的外部接口层
2. 订阅 `/obsNavigation_in`
3. 发布 `/obsNavigation_out`
4. 提供 `/inquiryNavigation_out`
5. 把外部任务转换为内部 `NavigateTask.action`

说明：

1. 新链对 DMS 的兼容入口只在这个包
2. 这个包不做 planner/controller

### 7.5 `none_move_base_global`

职责：

1. 输入当前位置和目标位姿
2. 输出全局路径 `nav_msgs/Path`
3. 不处理业务状态机

### 7.6 `none_move_base_local`

职责：

1. 输入全局路径、激光、里程计、定位
2. 输出全向底盘控制量
3. 直接发布 `/hf_platform/nav_vel`

关键约束：

1. 必须支持 `linear.x`
2. 必须支持 `linear.y`
3. 必须支持 `angular.z`
4. 不占用 `angular.x`
5. 不占用 `angular.y`

### 7.7 `none_move_base_navigation`

职责：

1. 新链内部唯一任务执行器
2. 维护 `move / charge / uncharge` 内部流程
3. 编排后退、旋转、去站点、充电前对位
4. 驱动 global/local

说明：

1. 旧业务语义保留在这里
2. 但实现原语不再是 `MoveBaseGoal`

## 8. Launch 设计

建议保留两套模式：

### 8.1 `parallel_debug.launch`

用途：

1. 新链独立调试
2. 不抢旧业务接口

建议公开接口：

1. `/none_move_base/obsNavigation_in`
2. `/none_move_base/obsNavigation_out`
3. `/none_move_base/inquiryNavigation_out`
4. `/none_move_base/status`
5. `/none_move_base/global_path`

### 8.2 `cutover_compat.launch`

用途：

1. 用新链替掉旧导航链
2. 但不改 DMS 侧调用方式

明确要求：

1. 不启动旧 `task_communication`
2. 不启动旧 `move_base`
3. 新链直接暴露：
   - `/obsNavigation_in`
   - `/obsNavigation_out`
   - `/inquiryNavigation_out`
   - `/hf_platform/nav_vel`

## 9. 外部接口表

这些接口对 DMS、底盘和定位链保持稳定：

| 接口 | 类型 | 说明 |
|---|---|---|
| `/obsNavigation_in` | `std_msgs/String` | DMS 下发任务 |
| `/obsNavigation_out` | `std_msgs/String` | 导航反馈 |
| `/inquiryNavigation_out` | `aichem_msg_srv/DmsService` | 状态查询 |
| `/hf_platform/nav_vel` | `geometry_msgs/Twist` | 新导航输出 |
| `/hf_platform/crg_vel` | `geometry_msgs/Twist` | 充电保持输出 |
| `/amcl_pose_tf` | `geometry_msgs/PoseWithCovarianceStamped` | 当前位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 地图 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/scan_full_filtered` | `sensor_msgs/LaserScan` | 激光输入 |
| `/tf_flash` | `std_msgs/String` | 终点微调偏差输入，第一阶段不接，后续按需启用 |
| `/voide_broadcast_msg` | `std_msgs/String` | 语音提示 |

## 10. 内部接口表

这些接口只在新链内部使用：

| 接口 | 类型 | 作用 |
|---|---|---|
| `/none_move_base/navigate_task` | `NavigateTaskAction` | adapter 调 navigation |
| `/none_move_base/global_goal` | `geometry_msgs/PoseStamped` | navigation 调 global |
| `/none_move_base/global_path` | `nav_msgs/Path` | global 输出 |
| `/none_move_base/path_tracking_state` | `PathTrackingState` | local 状态反馈 |
| `/none_move_base/status` | `NavigationStatus` | 内部状态广播 |
| `/none_move_base/reload_station_poses` | `ReloadStationPoses.srv` | 热更新站点 |

原则：

1. 外部接口维持旧名字
2. 内部接口全部进 `/none_move_base/*`
3. 内部只保留 1 层 action

## 11. 依赖关系表

| 包 | 编译依赖 | 运行时依赖旧资源 |
|---|---|---|
| `none_move_base_bringup` | `roslaunch` | `hf_bringup`、定位链 launch |
| `none_move_base_msgs` | `std_msgs`、`geometry_msgs`、`nav_msgs`、`actionlib_msgs`、`message_generation` | 无 |
| `none_move_base_common` | `roscpp`、`geometry_msgs`、`nav_msgs`、`tf2_ros` | 站点表、相机 pose 表 |
| `none_move_base_adapter` | `roscpp`、`std_msgs`、`actionlib`、`aichem_msg_srv` | `/amcl_pose_tf`、`/voide_broadcast_msg` |
| `none_move_base_global` | `roscpp`、`geometry_msgs`、`nav_msgs`、planner 所需依赖 | `/map`、`/amcl_pose_tf` |
| `none_move_base_local` | `roscpp`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_ros` | `/scan_full_filtered`、`/odom`、`/amcl_pose_tf` |
| `none_move_base_navigation` | `roscpp`、`actionlib`、`geometry_msgs`、`nav_msgs`、`tf2_ros` | `/amcl_pose_tf`、`/odom` |

推荐依赖方向：

```text
bringup
  -> adapter
  -> navigation
  -> global
  -> local

adapter
  -> msgs
  -> common

navigation
  -> msgs
  -> common

global
  -> msgs
  -> common

local
  -> msgs
  -> common
```

不要出现：

1. `global -> adapter`
2. `local -> adapter`
3. `navigation -> adapter`
4. `common -> adapter/navigation/global/local`

## 12. 算法选型结论

这里先直接给结论，不绕弯。

### 12.1 第一阶段全局规划算法

第一阶段推荐：

1. **二维占据栅格 A\***  
2. 路径回溯用 **gradient traceback**
3. 再做一次轻量级路径裁剪和朝向补齐

为什么这样选：

1. 你的机器人是全向麦轮，**全局层不需要非完整约束**，没必要一开始就上 Hybrid A\*。
2. 你的地图和定位链已经成熟，日常导航本质上是 **静态图上的起点到终点寻路**。
3. A\* 比 Dijkstra 更适合“单次起终点查询”，搜索面更小。
4. 这套算法和你当前仓库的地图形态最匹配，迁移成本最低。

实现建议：

1. `none_move_base_global` 里直接实现 `OccupancyGrid -> A* -> Path`
2. 可以参考 ROS `global_planner` 的算法思路，但**只借算法，不保留 `nav_core` / plugin 结构**
3. 起步时先不要加太多花哨后处理，只保留：
   - 路径去重
   - 直线段裁剪
   - 固定间距重采样
   - goal 朝向补齐

第一阶段对路径输出再补一个明确约束：

1. A\* 原始栅格路径不要直接给 local
2. 去掉共线点后，要按固定间距重新采样
3. 第一阶段建议间距先收敛到 **0.05 m 到 0.10 m**
4. 每个路径点都补切线方向 yaw

这样做的原因：

1. path tracker 吃的是参考轨迹，不是栅格锯齿
2. 点太稀会让 tracker 在拐角处抖动
3. 点太密会放大噪声和重复控制
4. 固定间距路径更容易做 preview point 和 near-goal 判定

为什么不是第一阶段就上更重的全局算法：

1. `Theta*` 可以后加，但不是当前主矛盾。
2. `Hybrid A*` 更适合车式或强非完整约束底盘，不是你这台麦轮当前的第一优先级。
3. 你现在真正的难点不在 global，而在 local 的全向避障与控制。

### 12.2 第一阶段局部规划算法

第一阶段推荐：

1. **轻量 Holonomic Path Tracker**

也就是：

1. 以全局路径为参考带
2. 用 preview point / carrot point 做路径跟踪
3. 输出 `vx / vy / wz`
4. 只做最小必要的限速、限加速度和障碍停障

为什么这样选：

1. 你要求 3 天内先动起来，TEB 对第一阶段来说太重。
2. 第一阶段的目标不是最优避障，而是先形成稳定的“目标 -> 路径 -> 速度输出”闭环。
3. 只要局部控制原生支持 `linear.y`，就已经能验证麦轮底盘的运动能力。
4. 轻量 tracker 更适合在 3 天内完成联调、限速、停障和到站判定。

这里要特别说清楚：

1. 第一阶段我**不建议**直接上 TEB
2. 这不是放弃 TEB，而是把它留到第二阶段

也就是说：

1. 第一阶段先做一个你自己的 standalone holonomic tracker
2. 第二阶段再把 local 升级为 Holonomic TEB

第一阶段 local 不能只停留在“沿路径跟一跟”，必须至少包含下面 5 个子能力：

1. **preview-point 跟踪**
   - 根据当前位姿在路径上找最近点
   - 再按 preview distance 取跟踪点
   - 在机器人坐标系下解出 `vx / vy / wz`
2. **速度限制**
   - 对 `vx / vy / wz` 做上下限和斜率限制
   - 避免指令突变
3. **最小停障**
   - 先不做复杂绕障
   - 但要基于 `/scan_full_filtered` 做前向危险区停障
4. **near-goal 模式**
   - 剩余距离进入阈值后，切换到近终点控制
   - 降低 preview distance 和速度上限
5. **终点收敛判定**
   - 单独判断 `xy` 和 `yaw`
   - 达到条件后输出 0 速并结束 action

### 12.2.1 第一阶段终点收敛策略

阶段 1 是否能“真正到站”，核心不在 A\*，而在 local 的终点处理。

第一阶段建议明确按下面这套最小策略做：

1. **远离终点时**
   - 正常按 preview-point 跟踪
   - 允许较大的 `vx / vy`
2. **进入 near-goal 距离后**
   - 典型阈值先建议 `0.30 m 到 0.50 m`
   - preview distance 同步收缩
   - 速度上限同步降低
3. **进入 xy_goal_tolerance 后**
   - 如果 `need_final_yaw=false`
     - 直接判成功
   - 如果 `need_final_yaw=true`
     - 切换到 final-yaw-align 模式
4. **final-yaw-align 模式**
   - `vx / vy` 限到接近 0
   - 只保留小幅 `wz`
   - 把航向误差收敛到 `yaw_goal_tolerance`
5. **最终 success 条件**
   - `xy` 在阈值内
   - `yaw` 在阈值内或本任务不要求最终朝向
   - 线速度和角速度均收敛到低阈值
6. **防抖条件**
   - 连续多周期满足 success 条件后再判完成
   - 避免穿过终点瞬间误判

一句话要求：

**阶段 1 的 tracker 必须做成“路径跟踪 + near-goal + final yaw align + success latch”，不能只是纯跟踪器。**

如果后续终点还需要更高精度微调，当前方案先预留但不实现：

1. 原始输入保留 `/tf_flash`
2. 内部行为阶段预留 `terminal_adjust`
3. `NavigateTask.action` 预留终点微调开关
4. 第一阶段默认不启用

### 12.3 第二阶段局部规划候选

第二阶段先只保留候选，不在这轮文档里锁死实现。

当前候选优先级：

1. **Holonomic TEB**
2. 更后续再考虑 **Holonomic MPC / MPC-CBF**

当前把 TEB 放在第一候选的原因：

1. 第一阶段已经把定位、路径和底盘速度链打通
2. 第二阶段才需要重点提升避障质量、轨迹质量和复杂场景稳定性
3. TEB 的实现和调参成本低于直接上 holonomic MPC

### 12.4 两阶段组合建议

第一阶段最稳的组合是：

1. `none_move_base_global`: **A\* + gradient traceback + path prune**
2. `none_move_base_local`: **Holonomic Path Tracker + obstacle stop**

第二阶段组合候选是：

1. `none_move_base_global`: **A\* + gradient traceback + path prune**
2. `none_move_base_local`: **更强的局部控制器**

这样分阶段的原因：

1. global 简单稳定，先别把复杂度堆在不关键的地方
2. 第一阶段先验证底盘闭环
3. 第二阶段再把 local 做强
4. 这样最符合“3 天内先动起来”的目标

### 12.5 更后续再考虑的算法

如果第二阶段跑通以后，你还要增强动态障碍和极限运动性能，再考虑：

1. local 从 Holonomic TEB 升级到 **Holonomic MPC / MPC-CBF**

但这不适合做第一阶段默认方案，原因是：

1. 你当前拿到的 `MPC-D-CBF` 代码是差速模型，不是全向模型
2. 一上来做 holonomic MPC，连状态方程、控制量、障碍输入、参数整定都要一起改
3. 风险明显高于先落地 TEB

## 13. 新链路拓扑

```text
[DMS / GUI]
   |
   v
[dms_to_robot.py]
   |
   v
[none_move_base_adapter]
  - /obsNavigation_in
  - /obsNavigation_out
  - /inquiryNavigation_out
  - station pose loader
  - DMS JSON codec
   |
   v
[none_move_base_navigation]
  - action server: /none_move_base/navigate_task
  - move / charge / uncharge state machine
  - backoff / rotate / path-follow / charge-approach
   |                    \
   |                     \
   v                      v
[none_move_base_global] [none_move_base_local]
   |                      |
   | global_path          | /hf_platform/nav_vel
   \______________________/
             |
             v
         [twist_mux]
             |
             v
   /hf_platform/twist_mux/cmd_vel
             |
             v
       [rs485_control]
```

充电保持链保持独立：

```text
[none_move_base_navigation::charging_keeper]
   -> /hf_platform/crg_vel
   -> [twist_mux]
   -> [rs485_control]
```

## 14. 充电链迁移原则

这里必须明确区分“保留业务语义”和“照搬旧代码”。

可以保留的：

1. `charge / uncharge` 业务语义
2. 到站后进入充电保持线程
3. `/hf_platform/crg_vel` 的使用方式
4. 电量检测和保持逻辑

不能原封不动照搬的：

1. `StrollToGoal()`
2. `RotateInPlace()`
3. `ReCharge()` 中靠 `MoveBaseGoal` 去充电站
4. `StrollToCharge()`

原因：

1. 这些旧实现原语就是 `MoveBaseGoal`
2. 一旦目标是彻底脱离 `move_base`，这些执行原语必须改写到新导航链里

## 15. 实施顺序

### 第一阶段：3 天能动版

#### 第 1 天

1. 建最小包骨架：
   - `none_move_base_bringup`
   - `none_move_base_msgs`
   - `none_move_base_common`
   - `none_move_base_global`
   - `none_move_base_local`
   - `none_move_base_navigation`
2. 起 `parallel_debug.launch`
3. 打通输入：
   - `/map`
   - `/odom`
   - `/amcl_pose_tf`
   - `/scan_full_filtered`

#### 第 2 天

1. 跑通 `A* -> /none_move_base/global_path`
2. 跑通轻量 holonomic path tracker
3. 输出 `/hf_platform/nav_vel`
4. 给路径加固定间距重采样和 yaw 补齐
5. 加最小速度限制和障碍停障

#### 第 3 天

1. 跑通 `NavigateTask.action`
2. 加 `near_goal` 模式和终点减速
3. 加最终 `yaw` 对齐和 success latch
4. 手工发目标让机器人完成基本到站
5. 固定接口文档
6. 记录参数和问题清单

### 第二阶段：替旧链版

#### 步骤 1

接外部业务接口：

1. `/obsNavigation_in`
2. `/obsNavigation_out`
3. `/inquiryNavigation_out`

#### 步骤 2

迁移旧业务语义：

1. `move`
2. `charge`
3. `uncharge`
4. `back-off`
5. `rotate-in-place`
6. `charge-approach`

#### 步骤 3

把 local 从轻量 tracker 升级到更强的局部控制器。

当前默认候选：

1. `Holonomic TEB`

#### 步骤 4

最后切换：

1. 停掉旧 `task_communication`
2. 停掉旧 `move_base`
3. 用 `cutover_compat.launch` 替代

## 16. 已确认决策

当前已经确认的内容：

1. 新根目录命名固定为 `none_move_base_nav`
2. 目标改为彻底脱离 `move_base` 体系
3. 不使用 MBF
4. 外部业务接口继续保持旧名字
5. 内部只保留 1 层 action
6. `charge` 业务语义保留，但执行原语要迁走
7. 第一阶段目标收敛为 **3 天内先动起来**
8. 第一阶段全局算法选 **二维占据栅格 A\***
9. 第一阶段局部算法选 **轻量 Holonomic Path Tracker**
10. 第二阶段局部算法暂不锁死，只保留扩展位
11. 当前第二阶段局部算法第一候选是 **Holonomic TEB**
12. 单独维护一份接口保留文档：[NoneMoveBaseReservedInterfaces.md](/data/a/chemist_robot3.0/src/README/NoneMoveBaseReservedInterfaces.md)，后续完善时只动实现，不轻易改接口名
13. 最终路线固定为 **方向 B：自有 7 包运行时，不采用 `locomotor` 直接落地**
14. 方案取舍原因单独记录在 [此方案的原因.md](/data/a/chemist_robot3.0/src/README/此方案的原因.md)
15. 第一阶段 path tracker 必须带 `near_goal`、最终 `yaw` 对齐和 success latch，确保能收敛到终点容差区
16. 预留 `/tf_flash` 终点微调扩展点，但第一阶段不实现具体逻辑
17. 旧目录默认只读，新增实现统一放到 `src/platform/none_move_base_nav/`
18. 没有单独确认前，不修改旧源码、旧 launch、旧 yaml

## 17. 待确认问题

开始实现前还需要确认下面 5 件事：

1. `none_move_base_global` 是自己实现 A\*，还是参考移植 `global_planner` 的核心代码
2. 第一阶段 `none_move_base_local` 的 path tracker 是用 preview-point + PID，还是用更简的 point-to-point controller
3. 第二阶段 local 是上 `Holonomic TEB`，还是换成别的更强控制器
4. `charge-approach` 是先做成 navigation 内部子流程，还是单独拆 `docking` 包
5. `parallel_debug.launch` 是否先只暴露 `/none_move_base/*` 调试接口

当前默认建议：

1. global 先用 A\*
2. 第一阶段 local 先用轻量 Holonomic Path Tracker
3. 第二阶段 local 第一候选保留为 `Holonomic TEB`
4. 第一阶段把 `charge-approach` 放在 `none_move_base_navigation` 内部
5. 先做 `parallel_debug.launch`
6. 第一阶段路径输出按固定间距重采样后再给 local

## 18. 第一阶段文件清单

这一节只列第一阶段真正建议落地的文件，不展开到所有可选优化项。

这里的第一阶段就是：

1. **3 天能动版**

所以原则是：

1. 先建会影响“路径 -> 速度输出”闭环的文件
2. `adapter` 和 `cutover_compat` 相关文件可以先建空壳，或直接延后到第二阶段

当前开工建议再收紧成一句话：

1. **先只保证 `bringup + msgs + common + global + local + navigation` 可编译、可启动、可跑通闭环**

### 18.1 `none_move_base_bringup`

建议第一批文件：

```text
none_move_base_bringup/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── parallel_debug.launch
│   └── include/
│       ├── localization_only.launch
│       ├── navigation_core.launch
│       └── debug_tools.launch
├── config/
│   ├── navigator.yaml
│   ├── global_planner.yaml
│   ├── local_controller.yaml
│   ├── reach_checker.yaml
│   └── behavior.yaml
└── rviz/
    └── none_move_base.rviz
```

### 18.2 `none_move_base_msgs`

建议第一批文件：

```text
none_move_base_msgs/
├── package.xml
├── CMakeLists.txt
├── action/
│   └── NavigateTask.action
├── msg/
│   ├── NavigationStatus.msg
│   ├── PathTrackingState.msg
│   └── BehaviorState.msg
└── srv/
    └── ReloadStationPoses.srv
```

### 18.3 `none_move_base_common`

建议第一批文件：

```text
none_move_base_common/
├── package.xml
├── CMakeLists.txt
├── include/none_move_base_common/
│   ├── station_pose_loader.h
│   ├── status_codec.h
│   ├── pose_utils.h
│   ├── frame_transform.h
│   └── reach_checker.h
└── src/
    ├── station_pose_loader.cpp
    ├── status_codec.cpp
    ├── pose_utils.cpp
    ├── frame_transform.cpp
    └── reach_checker.cpp
```

### 18.4 `none_move_base_adapter`

这个包属于第二阶段。

第一阶段建议：

1. 可以只建空包
2. 甚至可以先不建

原因：

1. 第一阶段不接 DMS
2. 第一阶段不需要 `/obsNavigation_*`

建议第一批文件：

```text
none_move_base_adapter/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── adapter_debug.launch
│   └── adapter_compat.launch
├── config/
│   └── adapter.yaml
└── src/
    ├── dms_navigation_adapter_node.cpp
    ├── dms_command_parser.cpp
    ├── dms_status_publisher.cpp
    └── inquiry_service.cpp
```

职责压缩成一句话：

1. 外部 JSON 接口都在这里结束
2. 内部统一转成 `NavigateTask.action`

### 18.5 `none_move_base_global`

建议第一批文件：

```text
none_move_base_global/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── global_planner.launch
├── config/
│   └── global_planner.yaml
├── include/none_move_base_global/
│   ├── astar_planner.h
│   ├── grid_adapter.h
│   └── path_postprocessor.h
└── src/
    ├── global_planner_node.cpp
    ├── astar_planner.cpp
    ├── grid_adapter.cpp
    └── path_postprocessor.cpp
```

### 18.6 `none_move_base_local`

建议第一批文件：

```text
none_move_base_local/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── local_controller.launch
├── config/
│   └── local_controller.yaml
├── include/none_move_base_local/
│   ├── path_tracker_controller.h
│   ├── obstacle_adapter.h
│   ├── velocity_limiter.h
│   └── goal_checker.h
└── src/
    ├── local_controller_node.cpp
    ├── path_tracker_controller.cpp
    ├── obstacle_adapter.cpp
    ├── velocity_limiter.cpp
    └── goal_checker.cpp
```

说明：

1. 第一阶段先做 `path_tracker_controller.*`
2. 第二阶段再补 `teb_controller.*`
3. 不继续保留 `teb_local_planner` 插件式入口
4. `goal_checker.*` 第一阶段就要承担 `near_goal`、`final_yaw_align` 和 success latch 判定

### 18.7 `none_move_base_navigation`

建议第一批文件：

```text
none_move_base_navigation/
├── package.xml
├── CMakeLists.txt
├── launch/
│   └── navigator.launch
├── config/
│   ├── navigator.yaml
│   └── behavior.yaml
└── src/
    ├── navigator_server_node.cpp
    ├── task_executor.cpp
    ├── behavior_manager.cpp
    ├── move_behavior.cpp
    ├── rotate_behavior.cpp
    ├── backoff_behavior.cpp
    ├── charge_behavior.cpp
    └── charging_keeper.cpp
```

### 18.8 第一阶段先不要建的文件

下面这些先不要急着建：

1. 多机器人调度相关接口
2. 第二层 pose action
3. 独立 `docking` 包
4. 动态障碍预测器
5. 可视化分析面板

理由：

1. 这些都不是第一阶段替旧链的必需项
2. 先建只会扩大边界

## 19. NavigateTask.action 草案

第一阶段内部只保留这一个 action。

建议定义：

```action
# Goal
string id
string exper_no
string action
string destination
geometry_msgs/PoseStamped target_pose
bool use_named_station
bool need_final_yaw
bool enable_terminal_adjust
string terminal_adjust_source
float32 xy_goal_tolerance
float32 yaw_goal_tolerance

---
# Result
bool success
int32 error_code
string state
string detail
string current_station

---
# Feedback
string phase
string detail
float32 distance_to_goal
float32 yaw_error
string current_station
```

### 19.1 字段约束

建议约束如下：

1. `action`
   - 只允许：
   - `move`
   - `charge`
   - `uncharge`
2. `destination`
   - 当 `use_named_station=true` 时必填
3. `target_pose`
   - 当 `use_named_station=false` 时使用
4. `xy_goal_tolerance`
   - 默认由 `navigator.yaml` 提供
5. `yaw_goal_tolerance`
   - 默认由 `navigator.yaml` 提供
6. `enable_terminal_adjust`
   - 第一阶段默认固定为 `false`
7. `terminal_adjust_source`
   - 第一阶段默认留空
   - 后续第一个保留值为 `tf_flash`

### 19.2 `error_code` 建议

建议先定义最小集合：

1. `0`: success
2. `1`: invalid_goal
3. `2`: station_not_found
4. `3`: no_localization
5. `4`: global_plan_failed
6. `5`: local_track_failed
7. `6`: timeout
8. `7`: canceled
9. `8`: charge_align_failed

### 19.3 `phase` 建议

建议先统一成下面这些运行阶段：

1. `idle`
2. `planning`
3. `tracking`
4. `near_goal`
5. `terminal_adjust`
6. `backoff`
7. `rotate`
8. `charge_approach`
9. `charging`
10. `done`
11. `error`

## 20. NavigationStatus.msg 草案

这个消息用于：

1. internal status 广播
2. adapter 组装 `/obsNavigation_out`
3. RViz / log / debug 面板观察运行状态

建议定义：

```msg
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
```

### 20.1 字段用途

建议这样用：

1. `state`
   - 对齐外部业务语义：
   - `idle / running / done / error / charging`
2. `phase`
   - 对齐内部执行阶段：
   - `planning / tracking / near_goal / terminal_adjust / rotate / backoff / charge_approach`
3. `detail`
   - 直接给日志和 DMS 文案使用
4. `error_code`
   - 和 `NavigateTask.action` 结果中的错误码保持一致

### 20.2 为什么单独要这个消息

不要把所有状态都塞进 action feedback，原因是：

1. action feedback 只对 action client 可见
2. 你后面还要给 adapter、调试工具、日志系统和可视化订阅
3. 单独状态 topic 更适合做运行时观测

## 21. PathTrackingState.msg 草案

这个消息只服务于 local 和 navigation 之间的闭环，不直接暴露给外部业务层。

建议定义：

```msg
std_msgs/Header header

bool has_path
bool goal_reached
bool oscillating
bool blocked

float32 cross_track_error
float32 heading_error
float32 remaining_distance

float32 commanded_vx
float32 commanded_vy
float32 commanded_wz

int32 tracking_status
string detail
```

### 21.1 `tracking_status` 建议

先定义最小集合：

1. `0`: idle
2. `1`: tracking
3. `2`: near_goal
4. `3`: goal_reached
5. `4`: path_lost
6. `5`: blocked
7. `6`: oscillating
8. `7`: failed

### 21.2 这个消息为什么不能省

因为 navigation 不能只知道“有没有到站”。

它还要根据 local 的状态决定：

1. 是否继续 tracking
2. 是否进入 rotate
3. 是否判定 blocked
4. 是否重规划
5. 是否超时失败

所以必须有一条 local -> navigation 的显式状态反馈。

## 22. BehaviorState.msg 草案

这个消息用于描述 navigation 内部行为管理器当前在跑什么。

建议定义：

```msg
std_msgs/Header header

string active_behavior
string previous_behavior

bool behavior_running
bool behavior_done
bool behavior_failed

int32 error_code
string detail
```

### 22.1 `active_behavior` 建议值

第一阶段建议先收成下面这些：

1. `idle`
2. `move`
3. `terminal_adjust`
4. `backoff`
5. `rotate`
6. `charge_approach`
7. `charging_keeper`

### 22.2 为什么需要单独的 behavior 消息

因为 `NavigationStatus.msg` 更偏“任务级状态”，而这里需要的是“行为级状态”。

例如：

1. 任务仍然是 `running`
2. 但当前具体卡在 `rotate`
3. 或当前已经切到了 `charge_approach`

这两类信息不要混在一起。

## 23. ReloadStationPoses.srv 草案

这个服务用于运行中热更新站点表，不要求第一阶段必须接 DMS，但建议接口先定好。

建议定义：

```srv
string file_path
bool reload_camera_poses
---
bool success
string detail
int32 station_count
```

### 23.1 字段说明

建议语义：

1. `file_path`
   - 允许为空
   - 为空时使用默认站点配置路径
2. `reload_camera_poses`
   - 是否顺带重载充电相关相机位姿表
3. `station_count`
   - 返回成功加载后的站点数量

### 23.2 为什么第一阶段就定这个接口

因为站点表迟早要热更新。

先把接口定下来，后面就不会反复改：

1. common 里的 loader
2. adapter 里的 station dispatch
3. navigation 里的 named station 解析逻辑

## 24. parallel_debug 最小启动图

第一阶段调试不要一上来就接 DMS。

推荐最小启动图：

```text
[hf_bringup old chain]
  - rs485
  - twist_mux
  - sick
  - scan merge/filter

[localization_only.launch]
  - map_server
  - cartographer localization
  - lidar_odometry
  - /amcl_pose_tf

[none_move_base_global]
[none_move_base_local]
[none_move_base_navigation]

[debug goal tool]
  - 手工发 NavigateTask.action
```

### 24.1 最小调试链路

最小链路应该是：

```text
debug goal
  -> /none_move_base/navigate_task
  -> none_move_base_navigation
  -> /none_move_base/global_goal
  -> none_move_base_global
  -> /none_move_base/global_path
  -> none_move_base_local
  -> /hf_platform/nav_vel
  -> twist_mux
  -> rs485_control
```

### 24.2 第一阶段建议先不接的内容

在 `parallel_debug` 阶段，先不要接：

1. `/obsNavigation_in`
2. `/obsNavigation_out`
3. `/inquiryNavigation_out`
4. `charge / uncharge` 全流程

理由：

1. 先把“导航基本闭环”跑通
2. 再接业务接口
3. 再接特殊行为

## 25. 第一阶段验收标准

第一阶段不是“文档完成”，而是要达到可以继续迁业务的门槛。

建议按下面 6 条验收：

### 25.1 全局规划可用

1. 给定 map 下起点和终点
2. `none_move_base_global` 能稳定输出 `nav_msgs/Path`
3. 路径没有明显回环和跳点
4. 路径经过重采样后点间距基本稳定
5. 路径点具备可用 yaw

### 25.2 局部控制可用

1. `none_move_base_local` 能稳定输出 `/hf_platform/nav_vel`
2. 输出包含：
   - `linear.x`
   - `linear.y`
   - `angular.z`
3. 不占用：
   - `angular.x`
   - `angular.y`
4. 进入 near-goal 后能明显降速
5. 到站后不会长时间来回穿点

### 25.3 真机能完成基本到站

1. 普通站点间导航可连续成功
2. 到站误差达到预设阈值
3. 需要最终朝向时，能收敛到 `yaw_goal_tolerance`
4. 成功判定后能稳定停住，不持续抖动
5. 至少覆盖：
   - 开阔区域
   - 狭窄区域
   - 轻微动态障碍

### 25.4 状态机闭环成立

1. `NavigateTask.action` 能正确返回：
   - success
   - aborted
   - canceled
2. `phase` 至少能区分：
   - `planning`
   - `tracking`
   - `near_goal`
   - `done`
   - `error`
3. `phase` 和 `error_code` 可用于后续接 DMS

### 25.5 老链不被破坏

1. 新链可以通过 `parallel_debug.launch` 独立启动
2. 不影响旧 `task_communication`
3. 不影响旧 `move_base`

### 25.6 可以进入第二阶段

达到下面条件才进入业务迁移：

1. 普通导航成功率达到可接受水平
2. 局部控制参数基本收敛
3. `/hf_platform/nav_vel` 输出稳定
4. 故障时不会出现明显危险动作
