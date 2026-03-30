## 协作原则

1. 不要假设我清楚自己想要什么。动机或目标不清晰时，停下来讨论。
2. 目标清晰但路径不是最短的，直接告诉我并建议更好的办法。
3. 遇到问题追根因，不打补丁。每个决策都要能回答“为什么”。
4. 输出说重点，砍掉一切不改变决策的信息。
5. 本方案的修改日志统一记录在 `NoneMoveBasePlanLog.md`。
6. 接口记录统一维护在 `/data/a/chemist_robot3.0/src/README/NoneMoveBaseReservedInterfaces.md`。

# MPC-D-CBF 改进方案

## 目录

1. 目标与结论
2. 为什么不直接复制 `/home/a/scout_ws/src/MPC-D-CBF`
3. 迁移边界与保持不变项
4. 推荐目录结构（第一版先收紧到 local_planner）
5. 运行链路设计
6. 接口设计
7. 模型与约束收敛
8. 分阶段实施计划
9. 验收标准
10. 风险与回退方案
11. 第一阶段最小文件清单

## 1. 目标与结论

目标：

1. 在保持当前 `none_move_base` 运行框架不变的前提下，吸收 MPC-D-CBF 的局部规划思想。
2. 提升全向麦轮底盘在动态障碍场景下的局部避障能力。
3. 继续坚持“脱离 move_base 框架”的路线，不引入 `move_base` 依赖。

结论：

1. 不建议直接复制整仓代码。
2. 建议采用“结构借鉴 + 算法核心迁移 + 接口适配”的方案。
3. 当前 `none_move_base_local` 的最终速度安全门控必须保留，作为不可绕过的硬约束。
4. 第一版只迁 `MPC-D-CBF` 的求解器思路，不迁原仓的 `controller.py`、`local_map` 和 `obs_param` 运行链。
5. 第一版先新增 `none_move_base_local_planner/`，直接复用当前 `global_path + amcl_pose_tf + odom + scan_full_filtered`。

## 2. 为什么不直接复制 `/home/a/scout_ws/src/MPC-D-CBF`

不直接复制的原因：

1. 该仓库是完整演示栈（含 scene、global_path_publisher、local_map、obs_param、local_planner），直接复制会引入过多耦合。
2. 现有项目已经有稳定的 topic 命名、状态机和调试链路，整仓迁移会破坏当前可验证基线。
3. 当前项目已实现手柄“按住才运行”的硬门控，直接替换 local 节点会有安全回归风险。
4. 直接复制后，故障定位会变成“跨两套工程风格”排障，不利于现场联调与回退。
5. 原仓 `controller.py` 只输出 `linear.x + angular.z`，没有 `linear.y`，不能直接用于当前全向麦轮底盘。
6. 原仓 `local_planner.py` 的优化变量是 `v + omega`，系统模型是差速/独轮车模型，不是当前项目需要的 `vx + vy + wz`。
7. 原仓障碍链依赖 `local_map -> obs_kf -> /obs_predict_pub`，而当前项目现成输入是 `/scan_full_filtered`，直接照搬会引入额外感知链耦合。
8. 原仓主要使用 `world -> base_link` 坐标关系，当前项目以 `map` 和 `/amcl_pose_tf` 为定位基准，直接迁移会增加坐标系改造面。

## 3. 迁移边界与保持不变项

保持不变：

1. 不引入 `move_base`、MBF 和 `nav_core` 插件框架。
2. `none_move_base_navigation` 继续作为任务执行与状态编排入口。
3. `none_move_base_global` 继续提供全局路径输入。
4. 最终速度出口继续由 `none_move_base_local` 统一发布。
5. 手柄门控逻辑（`/hf_platform/joy` + `buttons[5]`）保持在最终输出口。

允许新增：

1. 在 `src/platform/none_move_base_nav/` 下新增 MPC-D-CBF 适配包。
2. 在不改旧目录源码前提下，新增 launch/config/msg 参数。
3. 通过参数切换 current tracker 与 MPC-D-CBF planner。

## 4. 推荐目录结构（第一版先收紧到 local_planner）

第一版建议只新增：

1. `none_move_base_local_planner/`
		用途：实现 MPC-D-CBF 求解器，输出期望控制量。

保留并调整：

1. `none_move_base_local/`
	用途收敛为“执行与安全层”：
		- 接收 `none_move_base_local_planner` 输出
		- 执行速度限幅与最终门控
		- 发布 `/hf_platform/nav_vel` 或 `/cmd_vel`

后置预留：

1. `none_move_base_local_map/`
		用途：如果单纯使用 `/scan_full_filtered` 生成的简化障碍集合不够，再补局部障碍描述层。
2. `none_move_base_obs_param/`
		用途：如果后续确实需要动态障碍速度估计和预测，再补参数估计与滤波层。

## 5. 运行链路设计

第一版推荐链路：

1. `none_move_base_navigation` 下发目标。
2. `none_move_base_global` 发布 `/none_move_base/global_path`。
3. `none_move_base_local_planner` 直接读取 `path + pose + odom + scan`，生成局部控制量。
4. `none_move_base_local` 作为最终执行层进行限幅、硬门控和零速保护后发布速度。

后续增强链路：

1. 当动态障碍和遮挡场景证明单纯 `scan` 不够时，再在 `global/local` 之间插入 `none_move_base_local_map`。
2. 当确实需要障碍速度估计和预测时，再新增 `none_move_base_obs_param`，而不是一开始就复制 `/obs_predict_pub` 方案。

## 6. 接口设计

### 6.1 输入接口（建议）

1. `/none_move_base/global_path` (`nav_msgs/Path`)
2. `/amcl_pose_tf` (`geometry_msgs/PoseWithCovarianceStamped`)
3. `/odom` (`nav_msgs/Odometry`)
4. `/scan_full_filtered` (`sensor_msgs/LaserScan`)
5. `/hf_platform/joy` (`sensor_msgs/Joy`)

### 6.2 新增中间接口（建议）

1. `/none_move_base/local_planner_cmd` (`geometry_msgs/Twist`)
2. `/none_move_base/local_planner_state`（状态诊断 msg）
3. `/none_move_base/local_obstacles`（自定义 msg，阶段 C 可选）

### 6.3 保持不变的输出接口

1. `/hf_platform/nav_vel`（实机）
2. `/cmd_vel`（仿真）

### 6.4 明确不直接沿用的原仓接口

1. 不直接沿用 `/curr_state` 的 `Float32MultiArray` 接口，当前项目继续用 `/amcl_pose_tf + /odom`。
2. 不直接沿用 `/local_plan` 的 `Float32MultiArray` 接口，统一改成 `/none_move_base/local_planner_cmd` 的 `geometry_msgs/Twist`。
3. 不直接沿用 `/obs_predict_pub` 作为第一版障碍输入，先直接从 `/scan_full_filtered` 构造简化障碍约束。
4. 不直接沿用 `world` 作为规划固定坐标系，当前项目继续使用 `map`。

## 7. 模型与约束收敛

第一版模型约束必须先收敛到当前底盘，而不是照搬原仓：

1. 状态量：`x, y, yaw`
2. 控制量：`vx, vy, wz`
3. 输出话题：`/none_move_base/local_planner_cmd`
4. 最终速度出口：仍由 `none_move_base_local` 统一发布到 `/hf_platform/nav_vel` 或 `/cmd_vel`

第一版障碍约束建议：

1. 直接从 `/scan_full_filtered` 提取简化障碍点或局部圆障碍。
2. 先做静态障碍或准静态障碍约束，不在第一版引入复杂动态预测。
3. 求解失败时直接输出零速度，并允许按参数回退到当前 tracker。

## 8. 分阶段实施计划

### 阶段 A：框架接入（1-2 天）

1. 只新增 `none_move_base_local_planner` 包骨架。
2. 新增 launch 入口：`navigation_core_mpcdcbf.launch`。
3. 不改现有 tracker 实现，先保证编译和节点连通。
4. `none_move_base_local` 继续保留为最终执行与安全层。

### 阶段 B：最小可运行 MPC（2-4 天）

1. 迁移 MPC-D-CBF 的最小求解流程到 `none_move_base_local_planner`。
2. 将原仓差速模型改为全向 `vx + vy + wz` 模型。
3. 输入先直接使用 `/scan_full_filtered` 构造简化障碍集合，不做复杂预测。
4. 输出 `local_planner_cmd` 给 `none_move_base_local`。
5. 保持手柄门控有效，验证“按住才运行”。

### 阶段 C：动态障碍与鲁棒性（3-7 天）

1. 视第一版效果决定是否新增 `none_move_base_local_map`。
2. 视动态障碍需求决定是否新增 `none_move_base_obs_param`。
3. 强化 `scan/local_map` 与 CBF 约束一致性。
4. 增加失败保护：求解失败时零速、降级到旧 tracker（可选）。

### 阶段 D：实机收敛与切换（持续迭代）

1. 场景化调参：窄通道、交汇口、动态穿行。
2. 对比旧 tracker：成功率、到站时间、最小距离、急停响应。
3. 达标后将默认 local 模式切换为 MPC-D-CBF。

## 9. 验收标准

1. 编译通过：新增包及原有包均可 `catkin build`。
2. 启动通过：新入口可稳定启动且不依赖 `move_base`。
3. 安全通过：RB 松手同周期零速，无残余速度输出。
4. 功能通过：发目标后可稳定收敛到 `goal_reached`。
5. 退化通过：局部求解异常时可安全停机，不发生突进。

## 10. 风险与回退方案

主要风险：

1. 求解器实时性不足导致控制抖动。
2. 动态障碍参数误差导致 CBF 约束过紧或失效。
3. 坐标系与时序不同步导致控制量异常。
4. 若未先改成 `vx + vy + wz` 全向模型，控制效果会与底盘运动学不匹配。

回退策略：

1. 保留当前 `parallel_debug.launch` 作为稳定基线。
2. 新增入口与原入口并存，不覆盖旧入口。
3. 通过 launch 参数切换 local 模式：`tracker` / `mpc_dcbf`。
4. 回退时只切参数和 launch，不回滚代码目录结构。

## 11. 第一阶段最小文件清单

1. `none_move_base_local_planner/package.xml`
2. `none_move_base_local_planner/CMakeLists.txt`
3. `none_move_base_local_planner/src/mpc_dcbf_planner_node.cpp`（或 Python 等价实现）
4. `none_move_base_bringup/launch/include/navigation_core_mpcdcbf.launch`
5. `none_move_base_bringup/config/local_planner_mpcdcbf.yaml`
6. `none_move_base_local/` 中增加接收 `local_planner_cmd` 的模式切换参数
7. `none_move_base_bringup/config/local_controller.yaml` 中增加 `local_mode: tracker/mpc_dcbf`

后续可选文件：

1. `none_move_base_local_map/package.xml`
2. `none_move_base_local_map/src/local_obstacle_builder_node.cpp`
3. `none_move_base_obs_param/package.xml`
4. `none_move_base_obs_param/src/obstacle_state_estimator_node.cpp`

---

本方案定位：

1. 不是整仓搬迁方案。
2. 是“在现有 none_move_base 框架内，安全增量引入 MPC-D-CBF”的执行方案。
