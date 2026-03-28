## 协作原则

1. 不要假设我清楚自己想要什么。动机或目标不清晰时，停下来讨论。
2. 目标清晰但路径不是最短的，直接告诉我并建议更好的办法。
3. 遇到问题追根因，不打补丁。每个决策都要能回答“为什么”。
4. 输出说重点，砍掉一切不改变决策的信息。
5. 修改日志记录在 “NoneMoveBasePlanLog.md”，目录更新原则和方案落实情况由 `README.md` 负责维护，`Debug.md` 只记录调试流程。

# NoneMoveBasePlan 修改日志

## 2026-03-28

1. 新建 `src/platform/none_move_base_nav/` 作为新导航链的独立根目录。
2. 在根目录下创建 `README.md`、`NoneMoveBasePlanLog.md`、`Debug.md` 三份基础文档。
3. 创建 `none_move_base_bringup`、`none_move_base_msgs`、`none_move_base_common`、`none_move_base_global`、`none_move_base_local`、`none_move_base_navigation`、`none_move_base_adapter` 目录骨架。
4. 在 `README.md` 中补充目录更新原则，明确 README 负责维护当前有效结构和方案落实情况。
5. 明确 `none_move_base_adapter/` 只作为阶段 2 预留目录，当前阶段不进入实现清单。
6. 完成 `none_move_base_msgs` 的阶段 1 接口定义，包括 `NavigateTask.action`、`NavigationStatus.msg`、`PathTrackingState.msg`、`BehaviorState.msg` 和 `ReloadStationPoses.srv`。
7. 完成 `none_move_base_common` 的公共工具实现，包括位姿转换、终点判定、状态编码和阶段 1 占位版站点加载器。
8. 完成 `none_move_base_global` 的阶段 1 全局规划器，实现二维占据栅格 A*、障碍膨胀、路径裁剪、重采样和 yaw 补齐。
9. 完成 `none_move_base_local` 的阶段 1 局部控制器，实现轻量 Holonomic Path Tracker、速度限幅、近终点控制和激光停障。
10. 完成 `none_move_base_navigation` 的阶段 1 导航执行节点，实现 `/none_move_base/navigate_task` action server、全局目标下发、状态反馈和超时控制。
11. 完成 `none_move_base_bringup` 的 launch 和参数文件，实现 `parallel_debug.launch`、`navigation_core.launch`、`localization_only.launch`、`debug_tools.launch`。
12. 完成新链定向编译验证，相关包均可通过 `catkin build`。
13. 完成全工作区编译验证，结果为 `55/55` 成功，未引入新的构建失败。
14. 完成最小启动验证，`parallel_debug.launch start_localization:=false start_debug_tools:=false` 可正常拉起三个核心节点。
15. 更新根目录 `README.md` 和 `Debug.md`，将目录状态从“仅骨架”修正为“阶段 1 已落地并通过编译、启动校验”。
16. 新增 `none_move_base_bringup/scripts/phase1_smoke_test.py`，作为阶段 1 标准闭环烟雾测试脚本。
17. 使用 `phase1_smoke_test.py` 完成最小闭环验证，在空白地图和模拟位姿条件下，`/none_move_base/navigate_task` 可成功到达 `goal_reached`。
18. 回写 `README.md` 和 `Debug.md`，补充烟雾测试脚本入口和当前已验证范围。
19. 将 `navigation_core.launch` 改为支持通过 launch 参数切换配置文件，便于后续为不同运行环境复用同一核心启动入口。
20. 新增 `local_controller_ridgeback_sim.yaml`，将阶段 1 仿真环境的速度输出切到 `/cmd_vel`，并将 odom 输入切到 `/odometry/filtered`。
21. 新增 `ridgeback_sim_localization.launch`，只启动 ridgeback 仿真的地图服务、cartographer 定位和 `amcl_pose_tf` 发布，不再夹带旧 `move_base`。
22. 新增 `ridgeback_sim_debug.launch`，用于将 ridgeback 仿真定位链与阶段 1 新导航核心拼接联调。
23. 校验 `ridgeback_sim_debug.launch` 的展开文件和节点列表，确认语法和节点拓扑正确。
24. 将 `/data/a/MPPI_ws/src/ridgeback_navigation/maps/ridgeback_map.pgm` 和 `ridgeback_map.yaml` 复制到当前项目的 `ridgeback_navigation/maps/`。
25. 修正复制进项目的 `ridgeback_map.yaml`，将 `image` 路径改为项目内相对路径 `ridgeback_map.pgm`。
26. 新增 `ridgeback_sim_amcl_localization.launch`，用于只启动 `map_server + amcl + lidar_odometry_rg_single_node` 的仿真定位链。
27. 新增 `ridgeback_sim_amcl_debug.launch`，用于直接使用 `pgm + yaml` 地图进行阶段 1 仿真联调，不依赖 `pbstream`。
28. 校验 `ridgeback_sim_amcl_debug.launch` 的展开文件和节点列表，确认语法和节点拓扑正确。
29. 新增 `ridgeback_sim_carto_mapping.launch`，用于只启动 ridgeback 仿真 Cartographer 建图链，不夹带旧 `move_base`。
30. 校验 `ridgeback_sim_carto_mapping.launch` 的展开文件和节点列表，确认语法和节点拓扑正确。
31. 在 `Debug.md` 中补充 ridgeback 仿真 Cartographer 建图步骤，以及 `finish_trajectory`、`write_state`、`map_saver` 的存图命令。
32. 从 `ridgeback_sim_carto_mapping.launch` 中移除 `lidar_odometry_rg_single_node`，避免纯建图时引入无关的 `map<->base_link` TF 查询噪声。
33. 在 `Debug.md` 中补充 Cartographer 建图启动初期 TF extrapolation 报错的判断方式和重启建议。
34. 将 `parallel_debug.launch` 改为支持通过 launch 参数覆盖 `global/local/navigator/reach/behavior` 配置文件，便于切换不同联调参数。
35. 新增 `local_controller_hw_debug.yaml`，作为实物第一轮联调的低速参数配置。
36. 新增 `send_relative_goal.py`，用于基于当前 `/amcl_pose_tf` 发送相对小目标，避免现场手算绝对坐标。
37. 对 `send_relative_goal.py` 做 Python 语法检查，结果通过。
38. 在 `Debug.md` 中新增实物联调章节，补充启动顺序、低速参数、相对目标、停机方法和分层排障顺序。
