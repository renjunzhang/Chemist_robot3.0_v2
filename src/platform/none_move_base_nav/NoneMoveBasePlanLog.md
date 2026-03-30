## 协作原则

1. 不要假设我清楚自己想要什么。动机或目标不清晰时，停下来讨论。
2. 目标清晰但路径不是最短的，直接告诉我并建议更好的办法。
3. 遇到问题追根因，不打补丁。每个决策都要能回答“为什么”。
4. 输出说重点，砍掉一切不改变决策的信息。
5. 修改日志记录在 “NoneMoveBasePlanLog.md”，目录更新原则和方案落实情况由 `README.md` 负责维护，`Debug.md` 只记录调试流程。
6. 接口记录统一维护在 `/data/a/chemist_robot3.0/src/README/NoneMoveBaseReservedInterfaces.md`。

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

## 2026-03-29

39. 在 `none_move_base_local/src/local_controller_node.cpp` 增加最终速度输出口安全门控，直接订阅 `/hf_platform/joy` 并读取 `buttons[5]`。
40. 新增门控参数：`require_enable_button`、`enable_button_topic`、`enable_button_index`、`enable_release_timeout`、`enable_release_behavior`、`joy_msg_timeout`、`publish_zero_on_block`。
41. 实现“按住才运行”语义：使能键未按下时立即发布零速度，并阻断 tracker 控制量下发。
42. 新增门控 detail：`hold_to_run_required`、`manual_hold_released`、`manual_hold_timeout_cleared`、`joy_timeout_fail_safe`，便于在线诊断。
43. 新增二级保护：松手超过阈值且 `enable_release_behavior=cancel` 时自动 `clearPath`，避免重新按下后沿旧路径突进。
44. 更新配置默认值：
	- `local_controller.yaml`：门控默认开启
	- `local_controller_hw_debug.yaml`：门控默认开启
	- `local_controller_ridgeback_sim.yaml`：门控默认关闭
45. 定向编译验证通过：执行 `catkin build none_move_base_local`，`none_move_base_msgs`、`none_move_base_common`、`none_move_base_local` 全部成功。
46. 在 `README.md` 新增“安全门控（Hold-to-Run）”章节，记录参数、默认策略和快速回退步骤。
47. 在 `Debug.md` 新增“手柄门控调试”和“门控快速回退”章节，补充现场排障与回滚流程。

## 2026-03-30

48. 新增 `MPC-D-CBF改进方案.md`，记录基于 `/home/a/scout_ws/src/MPC-D-CBF` 思路改造 `none_move_base_local` 的迁移方案。
49. 在方案文档开头补充“协作原则”和“目录”章节，统一文档风格。
50. 明确改造策略为“结构借鉴 + 算法核心迁移 + 接口适配”，不采用整仓直接复制。
51. 明确新增三包建议：`none_move_base_obs_param`、`none_move_base_local_map`、`none_move_base_local_planner`，并保留 `none_move_base_local` 作为最终执行与安全门控层。
52. 补充分阶段实施计划、验收标准、风险与回退策略，确保可调试、可回退、可并行验证。
53. 根据对 `/home/a/scout_ws/src/MPC-D-CBF` 实码核查，确认其原始控制器为差速/独轮车模型，控制量为 `v + omega`，不适合直接用于当前全向麦轮底盘。
54. 将 `MPC-D-CBF` 方案收紧为“只迁求解器核心，不迁 `controller.py`、`local_map`、`obs_param` 原始运行链”。
55. 调整目录规划：第一版只建议新增 `none_move_base_local_planner/`，`none_move_base_local_map/` 和 `none_move_base_obs_param/` 改为后置可选项。
56. 调整运行链路：第一版 `none_move_base_local_planner` 直接复用 `/none_move_base/global_path`、`/amcl_pose_tf`、`/odom`、`/scan_full_filtered`，输出 `/none_move_base/local_planner_cmd` 给 `none_move_base_local`。
57. 在方案文档中明确第一版模型约束为全向 `vx + vy + wz`，并要求求解失败时零速与可回退到当前 tracker。
58. 在 `README.md` 新增“局部算法升级路线”章节，明确当前稳定基线仍是 tracker，MPC-D-CBF 仅作为下一步局部算法升级路线。
59. 新增 `none_move_base_local_planner/` 包骨架（`package.xml`、`CMakeLists.txt`、`src/mpc_dcbf_planner_node.cpp`），先落地第一版输入输出接口。
60. 在 `none_move_base_local_planner` 中固定第一版接口：
	- 输入：`/none_move_base/global_path`、`/amcl_pose_tf`、`/odom`、`/scan_full_filtered`
	- 输出：`/none_move_base/local_planner_cmd`、`/none_move_base/local_planner_state`
61. 新增 `none_move_base_bringup/config/local_planner_mpcdcbf.yaml`，统一第一版 local_planner 参数入口。
62. 新增 `none_move_base_bringup/launch/include/navigation_core_mpcdcbf.launch`，用于并行拉起 `none_move_base_local_planner + none_move_base_local + none_move_base_global + none_move_base_navigation`。
63. 在 `navigation_core_mpcdcbf.launch` 中显式设置 `none_move_base_local/local_mode=mpc_dcbf`，保证新入口默认走 local_planner 控制量。
64. 扩展 `none_move_base_local/src/local_controller_node.cpp`，新增 `tracker/mpc_dcbf` 双模式：
	- 新增 `local_mode`、`local_planner_cmd_topic`、`local_planner_state_topic`、`local_planner_cmd_timeout` 参数
	- 新增对 `/none_move_base/local_planner_cmd`、`/none_move_base/local_planner_state` 订阅
	- `mpc_dcbf` 模式下使用 local_planner 控制量，但继续保留最终速度门控
65. 更新 `local_controller.yaml`、`local_controller_hw_debug.yaml`、`local_controller_ridgeback_sim.yaml`，补充 local_planner 相关参数并保持默认 `local_mode=tracker`。
66. 定向编译验证通过：执行 `catkin build none_move_base_local_planner none_move_base_local`，相关包全部成功。
67. 新增 `none_move_base_bringup/launch/parallel_debug_mpcdcbf.launch`，提供一键 mpc 版总入口，避免手工拼装 include 层 launch。
68. 在 `Debug.md` 新增 “MPC 一键入口” 章节，补充一键启动命令和参数覆盖方式。
69. 在 `Debug.md` 新增 “MPC 第一轮联调检查单”，覆盖 4 类检查：话题连通、模式切换、超时保护、门控生效。
70. 将 `none_move_base_local_planner/src/mpc_dcbf_planner_node.cpp` 从占位比例跟踪器改为可运行的 Holonomic MPC 最小核，状态量固定为 `x,y,yaw`，控制量固定为 `vx,vy,wz`。
71. 在 `mpc_dcbf_planner_node` 中显式接入 `/odom`（平滑项种子）和 `/scan_full_filtered`（简化静态障碍约束），不再仅做预瞄比例控制。
72. 新增最小 MPC 关键机制：
	- 有限时域滚动预测（`mpc_horizon_steps`、`mpc_dt`）
	- 控制采样搜索（`sample_vx_count/sample_vy_count/sample_wz_count`）
	- 代价项（路径/朝向/终端/控制/平滑/障碍）
	- 约束与失败保护（硬距离阈值、`mpc_solve_failed` 零速回退）
73. 更新 `none_move_base_bringup/config/local_planner_mpcdcbf.yaml`，将占位参数 `k_x/k_y/k_yaw` 替换为 MPC 参数集与障碍约束参数。
74. 定向编译验证通过：执行 `catkin build none_move_base_local_planner`，相关依赖包与目标包全部成功。
75. 进入第二步“最小障碍约束联调与参数收敛”，新增实机调参配置 `local_planner_mpcdcbf_hw_debug.yaml`。
76. 新增仿真调参配置 `local_planner_mpcdcbf_sim_debug.yaml`，与实机参数分离，降低联调切换成本。
77. 在 `Debug.md` 新增“MPC 第二步：最小障碍约束联调与参数收敛”章节，补充启动命令、最小检查项、参数收敛顺序和完成判定标准。
78. 在 `none_move_base_local/src/local_controller_node.cpp` 为 `mpc_dcbf` 模式补齐最终执行层安全兜底：对 `local_planner_cmd` 增加末端限幅、死区处理、激光停障复核与加速度限幅，避免求解器输出直接裸下发。
79. 新增 `mpc_dcbf` 模式阻塞状态 `mpc_blocked_by_scan`，用于区分“规划器无解”和“执行层停障拦截”两类问题，提升在线诊断可读性。
80. 更新 `phase1_smoke_test.py`，新增 `/hf_platform/joy` 发布并持续置位 `buttons[5]`，使默认开启门控配置下的烟雾测试可自动通过。
