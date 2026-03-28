# NoneMoveBasePlan 修改日志

## 2026-03-27

1. 新增 `src/README/NoneMoveBasePlan.md`，记录去掉 `move_base` 后的平行导航架构方案。
2. 在 `src/README/NoneMoveBasePlan.md` 开头新增“协作原则”章节。
3. 约定新导航体系采用完全平行结构，不修改现有 `hf_navigation`、`task_communication`、`dms_to_robot.py`、`hf_bringup` 目录和代码。
4. 记录推荐的新目录根为 `src/platform/none_move_base_nav/`，并拆分为 bringup、msgs、common、adapter、global、local、navigation 七个包。
5. 修正文档中“6 个包”为“7 个包”的表述错误。
6. 在 `src/README/NoneMoveBasePlan.md` 中补充七包结构细化总表，包含：
   - 每个包的 launch 设计
   - 每个包的 topic/action/service
   - 每个包的 catkin 依赖和包间依赖方向
   - `parallel_debug` 与 `cutover_compat` 两种模式的接口映射
7. 在 `src/README/NoneMoveBasePlan.md` 开头新增“目录”章节，便于快速定位文档内容。
8. 在 `src/README/NoneMoveBasePlan.md` 开头新增“修改方案摘要”章节，只保留影响架构决策的核心修改点。
9. 根据新确认原则，将方案从“完整平行新业务链”收敛为“只替换 `/move_base`”。
10. 重写 `src/README/NoneMoveBasePlan.md` 的目录和主体结构，删除原 7 包平行 adapter/navigation 方案。
11. 将第一阶段推荐结构收敛为：
   - `none_move_base_bringup`
   - `none_move_base_move_base_compat`
   - `none_move_base_global`
   - `none_move_base_local`
   - `none_move_base_common`
12. 明确当前旧链只有 1 层 action，第一阶段新链也保持 1 层 action。
13. 明确 `dms_to_robot.py`、`task_communication`、`hf_bringup`、定位链、`charge` 微调逻辑保持不变。
14. 将文档的“修改方案摘要 / 目标 / 核心原则 / 现状结论 / 替换边界 / 包结构 / Launch 设计 / 接口表 / 依赖关系表 / 新链路拓扑 / 已确认决策 / 待确认问题”全部改写为“只替换 `/move_base`”版本。
15. 根据新决策，再次将方案从“只替换 `/move_base`”改为“彻底脱离 `move_base` 体系”。
16. 明确旧 `task_communication` 因直接依赖 `MoveBaseAction`，不能进入新运行链。
17. 恢复并重写 7 包结构：
   - `none_move_base_bringup`
   - `none_move_base_msgs`
   - `none_move_base_common`
   - `none_move_base_adapter`
   - `none_move_base_global`
   - `none_move_base_local`
   - `none_move_base_navigation`
18. 明确外部接口继续保持 `/obsNavigation_in`、`/obsNavigation_out`、`/inquiryNavigation_out`，但内部不再保留 `/move_base`。
19. 明确内部只保留 1 层 `NavigateTask.action`，不再保留 `NavigatePose.action`。
20. 新增“充电链迁移原则”，区分“保留业务语义”和“照搬旧代码”，并明确 `StrollToGoal / RotateInPlace / ReCharge / StrollToCharge` 的执行原语必须迁走。
21. 重写最终拓扑图为：`dms_to_robot.py -> none_move_base_adapter -> none_move_base_navigation -> global/local -> /hf_platform/nav_vel`，并保留独立的 `/hf_platform/crg_vel` 充电保持链。
22. 在方案文档中新增“算法选型结论”章节。
23. 明确第一阶段全局算法选择为“二维占据栅格 A* + gradient traceback + path prune”。
24. 明确第一阶段局部算法选择为“Holonomic TEB”，并强调这是算法选型，不代表继续使用 `move_base`/MBF 运行结构。
25. 将 “A* + Holonomic TEB” 写入实施顺序、已确认决策和待确认问题，收敛第一阶段技术路线。
26. 明确最终路线固定为方向 B：采用自有 7 包运行时，不直接落地 `locomotor`。
27. 在文档末尾新增“为什么不用 MBF”章节，明确 MBF 属于过渡期兼容壳，不符合“彻底脱离 `move_base` 体系”的最终目标。
28. 在文档末尾新增“项目落地时间估计”章节，给出 6 到 10 周的保守排期，以及 8 到 12 周的现场联调版排期。
29. 将耗时原因拆解为：局部控制参数整定、charge 特殊流程、旧业务语义迁移、接口边界收敛、真机回归验证。
30. 继续完善方案文档，新增“第一阶段文件清单”章节，按 7 个包列出建议首批文件。
31. 新增 `NavigateTask.action` 草案，补充 goal/result/feedback 字段、`error_code` 和 `phase` 建议。
32. 新增 `parallel_debug` 最小启动图，明确第一阶段先跑内部导航闭环，再接 DMS 和特殊行为。
33. 新增“第一阶段验收标准”，把可继续迁业务的门槛收敛成全局规划、局部控制、真机到站、状态机闭环、老链不受影响 5 类标准。
34. 继续完善内部接口定义，新增 `NavigationStatus.msg` 草案，用于内部状态广播和 adapter 组装外部反馈。
35. 新增 `PathTrackingState.msg` 草案，明确 local 到 navigation 的状态反馈字段和 `tracking_status` 建议值。
36. 新增 `BehaviorState.msg` 草案，用于区分任务级状态和行为级状态。
37. 新增 `ReloadStationPoses.srv` 草案，提前固定站点表热更新接口。
38. 调整文档目录和章节编号，把消息与服务接口定义放到 `NavigateTask.action` 之后、`parallel_debug` 之前。
39. 将主方案正式拆成两个阶段：第一阶段“3 天先动起来”，第二阶段“再补齐旧业务接口和特殊流程”。
40. 将第一阶段局部算法从 `Holonomic TEB` 收敛为“轻量 Holonomic Path Tracker”，把 `Holonomic TEB` 明确后移到第二阶段。
41. 重写实施顺序、已确认决策、待确认问题和工期估算，使文档与“两阶段落地”一致。
42. 调整第一阶段文件清单，明确 `adapter` 和 `cutover_compat` 属于第二阶段内容。
43. 新增 `src/README/NoneMoveBaseReservedInterfaces.md`，单独记录两阶段方案中保留下来的外部接口、内部接口、Twist 语义约束和 `NavigateTask.action` 保留约束。
44. 再次全盘整理 `src/README/NoneMoveBasePlan.md`，把当前开工版本收敛为“第一阶段固定 A* + 轻量 Holonomic Path Tracker，第二阶段只保留局部控制升级候选，不锁死实现”。
45. 在 `已确认决策` 中加入接口保留文档路径，明确后续完善时优先保持接口稳定。

## 2026-03-28

46. 新增 `src/README/此方案的原因.md`，单独记录第一阶段采用当前方案的原因。
47. 将 `src/README/NoneMoveBasePlan.md` 中“为什么不用 MBF”和“项目落地时间估计”迁移到 `src/README/此方案的原因.md`，使主方案文档只保留方案本体。
48. 在 `src/README/此方案的原因.md` 中补充“为什么第一阶段不用完整 TEB，而采用 A* + 轻量 Holonomic Path Tracker”的说明。
49. 重排 `src/README/NoneMoveBasePlan.md` 的目录、章节编号和相应子章节编号，消除迁移后的编号错位。
50. 在 `src/README/NoneMoveBasePlan.md` 中补充原因文档路径，明确方案本体与取舍论证分文档维护。
51. 继续完善第一阶段方案，在 `src/README/NoneMoveBasePlan.md` 中补充路径重采样、near-goal、最终 yaw 对齐和 success latch 的终点收敛策略。
52. 同步更新第一阶段实施顺序、已确认决策、第一阶段文件清单和验收标准，使“能到站并稳定停住”成为显式目标。
53. 在 `src/README/此方案的原因.md` 中补充阶段 1 先用 tracker 的另一个原因：可以先把终点收敛语义和接口边界固定下来。
54. 根据 `src/README/tf_flash注意事项.md`，在方案文档和接口保留文档中预留 `/tf_flash` 终点微调扩展点，但明确第一阶段不实现。
55. 在 `NavigateTask.action` 草案中新增 `enable_terminal_adjust` 和 `terminal_adjust_source` 预留字段，用于后续按需接入终点微调。
56. 同步预留 `terminal_adjust` 运行阶段和行为名，避免后续加入高精度终点微调时再改 phase / behavior 接口。
57. 在 `src/README/NoneMoveBasePlan.md` 中补充“非侵入原则”和“独立目录原则”，明确旧目录默认只读、新代码统一放到 `src/platform/none_move_base_nav/`。
58. 在 `src/README/NoneMoveBaseReservedInterfaces.md` 中补充“非侵入实现边界”，明确默认不修改旧源码、旧 launch、旧 yaml，只通过新目录和新 launch 接入。
