# 规划与 Habitat 执行一致性源码审计（2026-10-15）

本轮只读审计源码与现存实验记录，没有修改算法、参数或启动新实验。以下路径相对于 PRE_MAP_VLN。

## 审计对象和证据边界

- 冻结对照：`outputs/exploration_baseline_restore_504/source_snapshot`、该实验的配置、启动脚本与 `execution_contract_audit.json`。
- 当前文件：`runtime/falcon_selected_yaw_checkpoint`、`scripts/run_habitat_falcon.py`、`ros_ws/src/pre_map_bridge/scripts/file_bridge_node.py`。这些文件部分存在未提交修改，目录名不代表它仍是最初验收快照。
- 现存原始记录：`outputs/exploration_oct12_stall_state_retry`。
- 额外查看了 `outputs/exploration_baseline_replay_504_20261015` 的已有记录。读取时仍为 `.bag.active`，仅约 247 秒、1237 帧；不作为完整504秒结果。
- A/B、single_selection、interface_fix_v2 等失败实现和原始记录已清理，只能引用 `docs/EXPLORATION_FAILED_TRIALS_SUMMARY_20261015.md`，不能冒充重新逐帧验证。

## 1. 已确认：滚动重规划会提前截断旧轨迹，缺少等待新轨迹成功的交接保证

冻结对照 `exploration_fsm.cpp:387` 附近，在调用规划器之前发布 `replan=0`。
同一快照 `traj_server.cpp:234` 的 `replanCallback` 收到0后，将执行期限设为当前轨迹时刻加0.5秒，再与原期限取最小值。
因此即使新规划超时、失败或随后轨迹检查被拒，旧轨迹也已被截短。

`exploration_fsm.cpp:397` 规划失败仅设置 `static_state_=true` 并继续重试，没有将“旧轨迹还会走多久、实际何时停稳”与下一次规划起点同步。
非静止分支从原 B-spline 预测未来起点，而服务端执行期限可能已被缩短。两端没有统一的接管确认。

`exploration_fsm.cpp:535` 还明确要求 `now-start_time > 0` 才发布新轨迹：迟于标称起点是代码行为，不只是偶发计算抖动。
冻结对照的既有执行审计记录319次发布全部晚于标称起点；时间误差会有多大、是否导致具体房间遗漏仍需隔离验证。

当前 selected_yaw 文件保留了上述关键分支。这是源码确认的连续性隐患，但并不等于每次滚动重规划都会卡死。

## 2. 已确认：两种碰撞入口对下一次起点的处理不一致

冻结对照 `exploration_fsm.cpp:972` 的 `safetyCallback` 检测到规划轨迹碰撞后发布立即停止并进入 PLAN_TRAJ，却没有设置 `static_state_=true`。
若此前正在执行，下一轮仍可能从旧轨迹的未来位置规划，而非停止后的真实位置。

相反，Habitat `/preview/motion_blocked` 反馈分支会设置 `static_state_=true`。
当前 selected_yaw 的 `safetyCallback:1010` 与反馈分支约78行仍有这一差别。

此外，停止服务端冻结的是曲线上的命令位置；服务端没有实际 odometry 位置输入来保证这个点等于受阻后的真实位置。
冻结对照既有审计11次停止检查的误差中位数0.241m、最大0.562m；审计本身注明了约100ms与桥接延迟的时间对齐限制，因此不能把全部误差都归到单一分支。

## 3. 已确认：搜索预算耗尽会向上传播为不可达并影响目标优先级

冻结对照 A* 超时、资源耗尽、open set耗尽统一返回 `NO_PATH`（例如 `astar.cpp:219`）。
`path_cost_evaluator.cpp:133` 将失败转换成 `1000 + 欧氏距离`，上层再把高代价当成不可达。
`exploration_manager.cpp` 的候选循环会为这类目标调用 `blockViewpoint(...,20.0, "active frontier is outside current FREE-space component")`。

这里的日志措辞不能证明候选真的处于另一个连通分量：现存这个分支直接依据搜索代价，代价也可能来自超时。本次没有在这些源码中找到独立的当前位置 FREE 连通分量预过滤实现来支持网页端的这一具体说法。

当前 selected_yaw 的 A* 诊断可记录 timeout/exhausted/resource_limit，但返回给调用方的类型仍是 NO_PATH，因此增加日志尚未消除上层语义合并。

## 4. 部分成立：起点逃逸规则确实特殊，但不能据此断定0.18m导致锁死

A* FREE_ONLY 允许起点所在 UNKNOWN 体素，其余 UNKNOWN/OCCUPIED 仍拒绝。
ESDF和已记录接触点约束在起点0.15m内允许近似不恶化：ESDF可降低0.01m，接触点距离可降低0.005m；离开此范围后要求完整净空。
因此它不是严格单调远离障碍的专用恢复规划器，也不保证一定存在能接上正常安全区的路径。

直线路径代价检查先做占据射线检查，缺少与A*相同的起点 UNKNOWN 体素放行，存在规则差异；但直线失败后还会调用A*，不能仅凭差异断言整轮必然失败。

重要版本区别：冻结对照 A* 源码默认净空0.18m，但实验 `config.yaml` 实际覆盖为0.07m；当前 selected_yaw 配置为0.01m。
不能把源码默认0.18m直接当成所有实验的生效值。
0.1317m接触距离与0.15m放行半径的比较本身也不能证明无路可走：向外运动可能在放行范围内就达到0.18m，必须检查真实方向上的占据/ESDF。

213秒受阻、320次反馈及38个远处目标轮换，在清理摘要中有明确记录；对应失败版本原始状态已删除，本轮无法重新核实0.1317m数值或证明在线ESDF阻断全部退出方向。
`scripts/audit_stall_egress.py` 自身也明确说明只验证场景网格和记录接触点，不验证在线占据/ESDF。

## 5. 需要纠正：无碰撞600秒实验不能用1208个小位移采样证明持续活锁

重新读取 `exploration_oct12_stall_state_retry/live_trajectory.jsonl` 的2997帧并累计实际位置：

| 指标 | 本轮重算/读取 |
|---|---:|
| 实际路程 | 430.477m |
| 碰撞拦截帧 | 0 |
| 相邻位移不足1cm的间隔 | 1208（既有审计） |
| 连续相邻位移不足1cm的最长段 | 2.6秒，按sequence/5Hz估计 |
| 日志 Total time too long | 314 |
| 日志 Short path | 187 |
| 日志 Plan fail | 0 |

这能支持反复规划停顿、短路径频繁出现；不能支持机器人长时间一直停在同一点。2.6秒的统计定义为相邻步长阈值，不等于在固定半径内停留或区域循环；后两项需要另算。
`Total time too long` 是FSM整轮规划超过重规划预算的日志，不等于单次A*搜索超时。
覆盖没有完成也可能是反复访问、视角或目标选择问题，仍不能排除Frontier/HGrid层的影响。

## 6. yaw历史缺陷已修，但基线必须按真实入口区分

- `exploration_round6_extended_rerun_20261014/run_simulator.sh:9` 明确使用 `--legacy-motion-heading`。
- `exploration_yaw_consistent_trial/run_simulator.sh:9` 明确使用 `--use-planner-yaw`。
- `exploration_baseline_restore_504/run_simulator.sh:9` 使用 planner yaw；2518条轨迹记录全部为 `yaw_policy=planner`。
- `exploration_threshold_double_half_504` 的2518条记录也全部为planner。
- 当前 `run_habitat_falcon.py` 保留规划yaw执行与显式legacy开关。

所以两套旧基线确实不等价，但保留的10月15日冻结对照没有重新退回移动方向yaw。被删除实验无法逐一重查。

## 7. 未完成目标：确认存在冷却和重选，尚未证明覆盖责任丢失

源码会在局部搜索失败或实际受阻后暂时屏蔽目标，也会重算下一轮访问顺序；这足以改变房间访问时间。
但屏蔽20秒不是删除Frontier，`clearExplorationData()` 清理临时规划数据也不等于地图覆盖任务被标记完成。
现存源码还保留历史位置恢复与观测结果处理，不能描述成完全没有未完成任务处理。
要证明某个区域任务丢失，必须跟踪对应Frontier/未知体素是否仍在、为什么退出候选、是否被错误宣告完成。目前证据不足。

## 其他已核实的接口问题

当前桥接器对最新位姿周期性使用 `rospy.Time.now()` 发布，frame_timer也使用读取时刻而非采集时间。
桥接器读取 `velocity_world`，缺省为零；当前模拟器构造的frame_state未提供该字段。因此默认反馈速度为零。
冻结对照既有审计也记录 `odom_velocity_max=0`。这与真实移动不一致，但单独修正的覆盖收益尚未验证。

## 结论与下一次验证范围

网页端提出的一致性审计方向有实质源码依据。证据最扎实的是旧轨迹提前截断、碰撞入口状态转换差异、失败类型传播混淆，以及反馈速度/时间戳问题。
“0.18m起点规则是最大根因”“无碰撞600秒实验已经证明活锁”“覆盖责任已经丢失”“Frontier算法可以排除”均超过当前证据。

下一次应冻结同一份源码和配置，只跟踪一段明确受阻事件：实际采集位姿/速度、预测起点、旧轨迹停止点与期限、新轨迹起点/发布时间、A*失败类型、同一候选冷却与再次进入。起点附近快照应围绕实际机器人位置，并覆盖A*、轨迹检查和占据/ESDF规则；仅围绕远处轨迹拒绝点采样不足以证明当前位置不可退出。
