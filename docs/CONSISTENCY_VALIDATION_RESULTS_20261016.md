# 执行、搜索与候选机制验证结果

R0、A、B各完成两次504秒正式运行。C完成两次完整候选链诊断，未修改算法。A未通过独立系统验收，因此按“只合并已验证变更”的约定，不运行A+B组合，不晋升任何新基线。

|组别/重复|已知体积m³|路程m|移动时间s|最长小位移段s|碰撞拦截帧|2号旁内部帧|9号内部帧|
|---|---:|---:|---:|---:|---:|---:|---:|
|R0/1|1184.024|445.56|337.45|5.81|79|0|0|
|R0/2|1007.338|393.01|288.64|17.82|206|0|49|
|A/1|660.831|180.27|170.91|252.26|1|0|0|
|A/2|1108.146|378.00|364.85|6.80|2|0|69|
|B/1|820.027|291.67|217.35|188.53|974|0|1015|
|B/2|992.267|425.81|307.20|33.63|465|0|0|

## 机制与系统结果

- A：真实反馈、服务端接受后切换、规划失败保留安全旧轨迹、两类停止后的新鲜实际起点均通过短测。但完整运行一轮出现252秒停滞，不能验收为可用修复。录包证明新增时长条件反复拒绝位置2.09秒/yaw1.58秒的候选；这是本次实现引入的兼容性问题，不能据此归因原基线。另一轮能够进入三楼，说明短测与单轮成功都不足以证明稳健。
- B：真实A*构造的SUCCESS/TIMEOUT/RESOURCE_LIMIT/EXHAUSTED/INVALID_START/INVALID_GOAL及目标冷却、代价传播共11项检查通过。第一轮完整运行5303次避免了基于未建立可达性的冷却，但仍在9号附近受阻约188秒。语义正确不等于执行恢复或覆盖改善。
- C：历史949/780/169及9号68帧、2号旁0.7175米已独立核实。新诊断两轮分别有2/27个内部候选完成评分，一轮选中过内部目标；存在普通候选压制靠障碍候选，但未证明被压制者安全可达且可补齐有效观测。因此不修改>4、收益64或类别优先。

## 可复核材料

- 汇总与分层指标：`outputs/consistency_validation_20261015/final_report.json`、`comparison.csv`。
- A门槛：`outputs/consistency_A_final_gate_90_20261015/execution_gate.json`、`feedback_ros_test.json`及`outputs/consistency_validation_20261015/A_server_final/server_test.json`。
- B门槛：`outputs/consistency_validation_20261015/search_test/search_test.json`、真实C++测试源码`scripts/consistency_search_test.cpp`。
- C诊断：`outputs/consistency_validation_20261015/C_mechanism_diagnosis.json`、历史核验JSON、R0完整候选日志及逐候选生命周期。
- 每轮目录保存源码快照、配置、有效ROS参数、镜像/二进制与源码身份、采集轨迹、录包和地图统计。独立变更patch在`runtime/consistency_validation_20261015/review`。

## 因果边界

- Two repeats are descriptive evidence, not a statistical significance claim.
- B writes per-search status logs in addition to common execution diagnostics; overhead not independently benchmarked. Performance differences cannot be cleanly attributed to semantics alone.
- Historical full upstream environment identity was not archived; comparisons use one isolated container with per-run effective params and hashes.
- Candidate diagnostic runs are fresh trajectories, not exact historical-state counterfactual replays.
- Known FREE+OCCUPIED volume and fixed region entry are not ground-truth completeness or containment certification.
- One earlier A run was interrupted around136s and excluded. Development failures excluded from full-run comparison.

## 下一步的明确定位

当前最具体的A阻断是接管校验与基线已有位置/yaw时域不一致的处理，需扩展真实服务端测试覆盖该输入，并明确较短轨迹终点保持语义，再单独验证。不能简单去掉所有检查或放宽净空。B保留为已验证语义候选，不把它当成完整探索通过。C仍需同状态证据才能更改规则。原验收版本保持原样。
