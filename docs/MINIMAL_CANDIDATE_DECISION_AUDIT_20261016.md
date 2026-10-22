# 最小候选观测决策审计：一、二楼

## 结论

本次仅核查最新A3_observation_lifecycle的既有bag、事件、冻结源码及历史C候选日志，没有修改规划器、启动仿真或把GT送入在线计算。

**plan209的指定候选确实进入最终评分，并因utility低于另一候选而未胜出。** 363是在线函数记录的UNKNOWN体素数，不是离线GT表面积；其yaw、体素键集合和候选ID没有保存，无法证明它当时的朝向能覆盖二楼普通区域6.42m²缺口，更无法独立重算363是否正确。

历史C保存了真实计算后的候选yaw及部分ID链，但缺失UNKNOWN集合、完整最终比较代价和一致在线地图状态。不能用历史C的地图或yaw替代最新plan209。当前不足以提出候选生成、收益公式或排序策略的算法修复。

## 两区域归因表

|对象|已有正证据|朝向/收益估计|筛选/最终排序|执行归因|裁定|
|---|---|---|---|---|---|
|二楼普通区域6.42m²；plan209指定候选|历史位点可观测；本轮该xyz的在线gain363、utility57.519有完整记录|未选候选yaw及UNKNOWN集合缺失；不能验证其与这6.42m²的实际交集|已进入最终评分，未在门槛/代价过滤中被淘汰；被utility58.373的候选压过|该未选候选没有轨迹；不能归因于其服务器拒绝或碰撞|直接未选原因落在最终排序；排序是否合理、朝向/收益是否正确仍未定|
|9号附近5.94m²；任务82/轨迹98|选中yaw=-0.027912；实际执行后SceneGeometryFeedback中止|该目标观测姿态与最终B类集合交集仅0.16m²；不是5.94m²全量收益|后来仍有候选评分，但其yaw/集合未保存，不能确认有效替代视点被错误压制|任务82执行受阻已确认；不能将其推广为整个缺口的根因|这一任务的断点在执行；其余区域机会仍卡在候选朝向/收益至选择之间的证据缺口|

两块表面集合共12.36m²，不是同一缺陷的已证实损失。已选姿态的0/0.16m²潜在覆盖也不等同于真实新增观测或碰撞因果损失。沿途视角与目标视角需要区分。

## plan209：能还原到哪里

原始`run.log`按ROS时间落入manager plan209区间，保留99条完整评分记录。FSM `planning_start`事件的plan字段仍为208，manager `plan_begin`后为209；审计按实际区间及manager编号关联，不能要求两个入口的数字天然一致。

|比较项|指定未选候选|实际胜出候选|
|---|---:|---:|
|位置（日志精度）|(2.07,5.42,3.68)|(4.31,8.89,4.24)|
|在线gain（唯一UNKNOWN体素数）|363|470|
|confidence|1.0000|1.0000|
|travel|1.786|3.609|
|onward|2.525|2.442|
|utility|57.519|58.373|
|保留下来的99条评分中名次|2|1|
|yaw|未记录|-1.186145（最终选择事件）|
|稳定候选ID|未记录|未记录；后续有任务130/轨迹161|

分差0.854，约为指定候选utility的1.48%。源码采用严格`utility>best_utility`更新胜者；指定候选已经通过gain及路径代价过滤，否则不会输出这条完整评分。因此这一次不应写成`>4`、64门槛、类别优先或A*失败直接淘汰，也没有证据表明排序实现偏离其数值规则。

评分为`gain*reliability/(2+travel+onward+floor_cost)`，未完成任务邻近候选可乘1.15。本例两条记录与上述比较相符；日志数值经过舍入，不据此反推精确中间代价或未知集合。

实际选择事件：task130、目标(4.31006562233,8.88943533698,4.24089253188)、yaw=-1.186145。随后冻结候选→发布轨迹161→server_accept→server_switch→observation_executing可关联；不存在“指定未选候选发布后被服务器拒绝”的链。关联靠同一运行的plan、task及trajectory，不捏造缺失的candidate_id。

未知的关键部分仍是：指定位置的真实候选yaw、参与363计数的体素键、当时占据地图/Frontier集合，以及它们与离线缺口的交集。离线可见上界4.87m²不能替代这些在线证据。

## 记录是否足够

### 最新验收

- `manifest.json`为`light_execution_and_search`。decision_trace共5734条，candidate事件为0；五份关键源码快照哈希与manifest一致。
- `ObservationDecision`只打印xyz、gain、confidence、travel、onward、utility。源码确实调用了`viewEvent`传递yaw和candidate_id，但候选诊断开关关闭，表达式不会写出。不能因为源码有字段就认定运行时保存了字段。
- bag里没有候选决策/预测UNKNOWN集合话题。`/planning_vis/frontier`虽有182857条Marker，实际仅包含`frontier_id`及`frontier_visble_num`文本命名空间；没有候选视线端点。FSM的视线绘制使用独立VIEWPOINT发布器，该话题未录入本bag。plan209附近没有可恢复指定候选yaw的线段。
- 轨迹yaw和PositionCommand仅表示实际选中/生成的运动，不是所有未选候选。最终地图、原始深度可供重新建图，但不包含本轮当时的完整同步处理顺序及Frontier状态，不能称为同状态重算。

### 历史C完整日志

实际扫描`consistency_R0_504_r2_20261015`及`r3`的decision_trace：候选事件分别5456676、9381681条；scored分别8012、10063条。未发现JSON坏行或显式records_dropped记录；这不证明所有未设计的淘汰原因都已覆盖。

- `candidate_id`在采样时分配，随Viewpoint传递，可在同一运行内关联生成、visibility、类别、gain、配额、去重、score_begin及scored等阶段。
- **generated阶段yaw=0是占位，不能视为生成结果。** 真正朝向在其后基于`frontier.filtered_cells_`计算平均yaw，在visibility事件才出现。
- 例：r2候选1224835从generated的0转为visibility的-0.420827，随后ordinary_admitted、gain453、quota_admitted、score_begin、scored保持同一ID。另两个历史ID链保存在机器产物中。
- gain事件保存count/threshold；scored保存gain/utility，但没有UNKNOWN键集合、集合哈希、完整travel/onward分解或显式“未胜出于谁”。生成/几何拒绝、可见数阈值、普通/靠障碍类别、gain阈值、配额/去重及部分代价拒绝有记录，但不能称为全部终态原因完整可复核。
- 前置参考目标和pending目标是后插入评分队列的，`observation_ids.resize(...,0)`给它们0；这不是所有候选都有非零稳定身份。最新`observation_selected`事件也没有携带最终candidate_id，不能端到端贯通到task/trajectory。
- 历史样本的map_stamp有0值，且没有可恢复的同状态占据/Frontier快照。跨运行候选ID不能复用，坐标相同也不能证明在线状态或yaw相同。

## 相机光轴与坐标

读取冻结采集源码：

`R_F_opt = S_HF * R_H_camera * diag(1,-1,-1)`，其中`S_HF=[[0,0,-1],[-1,0,0],[0,1,0]]`。实际光轴是`R_F_opt[:,2]`；候选yaw约定的正向是`[cos(yaw),sin(yaw),0]`。manager的refined_view方向和`evaluateObservation`的Rz(yaw)同样使用该约定。

2518帧逐帧比较实际光轴与同帧记录的executed_command_yaw：中位夹角约0.00000148°、p95约0.00000540°；最大1.68017°，另有1.13878°及0.30835°异常帧。未发现系统性的90°、180°或镜像错误；不能把它表述为每帧严格零误差，更不能由已执行命令推回未选候选yaw。

离线NPZ索引检查确认普通区642个、9号附近594个参考样本均属于8cm未匹配、几何复核且本轮未见集合。只用这些集合评价，不向在线函数传入GT。显示`-X/Y`不参与几何运算。

## 唯一建议方向：补齐公共诊断关联，不修改决策算法

目前有证据支持的是**候选诊断契约缺口**，而非候选生成/排序算法错误。建议仅在未来需要采集时补齐以下记录；本次未实施、未开启采集：

1. 在`sampleViewpoints3D`计算avg_yaw之后记录有效朝向；生成占位阶段明确`yaw_valid=false`。沿用run UUID+candidate_id+sampling_id，补充约束/地图版本。
2. 在`evaluateObservation`返回处利用已经生成的UNKNOWN键，按排序后的int32三元组计算集合ID，并将集合保存到独立可去重侧文件。记录分辨率、地图原点、坐标系、地图epoch前后值及积分时间；epoch变化时明确非一致读。哈希只能校验已保存集合，不能单靠哈希恢复集合，也不能证明地图一致。
3. 评分前后记录同一ID的position、yaw、gain、confidence、travel、onward、floor_cost、pending倍率及utility；拒绝写filter_reason，通过但未胜出写winner_id及比较结果。参考/pending插入项须有非零身份及origin字段。无需保存GT或GT面积。
4. 最终胜者将candidate_id/set_id显式传到最终候选、task、trajectory；路径恢复若改目标，登记新的实际来源，不沿用旧ID伪装同一候选。保持普通重规划的现有执行语义。

开销：历史全量候选日志分别1.831/3.141GB，折算约3.63/6.23MB/s，因此不建议直接恢复所有逐生成事件。只按每条500B估算8012/10063条scored摘要，约4.01/5.03MB每504秒；若每个集合都恰为363个int32三元组，二进制键约34.90/43.83MB，另加索引和元数据。这只是容量估算，363不是全局平均值，未测实际CPU延迟。集合遍历/哈希为O(n)，利用现成vector、异步有界队列、去重并保留丢弃计数；若按区域/阶段限流，要明确日志不完备，不能用缺行断言没候选。

### 对应最小回归（仅方案，本次未运行新规划测试）

一项诊断透传回归即可：用两个受控候选及已知UNKNOWN键集合，走生产日志接口，核对非零ID与set_id在生成后有效yaw、评分、winner/loser、task和trajectory之间一致；集合能由侧文件恢复且count/hash一致。覆盖参考/pending来源和拒绝后不串ID。开关开启/关闭时选中位置、yaw、gain、utility完全相同；模拟日志队列满只影响诊断且有丢弃计数。它只证明记录不改变决策，不证明历史363计数或候选朝向正确。

本次按缺证条件在此收口。不提出第二个算法修复方向，不启动504秒，也不以历史C或GT替代本轮在线地图。

## 可复核产物

`outputs/consistency_validation_20261015/minimal_candidate_audit/`：

- `evidence_inventory.json`：三份日志字段/阶段计数、源码哈希及光轴检查。
- `plan209_score_records.json`、`plan209_events.json`：原始评分及事件。
- `bag_inventory.json`：所有话题、Marker命名空间和指定时间窗检查。
- `historical_C_identity_examples.json`：三个历史稳定ID链；`*_ROI_events.json`保存相应区域候选阶段事件，不作为最新状态替身。
- `minimal_audit_checks.json`：离线集合索引、光轴异常帧、开销算术。

新增脚本仅做只读审计：`audit_candidate_evidence_minimal.py`、`audit_candidate_marker_bag_ros.py`、`finalize_minimal_candidate_audit.py`。
