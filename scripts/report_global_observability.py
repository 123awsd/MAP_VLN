import csv,json,math,hashlib
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/global_observability';s=json.loads((out/'global_observability_summary.json').read_text());rows=s['threshold_floor_table'];regions=s['regions_2m_tiles_ranked_by_verified_opportunity'];mesh=json.loads((out/'mesh_verification.json').read_text())
for r in rows:
 total=r['current_raw_depth_witness_m2']+r['actual_geometric_visibility_without_depth_witness_m2']+r['historical_mesh_checked_opportunity_m2']+r['unresolved_m2'];assert abs(total-r['unmatched_area_est_m2'])<.0001
for name,data in [('threshold_floor_table.csv',rows),('ranked_regions.csv',regions)]:
 with (out/name).open('w')as f:
  writer=csv.DictWriter(f,fieldnames=data[0].keys());writer.writeheader();writer.writerows(data)
text='''# 官方HM3D表面缺口：观测机会归因（只读）

## 结论

在本次统计的三层室内高度带中，8cm下未匹配参考表面积估计334.26m²。其中106.22m²在本轮没有找到实际观测证据，但有历史深度线索，且替代观察位置通过修复后几何的离散安全及视线复核。7.71m²属于本轮实际可见/测到却未在稠密导出中匹配，其余220.33m²尚不能归因。

**当前证据更支持继续分析观测访问与回访，而不支持把大面积缺口首先归因于深度积分。但未判定部分占65.9%，不能宣布已确定整栋楼的全局主因。** 9号确有执行受阻；不能用这一例代表全部106.22m²。

## 范围与方法

本次复用官方网格均匀面积采样，在原始3D坐标中比较，不做ICP或镜像。原密度1600点/m²，以固定种子独立1/16抽样，约100点/m²，每点面积估计权重0.01m²。以下是面积的蒙特卡洛估计，单项采样标准误粗估sqrt(样本数)×0.01m²，另有匹配和几何近似误差。

**范围是与既有稠密图一致的三层墙体/物体高度带，不是整栋建筑所有网格表面的完整盘点。** XY为X[-8.5,10.5)、Y[-2.5,18.5)，Z分别[-0.005,2.045)、[2.745,4.795)、[5.745,7.795)。地面、天花板和裁剪之外不计入；不能将下表外推为全建筑覆盖率。

最新2518帧全部按精确采集时间匹配，640×480全分辨率深度，fx=fy=320，光学深度0.15–5m。参考点投影到原深度，深度差≤2.5cm作为正测量证据。另对尚无深度证据的缺口，遍历实际相机视锥并查询原始HM3D及14个修复碰撞物体的第一交点，端点差≤2.5cm作为几何可见证据。几何可见不等于像素必然采样到；未命中也不证明永不可见。

替代位点只取R0两轮和历史A3保存的相机位置，不凭空假设房间内部可达。旧R0消息重盖时间戳，≤50ms邻近采集匹配只用作线索：分别获得1231/1184帧，未匹配帧丢弃；历史A3为2518帧。每个B类证据再以修复后场景独立核对视线（端点容差4cm）、26方向8.5cm净空射线、前一采集位置至该点的0.07m离散扫掠执行模型，并排除记录为受阻的帧。

这些是**潜在安全观察机会的有限检查**，不是精确球体碰撞证明、全局可达路径证明或当时在线FREE/ESDF可达性证明。只搜索历史位置，因此B是保守证据量；未找到机会的C仍不可判定。

## 8cm：分层定量表

|高度带|未匹配面积m²|A：本轮有实际可见/深度证据m²|B：本轮未见、历史位点几何复核m²|C：未判定m²|
|---|---:|---:|---:|---:|
'''
for r in rows:
 if r['threshold_m']==.08:
  text+=f"|{str(r['floor'])+'楼' if r['floor'] else '合计'}|{r['unmatched_area_est_m2']:.2f}|{r['current_raw_depth_witness_m2']+r['actual_geometric_visibility_without_depth_witness_m2']:.2f}|{r['historical_mesh_checked_opportunity_m2']:.2f}|{r['unresolved_m2']:.2f}|\n"
text+='''
A类7.71m²中，5.54m²原始深度存在测量吻合；其中2.47m²在最终OCCUPIED体素中心12cm以内，优先解释为稠密导出的抽帧/降采样或表面匹配差异。另3.07m²没有该占据近邻，主要在三楼（2.90m²），是建图链的待核查小集合，尚不能认定积分缺陷：需继续核对输入有效深度、边界裁剪、积分完成时刻及体素状态。最终体素比较容差12cm考虑0.1m体素中心量化，不等同于8cm表面匹配。

A中另2.17m²仅几何视线可见，没有深度吻合；可由像素采样、边缘或几何/深度差异造成，不能直接叫传感器丢帧。

## 阈值敏感性

|匹配阈值|总未匹配m²|A m²|B m²|C m²|三楼未匹配m²|
|---|---:|---:|---:|---:|---:|
'''
for r in rows:
 if r['floor']==0:
  z=next(x for x in rows if x['floor']==3 and x['threshold_m']==r['threshold_m']);text+=f"|{r['threshold_m']*100:.0f}cm|{r['unmatched_area_est_m2']:.2f}|{r['current_raw_depth_witness_m2']+r['actual_geometric_visibility_without_depth_witness_m2']:.2f}|{r['historical_mesh_checked_opportunity_m2']:.2f}|{r['unresolved_m2']:.2f}|{z['unmatched_area_est_m2']:.2f}|\n"
text+='''
三个阈值均显示三楼缺口最大，已找到的历史观测机会远多于本轮实际可见但未匹配的部分；绝对面积对阈值敏感，不能把334.26m²当精确真值。

## 优先区域：按B类面积排序

以下是固定2×2m空间格，并非语义房间或连通缺失块。格内目标计数仅提供关联线索；格外视点也可能观测该格，因此“格内无目标”不能证明缺少有效候选。

|层|原始X范围|Y范围|未匹配m²|B类m²|执行线索|
|---|---|---|---:|---:|---|
'''
for r in regions[:12]:
 clue=f"格内执行任务{r['executed_goal_task_ids_in_tile']}" if r['executed_goal_task_ids_in_tile'] else '格内无执行目标/相机采集'
 if r['first_camera_time_s']is not None:clue+=f"；首次采集{r['first_camera_time_s']:.1f}s"
 text+=f"|{r['floor']}|{r['x_range_m']}|{r['y_range_m']}|{r['missing_area_m2']:.2f}|{r['historical_mesh_checked_opportunity_m2']:.2f}|{clue}|\n"
text+='''
9号所在二楼X[-2,0)、Y[10,12)空间格有6.96m²未匹配，其中5.94m²有历史观察机会证据；此格大于固定卫生间ROI，不能全称为卫生间缺失面积。任务82/轨迹98实际执行后被SceneGeometryFeedback取消，与既有审计一致；格内还执行过任务90，不能概括成此格此后完全未访问。当前分析没有把任务82的碰撞原因推广到其他缺口。

三楼靠Y[-2,4)的多个高优先格没有格内执行目标或实际采集，支持“访问尚未展开”的定位；但是否源于候选生成、评分、访问开始晚或有限504秒预算，还需其对应视点的逐轮决策证据，不能直接修改收益。三楼X[-4,-2)、Y[12,14)直到498.5s才有格内实际采集；任务177末尾因frozen_feedback_stale取消，不能仅凭取消事件判定它造成整个区域缺失。

## 决定

不改A*、服务器、时间预算、Frontier门槛、楼层权重或净空；不启动任何新探索实验。现有证据能排出一批比单个卫生间更大的实际未访问观测机会，并形成很小的建图链待核查集合；尚未完成全网格/所有潜在位点的穷尽分析，不能宣布B或碰撞占全建筑主导。

后续最有价值的具体对象是表中B类最大的三楼格与二楼X[2,4)、Y[4,6)格所对应的历史观察位点：检查本轮是否生成了能看到这些表面的候选，再区分未生成、竞争失败、执行中断及时间预算结束。不是强制机器人进入这些空间格。

## 产物与复核

`outputs/consistency_validation_20261015/global_observability/`：

- `global_observability_summary.json`、`threshold_floor_table.csv`、`ranked_regions.csv`：定量表及逐格事件关联。
- `surface_witnesses.npz`：参考采样位置、面积权重、三阈值距离、各运行深度证据和原始采集帧索引。
- `actual_geometry_visibility.npz`、`mesh_verified_opportunities.npz`、`mesh_verification.json`：视线及替代位点核查；可回查具体帧。
- `final_map_proximity.npz`：最终占据近邻距离。
- `witness_manifest.json`、`export0–3.json`：范围、匹配和时间误差。中间深度数组可由原bag重新导出。

脚本：`export_observability_depth_ros.py`（容器ROS）、`audit_global_observability_ros.py`（主机科学计算）、`verify_observability_mesh.py`、`summarize_global_observability.py`、`report_global_observability.py`。前者只读bag，后者仅离线加载几何，无探索控制、无新传感器渲染。
'''
(root/'docs/GLOBAL_OBSERVABILITY_AUDIT_20261016.md').write_text(text)
(out/'checks.json').write_text(json.dumps({'category_area_sums_passed':True,'reference_samples':s['sample_count'],'mesh_checks':mesh,'scripts_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in [root/'scripts'/name for name in ['export_observability_depth_ros.py','audit_global_observability_ros.py','verify_observability_mesh.py','summarize_global_observability.py']] }},indent=2))
print(root/'docs/GLOBAL_OBSERVABILITY_AUDIT_20261016.md')
