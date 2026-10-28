# Joint task–viewpoint planning and MINCO

论文插图：184 × 108 mm，英文标注，原生矢量 PDF / SVG，500 dpi PNG。单张主地图表现规划与观测更新，右侧完整放大图解释 MINCO。采用用户指定的浅灰背景、灰色墙体、青蓝轨迹与橙色激活提示。A 图下方单独放置时间条和图例；B 图在同一个放大场景中叠加灰色示意引导线与青蓝色真实 MINCO 轨迹，整块 B 统一对应 A 中的虚线框。

- `joint_task_viewpoint_planning_en.pdf`：论文用主文件。
- `joint_task_viewpoint_planning_en.svg`：可编辑矢量图。
- `joint_task_viewpoint_planning_en_preview.png`：快速预览。
- `rviz/rviz_preview.png`：相同数据的实际 RViz 空间预览。
- `example_inputs.json`：抽象房间、任务锚点和候选视角。
- `initial_minco.txt` / `updated_minco.txt`：真实 MINCO 多项式系数。
- `mechanism_evidence.json` / `minco_provenance.json`：规划结果、复核信息和生成配置。


## 当前配色

| 元素 | 色值 |
| --- | --- |
| 画布背景 | `#FAFBFC` |
| 墙体 | `#B1BBC4` |
| MINCO 轨迹、常规选中视角 | `#287A91` |
| 新激活任务 D | `#EA844A` |
| 已检查 A、已到达视点 | `#8B91A0` |
| B 图的 A* 示意引导线 | `#8B91A0` |

参考用户提供的平面场景图，房间采用很浅的冷暖底色和低对比地面网格，墙体在原有范围内增加明暗面。桌子和柜体使用浅木色，沙发采用灰蓝色；家具细节均位于原有碰撞范围内。网格仅帮助辨认空间，不表示实际体素分辨率；墙面明暗是视觉表达，不改变墙体或门洞几何。主语义配色保持不变。

A、B 外框采用细黑色圆角线，标题直接排在背景上，分图字母使用黑色粗体。放大框和引线仍使用粗黑虚线，引线端点随外框圆角位置对齐。

视点改为线框视锥符号，顶点仍位于真实候选坐标，朝向仍来自实际 yaw；浅灰细线表示备选，较粗彩色线表示选中，灰色线框配黑色顶点表示已到达。四边形像平面用于示意相机方向，不表示实际投影尺寸。任务锚点使用带字母的空心圆，颜色区分常规、已检查和新激活状态。A、B 使用相同的墙体、家具和视点绘制规则，轨迹不添加背景色描边。示例输入、规划记录和 MINCO 系数均未改变；RViz 保留原空间预览。

## 图意

**a · Joint task–viewpoint planning.** 在同一张地图上表达执行首个访问、获得观测、更新剩余计划。房间由墙体和门洞明确分隔，每间房只有一个字母任务锚点。锚点标记任务对应的检查位置，家具提供空间参照，不表示已确认目标存在；周围多个小视锥是该锚点的候选相机位姿。浅灰细线框为备选视角，较粗彩色线框为当前计划选中的观察位姿；灰色线框配黑色顶点保留已到达的 A 观察位。轨迹经过门洞、终止于选中视点，不飞到家具内的任务锚点。青蓝色实线均为保存的真实 MINCO 曲线，Start 位置以简洁的无人机俯视图标表示。

Start 位于 Room A 的右上侧。首段绕过家具到达选中的观察视点，随后朝门口继续前往其他房间，避免起点位于走廊中段时产生的入房后原路折返。任务顺序、选中视点和 MINCO 均按这个起点重新计算。

初始计划为 A → C → B，D 尚未激活。**实际只执行 Start → A**，不会先执行完初始计划。A 的检查结束后仍未找到目标，终结观测 `A: not found` 激活备用任务 D；贴近 A 观察位的标注给出这个因果关系。A 锚点变灰，图例 `Checked` 只表示已检查，不表示找到了目标；D 标为 `Newly active`。从实际 A 位姿重新规划，剩余访问顺序变为 D → C → B。若在 A 找到了目标，这个备用分支不会激活，D 会被跳过。渲染器直接从实际状态事件读取图中文字，并同时复核 found / not_found 两个分支。

本图对应显式的“此处没有就去指定备用地点”分支，例如 `config/stage2_tasks/00337_household_multifloor.json` 中 `not_found` 才激活备用电视搜索任务。A 的局部重试与观测验证过程在此抽象掉，不表示任一单帧未检出就立即转去 D；也不把显式备用分支与自主语义恢复混为一谈。状态机内部的 `completed` 表示该检查已结束，是否找到由独立的 `outcome` 表示。

主图只以浅灰虚线保留被替换的 A → C 旧路线，新的 A → D → C 以青蓝实线突出。C → B 的访问连接没有改变，因此只画一次，使用更新后计划的 MINCO 多项式；并不声称两次优化的连续曲线逐点一致。旧路线也直接取自初始 MINCO，未为避免重叠而平移或手工修改。底部时间条明确分开 `Executed: Start → A` 与 `Updated plan: D → C → B`，后者尚未执行。图中不再重复绘制第二张房间地图。

A 图的图例位于其自身边框内、时间条下方：第一行说明候选视点、选中视点、已到达视点及已检查锚点，第二行用 `Previous plan`（灰色虚线）与 `Updated plan`（青蓝实线）对应表示更新前后的计划。已执行的 Start → A 仍由时间条和 `Executed` 标注说明；MINCO 的技术名称保留在 B 图的轨迹生成说明中。`Reached` 图例的灰色空心视锥和黑色顶点与主图一致。`Checked` 表示检查已经结束，不等于目标已找到。B 图不再容纳 A 图的状态或路径图例。

**b · Trajectory generation.** 用 Start → A 的同一局部场景说明后端处理：灰色折线是前端 A* 引导路径的简化示意，青蓝色实线是实际 MINCO 平滑轨迹，两者叠加在同一坐标系内，共用起点与选中视点。A 图用粗黑虚线矩形圈出对应区域，其右上、右下角分别以黑色虚线引向整个 B 外框的左上、左下角。B 包含一个放大地图、专属路径图例和优化说明，不再拆成上下两幅地图；完整外框明确 B 是首段轨迹的局部解释。放大框直接使用 B 图的地图坐标范围。起点用小圆点表示，无人机图标仅保留在 a 图。图中未把 A* 折线、B-spline 或人工贝塞尔曲线当作 MINCO。

B 下方的两行标注 `MINCO optimization / within safe corridors` 表示在安全走廊约束下进行 MINCO 优化。这里的安全走廊是沿引导路径计算出的连续可飞行空间，不是场景中连接房间的建筑走廊。代码 `ros1_full_smooth_mission.cpp::buildSuperCorridorPhase` 先调用 CIRI 构建走廊，再将走廊传入 `super_optimizer_->optimize`。本图展示这两步的处理关系，未绘制实际 CIRI 多面体边界。

灰色折线的图例明确标为 `A* guide (schematic)`。绘图函数 `schematic_guide` 对原始引导路径做折线简化，保留起终点和绕障碍的主要转折，省略栅格阶梯与末端栅格中心接到准确视点的小回钩；每一段简化线都通过本例地图的碰撞检查。这个操作只用于论文示意，不是规划算法新增的处理步骤，也不是现有 MINCO 系数的生成输入。原始引导点、示例输入、规划结果和 MINCO 系数均保持原样；A、B 图的青蓝轨迹都直接使用保存的 MINCO 多项式采样。

## 与代码的对应和边界

- 核心联合优化：`PRE_MAP_VLN/stage2/joint_planner.py::plan_joint_mission`，对子集、末任务、末候选视点进行 DP；代价由 A* 路长、估计运动时间、视点终端代价构成。图不显示示例参数或数值。
- 房间是本图的空间组织方式，不意味着代码把一个房间永远限制为一个任务。**本例严格采用用户要求的每房间一个锚点**，同一任务的所有候选共享同一个 `location_hypothesis_id`，不会把不同候选误画成不同任务或不同物体。
- 输入为示意性的、已纳入规划的候选集合。这里展示核心 DP 对多个候选的联合选择，调用方式对应离线多目标入口 `real_fly/stage2_offline/scripts/plan_real_multitarget_smoke.py`。它不是 Habitat 默认 `rolling_representative` 首轮“每个位置只纳入一个代表”的逐配置复现；不把未纳入 DP 的候选宣称为参与联合优化。
- 任务更新：`stage2/mission_executor.py::MissionState.finish_task`；滚动执行首个访问的逻辑参考 `scripts/run_stage2_habitat.py`。初始和更新后的例子都实际调用核心 DP，并通过独立穷举核对最优代价。
- MINCO：实际运行项目已有 `full_smooth_mission` 离线预览程序，链路为 A* → XY clearance refinement → CIRI safe corridor → constrained MINCO。保存的系数由 `real_fly/stage2_runtime/scripts/saved_minco_artifact.py::read_trajectory` 解析。主图和 RViz 使用同一组多项式采样。
- **DP 采用前端路径代价，MINCO 是后端轨迹生成；没有声称 DP 对最终 MINCO 的时间或动力学代价做全局联合优化。** 末端停留观测允许速度降为零，因此图中曲线在观察位姿处相接，不要求多次访问在全局平面投影中互不重叠。
- 本图将核心联合规划、条件更新和项目已有 MINCO 后端组合成机制示例。冻结后回放的真机任务包本身不会自动变成在线重规划系统。
- 轨迹通过 MINCO 生成器的连续安全与导数连续性检查，并复核采样点、相邻采样段均通过本例体素地图的有效性检测。MINCO 允许很小的高度调整；图为俯视投影，未声称严格定高。
- 抽象场景用于机制解释，不作为实际 Habitat 或真机实验结果，不用于报告性能指标。

## Suggested caption

**Joint task–viewpoint planning with observation-driven updates and MINCO trajectory generation.** (a) Each room contains a task anchor with multiple candidate observation poses. The initial plan visits A, C and B, with D inactive. Only Start → A is executed. A terminal not-found outcome at A activates the explicit fallback task D, and the remaining visits are replanned as D → C → B from the reached A pose. Local verification and retries are abstracted. The replaced A → C trajectory is shown as a grey dashed line; teal solid curves show the executed prefix and updated remaining plan. The retained C → B connection is drawn once using the updated trajectory. Faint and bold wireframe frusta indicate candidate and selected poses; the grey wireframe with a dark vertex marks the reached pose, and the grey anchor denotes a checked location rather than a successfully found target. The bottom strip separates executed motion from future visits, followed by the panel's legend. The objective combines path length, estimated motion time and viewpoint cost. The dashed rectangle connects to the complete enlarged panel (b), where the Start → A detail illustrates MINCO optimization within safe corridors. The grey A* guide and teal MINCO trajectory are overlaid in the same scene with common endpoints. The guide is simplified for visual clarity while preserving the obstacle detour; the smooth curve is the saved MINCO trajectory generated from the original guide. Both plans and MINCO trajectories are computed using the project implementations; the scene is a mechanism illustration.

## 重绘与 RViz

重绘原生论文图（不需要 ROS / Docker）：

```bash
/home/uav/miniconda3/envs/env_isaaclab/bin/python 画图/联合任务视角规划/render_figure.py
```

启动 RViz 并回放保存的 MINCO 采样（需要本机图形会话及已有容器镜像）：

```bash
bash 画图/联合任务视角规划/rviz/run_preview.sh
```

RViz 保留初始 A → C → B 计划的几何与轨迹回放，用于检查空间关系；它不播放主图中的观测事件和重规划切换，也不是飞行动力学仿真。它发布独立的可视化 Marker 话题，没有飞控指令。当前使用软件 OpenGL 渲染，支持旋转视角；关闭窗口后退出。论文排版使用矢量 PDF，RViz 图适合检查空间关系或做补充材料。

RViz 的墙体采用截短显示，以便看到房间内部；规划与碰撞复核仍使用完整高度的体素墙体。截图由 `capture.cpp` 直接导出 OGRE 渲染缓冲区，避免桌面通知或其他窗口进入素材。

`minco_provenance.json` 留有实际离线生成配置、生成脚本和引导任务数据。系数文件绑定了输入的 SHA-256；修改场景或候选时必须重新生成 MINCO，渲染器不会静默套用旧轨迹。将示例分支纠正为 `not_found` 后，重新调用状态机和规划器确认两次规划的活跃集合、位姿、候选、完整路径及代价与原轨迹输入一致，因此保留真实 MINCO 系数；原始生成输入哈希、当前兼容输入哈希和这次等价复核记录均保存在 provenance 中。
