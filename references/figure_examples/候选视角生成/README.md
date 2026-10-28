# Fig.6 分方向参考图（重筛版）

当前保留 **13 篇论文、27 张不重复参考图**，按五条细分方向整理。旧版图集和未入选资料已移除。

[优先12项](Fig6_优先12项详解.pdf) · [完整图集](Fig6_完整参考图集_27项.pdf) · [分类速览](Fig6_分类速览.pdf) · [筛选图库](index.html) · [CSV](参考图索引.csv)

## 按 Fig.6 的视觉任务选择

|细分方向|对应面板|要画清什么|重点参考|
|---|---|---|---|
|语言与关系|a 的输入|table 是已知 anchor；under 是关系；backpack 是待验证目标|SOON Fig.1/2；ConceptGraphs Fig.1/2|
|观察区域|a 的输出|由 anchor 几何和关系构造区域，再放 region samples；目标位置不能画成已知|VLMaps Fig.1/2；VoxPoser Fig.2/3|
|候选位姿|b|距离×方位角采样、yaw 朝向区域、固定 pitch|FUEL Fig.4；Bubble Explorer Fig.4；MA-SCVP Fig.4|
|约束与可见性|b 侧视 inset、c|高度区间求交、FoV 包含、位置 clearance、样本 LOS 分开解释|ConstrainedSpaces Fig.4/5/16；SEE Fig.8；FALCON Fig.5|
|评分与压缩|d|每方位先按可见点数/代价选代表，再按预算保留角度分散的视角|SCONE Fig.2；MA-SCVP Fig.9；SEE Fig.10/11|

## 推荐的组合与限制

主图仍用同一张桌下场景贯穿 a–d；每组参考仅解决一个局部表达问题。不要把五组论文各自的完整系统框图拼成 Fig.6。
语义关系使用 SOON 的词语与实体同色标记；区域使用 VoxPoser 的透明三维体；采样使用 FUEL 的环向点；约束用侧视 FoV 与独立 pass/fail；筛选用可见样本矩阵和稀疏保留点。以上是针对本项目的绘图建议，不是这些论文共同实现的算法。
局部缺口：目前没有找到与本项目“固定 pitch 的高度交集 + 每方位按可见点数/终端代价排序 + 最小角距扩散”完全一致的单张论文图。高度交集和角度多样性应根据自己的公式原创绘制，不能把 set cover 说成等价算法。

## 裁图与来源

PNG/PDF 裁剪只保留图体及图内标签；长 caption 和正文留在原论文。每张图有原图号、PDF 页序、裁剪坐标、固定版本（可识别时）与源文件 SHA-256。原论文放在 papers/，分方向裁图放在 figures/。
REVERIE、SOON 使用 Matterport3D Simulator，不能因此标作 Habitat。VLMaps 有 Habitat 实验，但本包所选 Fig.1/2 是实景机器人示例。其余选图主要来自主动视觉、覆盖规划和操作领域；选择依据是局部画法与 Fig.6 的相关性。
所有图的版权归原作者/出版方。本包仅作研究与构图参考，未改动项目论文或 Fig.6。

## 删除与保留标准

删除纯网络架构、轨迹结果、失败恢复、普通导航拓扑更新，以及无法对应本图局部机制的价值地图。保留能够单独说明语义关系、空间区域、相机几何、可见性或覆盖互补性的图。ConceptGraphs 只作上游关系表示参考，不扩展 Fig.6 为建图方法图。

## 优先12项

- **02 SOON Fig. 1**：从大区域逐步缩小到目标区域（对应面板 a）。
- **05 SOON Fig. 2**：指令拆成物体、属性、关系、目标区域（对应面板 a）。
- **06 VLMaps Fig. 1**：空间指令定位：物体、距离与 between（对应面板 a）。
- **08 VoxPoser Fig. 2**：语言条件的三维 affordance 与 constraint maps（对应面板 a）。
- **10 FUEL Fig. 4**：目标区域中心周围的距离/方位角采样（对应面板 b）。
- **13 BubbleExplorer Fig. 4**：球面上的均匀候选视点（对应面板 b）。
- **16 ConstrainedSpaces Fig. 4**：相机 frustum/FOV 几何空间（对应面板 b/c）。
- **19 SEE-Journal Fig. 8**：沿 sight line 检测遮挡（对应面板 b/c）。
- **21 FALCON Fig. 5**：FoV raycast 与 qualified/disqualified viewpoint（对应面板 b/c）。
- **23 ConstrainedSpaces Fig. 16**：由目标样本、遮挡射线构造不可见空间（对应面板 b/c）。
- **24 SCONE Fig. 2**：proxy points 与 visibility gain（对应面板 d）。
- **25 MA-SCVP Fig. 9**：不同 view 覆盖不同 surface sets（对应面板 d）。

## 语言任务、anchor 与空间关系

|编号|论文/图号|页|优先级|借鉴点|
|---|---|---:|---:|---|
|01|[REVERIE Fig. 1](figures/grounding/01_REVERIE_Fig1.png) · [原论文](https://openaccess.thecvf.com/content_CVPR_2020/papers/Qi_REVERIE_Remote_Embodied_Visual_Referring_Expression_in_Real_Indoor_Environments_CVPR_2020_paper.pdf#page=1)|1|1|语言指令、导航视点与目标对象|
|02|[SOON Fig. 1](figures/grounding/02_SOON_Fig1.png) · [原论文](https://openaccess.thecvf.com/content/CVPR2021/papers/Zhu_SOON_Scenario_Oriented_Object_Navigation_With_Graph-Based_Exploration_CVPR_2021_paper.pdf#page=2)|2|1|从大区域逐步缩小到目标区域|
|03|[ConceptGraphs Fig. 1](figures/grounding/03_ConceptGraphs_Fig1.png) · [原论文](https://arxiv.org/pdf/2309.16650v1#page=1)|1|1|物体实例、语言标签和空间关系图|
|04|[ConceptGraphs Fig. 2](figures/grounding/04_ConceptGraphs_Fig2.png) · [原论文](https://arxiv.org/pdf/2309.16650v1#page=3)|3|1|RGB-D 多视图 → 3D objects → relations|
|05|[SOON Fig. 2](figures/grounding/05_SOON_Fig2.png) · [原论文](https://openaccess.thecvf.com/content/CVPR2021/papers/Zhu_SOON_Scenario_Oriented_Object_Navigation_With_Graph-Based_Exploration_CVPR_2021_paper.pdf#page=4)|4|1|指令拆成物体、属性、关系、目标区域|

## Observation region 与目标样本

|编号|论文/图号|页|优先级|借鉴点|
|---|---|---:|---:|---|
|06|[VLMaps Fig. 1](figures/region/06_VLMaps_Fig1.png) · [原论文](https://arxiv.org/pdf/2210.05714v4#page=1)|1|1|空间指令定位：物体、距离与 between|
|07|[VLMaps Fig. 2](figures/region/07_VLMaps_Fig2.png) · [原论文](https://arxiv.org/pdf/2210.05714v4#page=2)|2|1|near、定量位移与 between 的并列示例|
|08|[VoxPoser Fig. 2](figures/region/08_VoxPoser_Fig2.png) · [原论文](https://proceedings.mlr.press/v229/huang23b/huang23b.pdf#page=4)|4|1|语言条件的三维 affordance 与 constraint maps|
|09|[VoxPoser Fig. 3](figures/region/09_VoxPoser_Fig3.png) · [原论文](https://proceedings.mlr.press/v229/huang23b/huang23b.pdf#page=5)|5|2|三维价值区域与真实观测的对应|

## 相机位姿参数化与候选采样

|编号|论文/图号|页|优先级|借鉴点|
|---|---|---:|---:|---|
|10|[FUEL Fig. 4](figures/sampling/10_FUEL_Fig4.png) · [原论文](https://arxiv.org/pdf/2010.11561v2#page=3)|3|1|目标区域中心周围的距离/方位角采样|
|11|[SEE-2018 Fig. 4](figures/sampling/11_SEE-2018_Fig4.png) · [原论文](https://arxiv.org/pdf/1802.08617v1#page=4)|4|1|表面法向、观察距离和视锥|
|12|[SEE-Journal Fig. 5](figures/sampling/12_SEE-Journal_Fig5.png) · [原论文](https://arxiv.org/pdf/2207.13684v4#page=9)|9|1|frontier point + normal 的 view proposal|
|13|[BubbleExplorer Fig. 4](figures/sampling/13_BubbleExplorer_Fig4.png) · [原论文](https://arxiv.org/pdf/2304.00852v1#page=3)|3|1|球面上的均匀候选视点|
|14|[BubbleExplorer Fig. 5](figures/sampling/14_BubbleExplorer_Fig5.png) · [原论文](https://arxiv.org/pdf/2304.00852v1#page=4)|4|1|从球心候选到生成 view，再删除覆盖区域|
|15|[MA-SCVP Fig. 4](figures/sampling/15_MA-SCVP_Fig4.png) · [原论文](https://arxiv.org/pdf/2304.00910v4#page=6)|6|1|有限候选 view space 与 view pose 自由度|

## FoV、碰撞、遮挡与可见性判定

|编号|论文/图号|页|优先级|借鉴点|
|---|---|---:|---:|---|
|16|[ConstrainedSpaces Fig. 4](figures/feasibility/16_ConstrainedSpaces_Fig4.png) · [原论文](https://arxiv.org/pdf/2306.06969v1#page=7)|7|1|相机 frustum/FOV 几何空间|
|17|[ConstrainedSpaces Fig. 5](figures/feasibility/17_ConstrainedSpaces_Fig5.png) · [原论文](https://arxiv.org/pdf/2306.06969v1#page=10)|10|1|多个 viewpoint constraint 的交集|
|18|[ConstrainedSpaces Fig. 17](figures/feasibility/18_ConstrainedSpaces_Fig17.png) · [原论文](https://arxiv.org/pdf/2306.06969v1#page=23)|23|1|occlusion C-space 与 occlusion-free space|
|19|[SEE-Journal Fig. 8](figures/feasibility/19_SEE-Journal_Fig8.png) · [原论文](https://arxiv.org/pdf/2207.13684v4#page=10)|10|1|沿 sight line 检测遮挡|
|20|[SEE-Journal Fig. 9](figures/feasibility/20_SEE-Journal_Fig9.png) · [原论文](https://arxiv.org/pdf/2207.13684v4#page=11)|11|1|球面方向中寻找无遮挡视线|
|21|[FALCON Fig. 5](figures/feasibility/21_FALCON_Fig5.png) · [原论文](https://arxiv.org/pdf/2407.00577v2#page=8)|8|1|FoV raycast 与 qualified/disqualified viewpoint|
|22|[ConstrainedSpaces Fig. 22](figures/feasibility/22_ConstrainedSpaces_Fig22.png) · [原论文](https://arxiv.org/pdf/2306.06969v1#page=27)|27|2|多个 feature constrained spaces 求交|
|23|[ConstrainedSpaces Fig. 16](figures/feasibility/23_ConstrainedSpaces_Fig16.png) · [原论文](https://arxiv.org/pdf/2306.06969v1#page=23)|23|1|由目标样本、遮挡射线构造不可见空间|

## 评分、互补性与候选压缩

|编号|论文/图号|页|优先级|借鉴点|
|---|---|---:|---:|---|
|24|[SCONE Fig. 2](figures/selection/24_SCONE_Fig2.png) · [原论文](https://arxiv.org/pdf/2208.10449v2#page=3)|3|1|proxy points 与 visibility gain|
|25|[MA-SCVP Fig. 9](figures/selection/25_MA-SCVP_Fig9.png) · [原论文](https://arxiv.org/pdf/2304.00910v4#page=9)|9|1|不同 view 覆盖不同 surface sets|
|26|[SEE-Journal Fig. 10](figures/selection/26_SEE-Journal_Fig10.png) · [原论文](https://arxiv.org/pdf/2207.13684v4#page=12)|12|1|frontier-view visibility graph|
|27|[SEE-Journal Fig. 11](figures/selection/27_SEE-Journal_Fig11.png) · [原论文](https://arxiv.org/pdf/2207.13684v4#page=13)|13|1|next-best-view 的可见性/距离选择|

## 每项图注

### 01 · REVERIE Fig. 1 — 语言指令、导航视点与目标对象

[PNG](figures/grounding/01_REVERIE_Fig1.png) · [裁剪 PDF](figures/grounding/01_REVERIE_Fig1.pdf) · [来源 PDF 第 1 页](https://openaccess.thecvf.com/content_CVPR_2020/papers/Qi_REVERIE_Remote_Embodied_Visual_Referring_Expression_in_Real_Indoor_Environments_CVPR_2020_paper.pdf#page=1)

借鉴 instruction → remote object → viewpoint 的叙事关系。

**区别：** REVERIE 使用离散全景 waypoint，不包含你的 3D pose feasibility。

### 02 · SOON Fig. 1 — 从大区域逐步缩小到目标区域

[PNG](figures/grounding/02_SOON_Fig1.png) · [裁剪 PDF](figures/grounding/02_SOON_Fig1.pdf) · [来源 PDF 第 2 页](https://openaccess.thecvf.com/content/CVPR2021/papers/Zhu_SOON_Scenario_Oriented_Object_Navigation_With_Graph-Based_Exploration_CVPR_2021_paper.pdf#page=2)

直接参考你的 coarse-to-fine anchor/region 叙事。

**区别：** SOON 的搜索过程是导航策略，不是相机候选生成。

### 03 · ConceptGraphs Fig. 1 — 物体实例、语言标签和空间关系图

[PNG](figures/grounding/03_ConceptGraphs_Fig1.png) · [裁剪 PDF](figures/grounding/03_ConceptGraphs_Fig1.pdf) · [来源 PDF 第 1 页](https://arxiv.org/pdf/2309.16650v1#page=1)

适合画 mapped anchor、target 和 relation edge。

**区别：** 图中包含完整建图系统，Fig.6 只需借用对象关系层。

### 04 · ConceptGraphs Fig. 2 — RGB-D 多视图 → 3D objects → relations

[PNG](figures/grounding/04_ConceptGraphs_Fig2.png) · [裁剪 PDF](figures/grounding/04_ConceptGraphs_Fig2.pdf) · [来源 PDF 第 3 页](https://arxiv.org/pdf/2309.16650v1#page=3)

只借鉴右侧的物体实例与关系边，帮助读者看懂 anchor 来自已有地图。

**区别：** 上游建图流程不属于 Fig.6，不能整套照搬。

### 05 · SOON Fig. 2 — 指令拆成物体、属性、关系、目标区域

[PNG](figures/grounding/05_SOON_Fig2.png) · [裁剪 PDF](figures/grounding/05_SOON_Fig2.pdf) · [来源 PDF 第 4 页](https://openaccess.thecvf.com/content/CVPR2021/papers/Zhu_SOON_Scenario_Oriented_Object_Navigation_With_Graph-Based_Exploration_CVPR_2021_paper.pdf#page=4)

借鉴同色短语与场景实体的对应；只突出 anchor / relation / target。

**区别：** 原图是数据标注步骤，不是你的任务解析算法。

### 06 · VLMaps Fig. 1 — 空间指令定位：物体、距离与 between

[PNG](figures/region/06_VLMaps_Fig1.png) · [裁剪 PDF](figures/region/06_VLMaps_Fig1.pdf) · [来源 PDF 第 1 页](https://arxiv.org/pdf/2210.05714v4#page=1)

借鉴用同一场景表示语言中的物体参照与相对位置。

**区别：** 定位的是导航目标点；你的输出是可观察区域及样本。

### 07 · VLMaps Fig. 2 — near、定量位移与 between 的并列示例

[PNG](figures/region/07_VLMaps_Fig2.png) · [裁剪 PDF](figures/region/07_VLMaps_Fig2.pdf) · [来源 PDF 第 2 页](https://arxiv.org/pdf/2210.05714v4#page=2)

借鉴按空间关系展示不同目标位置，替换成桌下、周围、表面等区域。

**区别：** 原图有机器人路线；Fig.6(a) 保留区域标记即可。

### 08 · VoxPoser Fig. 2 — 语言条件的三维 affordance 与 constraint maps

[PNG](figures/region/08_VoxPoser_Fig2.png) · [裁剪 PDF](figures/region/08_VoxPoser_Fig2.pdf) · [来源 PDF 第 4 页](https://proceedings.mlr.press/v229/huang23b/huang23b.pdf#page=4)

借鉴先分别可视化目标区域和禁止区域，再组合成几何表示。

**区别：** 操作用 value map 不等于你的离散 region samples，也不是视角生成。

### 09 · VoxPoser Fig. 3 — 三维价值区域与真实观测的对应

[PNG](figures/region/09_VoxPoser_Fig3.png) · [裁剪 PDF](figures/region/09_VoxPoser_Fig3.pdf) · [来源 PDF 第 5 页](https://proceedings.mlr.press/v229/huang23b/huang23b.pdf#page=5)

借鉴半透明三维区域叠加物体的画法，使区域不是孤立的抽象框。

**区别：** 仅作为区域画法；不要搬入机械臂轨迹或假定目标已知。

### 10 · FUEL Fig. 4 — 目标区域中心周围的距离/方位角采样

[PNG](figures/sampling/10_FUEL_Fig4.png) · [裁剪 PDF](figures/sampling/10_FUEL_Fig4.pdf) · [来源 PDF 第 3 页](https://arxiv.org/pdf/2010.11561v2#page=3)

直接借鉴 Fig.6(b) 的柱坐标候选布局。

**区别：** FUEL 没有语言目标或竖直 FoV 高度区间。

### 11 · SEE-2018 Fig. 4 — 表面法向、观察距离和视锥

[PNG](figures/sampling/11_SEE-2018_Fig4.png) · [裁剪 PDF](figures/sampling/11_SEE-2018_Fig4.pdf) · [来源 PDF 第 4 页](https://arxiv.org/pdf/1802.08617v1#page=4)

适合画 region sample 到相机的朝向关系。

**区别：** 只沿法向生成，不能覆盖你的多 azimuth/multi-height。

### 12 · SEE-Journal Fig. 5 — frontier point + normal 的 view proposal

[PNG](figures/sampling/12_SEE-Journal_Fig5.png) · [裁剪 PDF](figures/sampling/12_SEE-Journal_Fig5.pdf) · [来源 PDF 第 9 页](https://arxiv.org/pdf/2207.13684v4#page=9)

适合做候选相机小图标和视锥的简洁样式参考。

**区别：** 没有候选集预算和角度多样性。

### 13 · BubbleExplorer Fig. 4 — 球面上的均匀候选视点

[PNG](figures/sampling/13_BubbleExplorer_Fig4.png) · [裁剪 PDF](figures/sampling/13_BubbleExplorer_Fig4.pdf) · [来源 PDF 第 3 页](https://arxiv.org/pdf/2304.00852v1#page=3)

可借鉴三维候选点云和目标区域的空间关系。

**区别：** 球面采样比你的固定 pitch 更自由。

### 14 · BubbleExplorer Fig. 5 — 从球心候选到生成 view，再删除覆盖区域

[PNG](figures/sampling/14_BubbleExplorer_Fig5.png) · [裁剪 PDF](figures/sampling/14_BubbleExplorer_Fig5.pdf) · [来源 PDF 第 4 页](https://arxiv.org/pdf/2304.00852v1#page=4)

适合画候选生成和重叠删除的顺序。

**区别：** 覆盖的是 frontier，不是 target samples。

### 15 · MA-SCVP Fig. 4 — 有限候选 view space 与 view pose 自由度

[PNG](figures/sampling/15_MA-SCVP_Fig4.png) · [裁剪 PDF](figures/sampling/15_MA-SCVP_Fig4.pdf) · [来源 PDF 第 6 页](https://arxiv.org/pdf/2304.00910v4#page=6)

适合表示候选池大小和相机位姿参数。

**区别：** 5-DOF object capture 与你的飞行高度约束不同。

### 16 · ConstrainedSpaces Fig. 4 — 相机 frustum/FOV 几何空间

[PNG](figures/feasibility/16_ConstrainedSpaces_Fig4.png) · [裁剪 PDF](figures/feasibility/16_ConstrainedSpaces_Fig4.pdf) · [来源 PDF 第 7 页](https://arxiv.org/pdf/2306.06969v1#page=7)

直接参考 horizontal/vertical FoV 和姿态约束。

**区别：** 需把 feature 改成你的 region samples。

### 17 · ConstrainedSpaces Fig. 5 — 多个 viewpoint constraint 的交集

[PNG](figures/feasibility/17_ConstrainedSpaces_Fig5.png) · [裁剪 PDF](figures/feasibility/17_ConstrainedSpaces_Fig5.pdf) · [来源 PDF 第 10 页](https://arxiv.org/pdf/2306.06969v1#page=10)

适合表达 FoV、距离、姿态和空间可行性的逐层求交。

**区别：** C-space 是连续空间，需要改成离散候选过滤。

### 18 · ConstrainedSpaces Fig. 17 — occlusion C-space 与 occlusion-free space

[PNG](figures/feasibility/18_ConstrainedSpaces_Fig17.png) · [裁剪 PDF](figures/feasibility/18_ConstrainedSpaces_Fig17.pdf) · [来源 PDF 第 23 页](https://arxiv.org/pdf/2306.06969v1#page=23)

直接参考通过/拒绝视角和障碍物造成的不可见区域。

**区别：** 原文是连续 manifold，不是 Habitat raycast。

### 19 · SEE-Journal Fig. 8 — 沿 sight line 检测遮挡

[PNG](figures/feasibility/19_SEE-Journal_Fig8.png) · [裁剪 PDF](figures/feasibility/19_SEE-Journal_Fig8.pdf) · [来源 PDF 第 10 页](https://arxiv.org/pdf/2207.13684v4#page=10)

适合画目标样本到 camera 的 LOS 检查。

**区别：** 使用点云邻域，不是你的 analytic/voxel geometry。

### 20 · SEE-Journal Fig. 9 — 球面方向中寻找无遮挡视线

[PNG](figures/feasibility/20_SEE-Journal_Fig9.png) · [裁剪 PDF](figures/feasibility/20_SEE-Journal_Fig9.pdf) · [来源 PDF 第 11 页](https://arxiv.org/pdf/2207.13684v4#page=11)

适合表达被障碍方向排除后保留可见朝向。

**区别：** 二维几何示意，需要转成你的 3D 相机姿态。

### 21 · FALCON Fig. 5 — FoV raycast 与 qualified/disqualified viewpoint

[PNG](figures/feasibility/21_FALCON_Fig5.png) · [裁剪 PDF](figures/feasibility/21_FALCON_Fig5.pdf) · [来源 PDF 第 8 页](https://arxiv.org/pdf/2407.00577v2#page=8)

适合做 Fig.6(c) 的 pass/fail 对照。

**区别：** 判据是未知体素数量，不是你的 Q_i。

### 22 · ConstrainedSpaces Fig. 22 — 多个 feature constrained spaces 求交

[PNG](figures/feasibility/22_ConstrainedSpaces_Fig22.png) · [裁剪 PDF](figures/feasibility/22_ConstrainedSpaces_Fig22.pdf) · [来源 PDF 第 27 页](https://arxiv.org/pdf/2306.06969v1#page=27)

借鉴两个目标特征的可观测空间求交，表达同一相机必须同时满足多个样本约束。

**区别：** 这是同一视角满足多个特征，不能解释成多个视角互补覆盖。

### 23 · ConstrainedSpaces Fig. 16 — 由目标样本、遮挡射线构造不可见空间

[PNG](figures/feasibility/23_ConstrainedSpaces_Fig16.png) · [裁剪 PDF](figures/feasibility/23_ConstrainedSpaces_Fig16.pdf) · [来源 PDF 第 23 页](https://arxiv.org/pdf/2306.06969v1#page=23)

借鉴 target samples → ray intersections → occluded region 的因果过程。

**区别：** 原算法构造连续遮挡空间；你的方法逐个候选执行 LOS 检查。

### 24 · SCONE Fig. 2 — proxy points 与 visibility gain

[PNG](figures/selection/24_SCONE_Fig2.png) · [裁剪 PDF](figures/selection/24_SCONE_Fig2.pdf) · [来源 PDF 第 3 页](https://arxiv.org/pdf/2208.10449v2#page=3)

对应 visible sample count / coverage gain。

**区别：** SCONE 用学习模块预测 visibility。

### 25 · MA-SCVP Fig. 9 — 不同 view 覆盖不同 surface sets

[PNG](figures/selection/25_MA-SCVP_Fig9.png) · [裁剪 PDF](figures/selection/25_MA-SCVP_Fig9.pdf) · [来源 PDF 第 9 页](https://arxiv.org/pdf/2304.00910v4#page=9)

是 Fig.6(d) 互补视角和 set-cover 的直接参考。

**区别：** 不包含你的 terminal cost。

### 26 · SEE-Journal Fig. 10 — frontier-view visibility graph

[PNG](figures/selection/26_SEE-Journal_Fig10.png) · [裁剪 PDF](figures/selection/26_SEE-Journal_Fig10.pdf) · [来源 PDF 第 12 页](https://arxiv.org/pdf/2207.13684v4#page=12)

可借鉴视角之间按可见性连接，再选择代表 view。

**区别：** 你的候选规模更小，还要考虑 azimuth diversity。

### 27 · SEE-Journal Fig. 11 — next-best-view 的可见性/距离选择

[PNG](figures/selection/27_SEE-Journal_Fig11.png) · [裁剪 PDF](figures/selection/27_SEE-Journal_Fig11.pdf) · [来源 PDF 第 13 页](https://arxiv.org/pdf/2207.13684v4#page=13)

可参考 visible gain 与 travel cost 同时存在的图示。

**区别：** SEE 使用 outdegree，不是你的完整 cost。
