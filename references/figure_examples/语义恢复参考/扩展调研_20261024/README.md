# Fig.8 参考图扩展调研（2026-10-24）

已下载并查看 **27 篇论文**，选出 **57 个参考单元**。一个参考单元可能是整图、明确标出的子图，或相邻两幅图；不将其混称为独立论文数量。
这是针对你当前 Fig.8 的定向图像调研，不是 VLN 全领域的穷尽综述。既有 6 图参考包保留，本包复用其中有用来源并显著扩充。

**先打开：** [优先 12 项总览](overviews/00_优先看这12项.png) · [优先 12 项详解](Fig8_优先12项详解.pdf) · [分类速览 PDF](Fig8_分类速览.pdf) · [完整带说明图集](Fig8_完整参考图集_57项.pdf) · [可筛选图库](index.html) · [CSV 索引](参考图索引.csv)

每个条目有：实际原图裁剪、论文固定版本、图号、PDF 页码、任务类别、Habitat 核实状态、可以借鉴的设计、与你机制的区别。`figures/` 同时提供 PNG 和保留原图矢量内容的 PDF 裁剪；`papers/` 是原论文。

## 与你的 Fig.8 对齐

依据 `PRE_MAP_VLN/论文/TASE_英文初稿/root.tex` 的 Fig.8 图注及执行章节，你需要讲清：未检出后允许的局部重试 → 局部候选耗尽/达到限制 → 同一目标换区域恢复 → 执行中成功验证 → 执行后将确认的新实例插入场景图。重复覆盖不能重复产生负证据；没有找到目标不能否定作为锚点的其他物体。

我最推荐 **BSC-Nav Fig.3b / SG-Nav Fig.6 的因果叙事 + BeliefMapNav Fig.12 的空间对照 + DovSG Fig.3 的局部更新**。前三篇有 Habitat 实验；DovSG 的所选图来自真实场景，仅补充地图更新画法。这是绘图取舍判断，不表示它们实现了你的方法。

## 优先选择

|编号|论文与图号|PDF 页|最适合借鉴|
|---|---|---:|---|
|24|[BSC-Nav Fig. 3b](figures/24_BSC-Nav_Fig3b.png)|5|候选 A 被否定后继续到候选 B|
|27|[SG-Nav Fig. 6](figures/27_SG-Nav_Fig6.png)|9|靠近再观察，否定误检，再找到目标|
|39|[BeliefMapNav Fig. 12](figures/39_BeliefMapNav_Fig12.png)|20|固定三维视角对照信念更新|
|54|[DovSG Fig. 3](figures/54_DovSG_Fig3.png)|5|新观测使地图局部发生变化|
|01|[ETPNav Fig. 5](figures/01_ETPNav_Fig5.png)|12|全局规划如何纠正走错的路线|
|04|[CA-Nav Fig. 7](figures/04_CA-Nav_Fig7.png)|11|指令、观测和价值地图按时间对齐|
|18|[RegretfulAgent Fig. 1](figures/18_RegretfulAgent_Fig1.png)|1|以一张主地图组织回退前后的观测|
|40|[UniGoal Fig. 4](figures/40_UniGoal_Fig4.png)|7|探索、候选定位、验证三个阶段|
|30|[VLFM Fig. 1](figures/30_VLFM_Fig1.png)|1|真实场景、轨迹和语义信息叠加|
|33|[BeliefMapNav Fig. 1](figures/33_BeliefMapNav_Fig1.png)|2|一张主地图串起六个观察时刻|
|42|[CARe Fig. 1–2](figures/42_CARe_Fig1-2.png)|2|初始地图出错后为何需要重规划|
|46|[HOV-SG Fig. 8](figures/46_HOV-SG_Fig8.png)|9|多房间、多楼层中的语义目标路线|

## 四条调研线与全部原图

### A · 执行、局部重试与跨区域恢复

|编号|论文 / 图号|PDF 页|优先级|适合参考的内容|
|---|---|---:|---:|---|
|01|[ETPNav Fig. 5](figures/01_ETPNav_Fig5.png) · [原文](https://arxiv.org/pdf/2304.03047v3#page=12)|12|2|全局规划如何纠正走错的路线|
|02|[ETPNav Fig. 6](figures/02_ETPNav_Fig6.png) · [原文](https://arxiv.org/pdf/2304.03047v3#page=12)|12|2|被障碍卡住后的试探与恢复|
|04|[CA-Nav Fig. 7](figures/04_CA-Nav_Fig7.png) · [原文](https://arxiv.org/pdf/2412.10137v4#page=11)|11|1|指令、观测和价值地图按时间对齐|
|07|[MapNav Fig. 7](figures/07_MapNav_Fig7.png) · [原文](https://arxiv.org/pdf/2502.13451v5#page=14)|14|2|RGB 与带文字标注的地图同步演进|
|10|[InstructNav Fig. 2](figures/10_InstructNav_Fig2.png) · [原文](https://arxiv.org/pdf/2406.04882v1#page=4)|4|3|用观测反馈更新当前导航步骤|
|12|[Open-Nav Fig. 2](figures/12_Open-Nav_Fig2.png) · [原文](https://arxiv.org/pdf/2409.18794v2#page=4)|4|3|候选方向与现场判断一一对应|
|13|[VLN-CE Fig. 5](figures/13_VLN-CE_Fig5.png) · [原文](https://arxiv.org/pdf/2004.02857v2#page=14)|14|2|短观测序列加小型轨迹地图|
|16|[DUET Fig. 5](figures/16_DUET_Fig5.png) · [原文](https://arxiv.org/pdf/2202.11742v1#page=8)|8|2|全局动作与局部绕路的对照|
|18|[RegretfulAgent Fig. 1](figures/18_RegretfulAgent_Fig1.png) · [原文](https://arxiv.org/pdf/1903.01602v1#page=1)|1|1|以一张主地图组织回退前后的观测|
|19|[RegretfulAgent Fig. 5](figures/19_RegretfulAgent_Fig5.png) · [原文](https://arxiv.org/pdf/1903.01602v1#page=9)|9|2|用连续第一视角明确展示纠错|
|20|[FAST Fig. 1](figures/20_FAST_Fig1.png) · [原文](https://arxiv.org/pdf/1903.02547v2#page=1)|1|2|同一俯视地图中的回退路径|
|21|[FAST Fig. 2](figures/21_FAST_Fig2.png) · [原文](https://arxiv.org/pdf/1903.02547v2#page=2)|2|3|抽象展示可以回到哪些候选分支|
|22|[Meta-Explore Fig. 2](figures/22_Meta-Explore_Fig2.png) · [原文](https://arxiv.org/pdf/2303.04077v1#page=2)|2|2|恢复不局限于返回原节点|
|23|[Meta-Explore Fig. 13](figures/23_Meta-Explore_Fig13.png) · [原文](https://arxiv.org/pdf/2303.04077v1#page=19)|19|2|卡住—回退—恢复探索的时间链|
|24|[BSC-Nav Fig. 3b](figures/24_BSC-Nav_Fig3b.png) · [原文](https://www.nature.com/articles/s41467-026-74358-5.pdf#page=5)|5|1|候选 A 被否定后继续到候选 B|
|25|[BSC-Nav Fig. 4c](figures/25_BSC-Nav_Fig4c.png) · [原文](https://www.nature.com/articles/s41467-026-74358-5.pdf#page=6)|6|2|长指令分段与地图关键点对应|
|27|[SG-Nav Fig. 6](figures/27_SG-Nav_Fig6.png) · [原文](https://arxiv.org/pdf/2410.08189v1#page=9)|9|1|靠近再观察，否定误检，再找到目标|
|40|[UniGoal Fig. 4](figures/40_UniGoal_Fig4.png) · [原文](https://arxiv.org/pdf/2503.10630v3#page=7)|7|1|探索、候选定位、验证三个阶段|
|57|[InnerMonologue Fig. 1](figures/57_InnerMonologue_Fig1.png) · [原文](https://proceedings.mlr.press/v205/huang23c/huang23c.pdf#page=2)|2|3|动作旁边立即呈现成功或失败反馈|

### B · 覆盖、负证据与搜索信念

|编号|论文 / 图号|PDF 页|优先级|适合参考的内容|
|---|---|---:|---:|---|
|06|[CA-Nav Fig. 4](figures/06_CA-Nav_Fig4.png) · [原文](https://arxiv.org/pdf/2412.10137v4#page=5)|5|2|相机观测投到地图并更新|
|11|[InstructNav Fig. 3](figures/11_InstructNav_Fig3.png) · [原文](https://arxiv.org/pdf/2406.04882v1#page=5)|5|2|多来源空间信息合成决策地图|
|28|[SG-Nav Fig. 9](figures/28_SG-Nav_Fig9.png) · [原文](https://arxiv.org/pdf/2410.08189v1#page=16)|16|2|给候选前沿着色并解释为何换方向|
|31|[VLFM Fig. 3](figures/31_VLFM_Fig3.png) · [原文](https://arxiv.org/pdf/2312.03275v1#page=3)|3|2|重叠视野如何表达|
|32|[VLFM Fig. 4](figures/32_VLFM_Fig4.png) · [原文](https://arxiv.org/pdf/2312.03275v1#page=4)|4|2|两次观测怎样改变空间地图|
|34|[BeliefMapNav Fig. 4](figures/34_BeliefMapNav_Fig4.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=9)|9|2|先验、可见性、后验并排对照|
|35|[BeliefMapNav Fig. 5](figures/35_BeliefMapNav_Fig5.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=16)|16|2|哪些图像区域能提供可靠负证据|
|36|[BeliefMapNav Fig. 9](figures/36_BeliefMapNav_Fig9.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=19)|19|2|观测序列与后验信念变化同时展示|
|37|[BeliefMapNav Fig. 10](figures/37_BeliefMapNav_Fig10.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=19)|19|3|第二个搜索场景的时序布局|
|38|[BeliefMapNav Fig. 11](figures/38_BeliefMapNav_Fig11.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=20)|20|3|目标较远时的搜索序列|
|39|[BeliefMapNav Fig. 12](figures/39_BeliefMapNav_Fig12.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=20)|20|1|固定三维视角对照信念更新|
|42|[CARe Fig. 1–2](figures/42_CARe_Fig1-2.png) · [原文](https://arxiv.org/pdf/2409.04837v2#page=2)|2|1|初始地图出错后为何需要重规划|
|43|[L3MVN Fig. 5](figures/43_L3MVN_Fig5.png) · [原文](https://arxiv.org/pdf/2304.05501v2#page=5)|5|2|Habitat 中搜索目标的紧凑横条|
|44|[ESC Fig. 5](figures/44_ESC_Fig5.png) · [原文](https://arxiv.org/pdf/2301.13166v3#page=14)|14|2|为什么离开当前房间继续找|
|45|[OpenFMNav Fig. 2](figures/45_OpenFMNav_Fig2.png) · [原文](https://arxiv.org/pdf/2402.10670v2#page=3)|3|3|指令、观察与地图的闭环关系|
|50|[SEEK Fig. 3](figures/50_SEEK_Fig3.png) · [原文](https://arxiv.org/pdf/2405.09822v2#page=7)|7|2|房间信念与跨区域搜索策略|
|51|[SEEK Fig. 9](figures/51_SEEK_Fig9.png) · [原文](https://arxiv.org/pdf/2405.09822v2#page=14)|14|2|连续未找到后房间概率如何变化|

### C · 主场景、路径与观测的空间对应

|编号|论文 / 图号|PDF 页|优先级|适合参考的内容|
|---|---|---:|---:|---|
|05|[CA-Nav Fig. 2](figures/05_CA-Nav_Fig2.png) · [原文](https://arxiv.org/pdf/2412.10137v4#page=2)|2|2|真实俯视场景上的任务状态与语义层|
|09|[MapNav Fig. 1](figures/09_MapNav_Fig1.png) · [原文](https://arxiv.org/pdf/2502.13451v5#page=1)|1|2|观察—地图—物体标签的空间对应|
|14|[BEVBert Fig. 5](figures/14_BEVBert_Fig5.png) · [原文](https://arxiv.org/pdf/2212.04385v2#page=7)|7|2|同一场景内对比路线与终点|
|26|[BSC-Nav Fig. 2](figures/26_BSC-Nav_Fig2.png) · [原文](https://www.nature.com/articles/s41467-026-74358-5.pdf#page=4)|4|2|真实三维环境里的多个目标候选|
|30|[VLFM Fig. 1](figures/30_VLFM_Fig1.png) · [原文](https://arxiv.org/pdf/2312.03275v1#page=1)|1|1|真实场景、轨迹和语义信息叠加|
|33|[BeliefMapNav Fig. 1](figures/33_BeliefMapNav_Fig1.png) · [原文](https://arxiv.org/pdf/2506.06487v1#page=2)|2|1|一张主地图串起六个观察时刻|
|46|[HOV-SG Fig. 8](figures/46_HOV-SG_Fig8.png) · [原文](https://arxiv.org/pdf/2403.17846v2#page=9)|9|1|多房间、多楼层中的语义目标路线|
|48|[VLMaps Fig. 2](figures/48_VLMaps_Fig2.png) · [原文](https://arxiv.org/pdf/2210.05714v4#page=2)|2|2|自然语言空间关系在场景中直接落地|

### D · 多视角融合与场景图更新

|编号|论文 / 图号|PDF 页|优先级|适合参考的内容|
|---|---|---:|---:|---|
|03|[ETPNav Fig. 2](figures/03_ETPNav_Fig2.png) · [原文](https://arxiv.org/pdf/2304.03047v3#page=4)|4|3|观测如何扩展拓扑图|
|08|[MapNav Fig. 3](figures/08_MapNav_Fig3.png) · [原文](https://arxiv.org/pdf/2502.13451v5#page=4)|4|2|RGB-D 到对象标签地图|
|15|[BEVBert Fig. 3](figures/15_BEVBert_Fig3.png) · [原文](https://arxiv.org/pdf/2212.04385v2#page=5)|5|3|图更新前后只突出变化部分|
|17|[DUET Fig. 3](figures/17_DUET_Fig3.png) · [原文](https://arxiv.org/pdf/2202.11742v1#page=3)|3|3|新观测触发节点插入|
|29|[SG-Nav Fig. 3](figures/29_SG-Nav_Fig3.png) · [原文](https://arxiv.org/pdf/2410.08189v1#page=4)|4|2|新节点与旧图的关系怎样画|
|41|[UniGoal Fig. 3](figures/41_UniGoal_Fig3.png) · [原文](https://arxiv.org/pdf/2503.10630v3#page=5)|5|2|锚点关系和图纠正的局部示意|
|47|[HOV-SG Fig. 7](figures/47_HOV-SG_Fig7.png) · [原文](https://arxiv.org/pdf/2403.17846v2#page=8)|8|2|房间层、楼层、物体层叠在真实地图上|
|49|[VLMaps Fig. 3](figures/49_VLMaps_Fig3.png) · [原文](https://arxiv.org/pdf/2210.05714v4#page=3)|3|3|观测语义如何融合到三维地图|
|52|[ConceptGraphs Fig. 2](figures/52_ConceptGraphs_Fig2.png) · [原文](https://arxiv.org/pdf/2309.16650v1#page=3)|3|2|多视角观测融合为三维实例与场景图|
|53|[ConceptGraphs Fig. 3](figures/53_ConceptGraphs_Fig3.png) · [原文](https://arxiv.org/pdf/2309.16650v1#page=6)|6|2|检索候选后再用视觉验证|
|54|[DovSG Fig. 3](figures/54_DovSG_Fig3.png) · [原文](https://arxiv.org/pdf/2410.11989v6#page=5)|5|1|新观测使地图局部发生变化|
|55|[DovSG Fig. 2](figures/55_DovSG_Fig2.png) · [原文](https://arxiv.org/pdf/2410.11989v6#page=4)|4|3|对象地图与图节点的双层表达|
|56|[SayPlan Fig. 2](figures/56_SayPlan_Fig2.png) · [原文](https://proceedings.mlr.press/v229/rana23a/rana23a.pdf#page=4)|4|3|空间场景与层次节点共同展示|

优先级 1：先看；2：针对局部表达补充；3：备选或构图较复杂。排序依据是与你 Fig.8 的适配程度，不是论文质量。

## 仿真器与任务边界

“用了 HM3D / MP3D 数据”不自动等于“用了 Habitat 仿真器”。同一论文也可能同时包含 Habitat、其他仿真器和真实机器人图。本表在无法核实处保留不确定性。

|论文|任务|Habitat 状态|核实依据|
|---|---|---|---|
|[ETPNav](https://arxiv.org/pdf/2304.03047v3)|VLN-CE|是|[PDF p4、p7 明确使用 Habitat；R2R-CE / RxR-CE。](https://arxiv.org/pdf/2304.03047v3)|
|[CA-Nav](https://arxiv.org/pdf/2412.10137v4)|VLN-CE|是|[PDF p6 明确使用 Habitat；本包为 arXiv v4，图号不同于早期 ICLR 投稿版。](https://arxiv.org/pdf/2412.10137v4)|
|[MapNav](https://arxiv.org/pdf/2502.13451v5)|VLN-CE|是|[正文 p4 任务定义；官方代码安装 Habitat-Sim / Habitat-Lab。本包为 v5。](https://github.com/linglingxiansen/MapNav)|
|[InstructNav](https://arxiv.org/pdf/2406.04882v1)|VLN-CE / ObjectNav / DDN|是（部分任务）|[PDF p6 实验设置：ObjectNav 和 R2R-CE 使用 Habitat；DDN 另有设置。](https://arxiv.org/pdf/2406.04882v1)|
|[Open-Nav](https://arxiv.org/pdf/2409.18794v2)|VLN-CE|是|[PDF p5 Simulated Environments 明确 R2R-CE / Habitat。](https://arxiv.org/pdf/2409.18794v2)|
|[VLN-CE](https://arxiv.org/pdf/2004.02857v2)|VLN-CE 基准|是|[PDF p5 Continuous Matterport3D Environments in Habitat。](https://arxiv.org/pdf/2004.02857v2)|
|[BEVBert](https://arxiv.org/pdf/2212.04385v2)|离散 VLN；另有 CE 实现|混合|[本文所选图为离散 R2R / 拓扑图；官方代码另提供 Habitat 的 R2R-CE 设置，不能把所选图直接称为 Habitat 实验图。](https://github.com/MarSaKi/VLN-BEVBert)|
|[DUET](https://arxiv.org/pdf/2202.11742v1)|离散 VLN|所选实验非 Habitat|[论文为 R2R / REVERIE / SOON 的离散导航图设定；不因使用 MP3D 数据就标为 Habitat。](https://arxiv.org/pdf/2202.11742v1)|
|[RegretfulAgent](https://arxiv.org/pdf/1903.01602v1)|离散 VLN|所选实验非 Habitat|[R2R 离散导航图上的回退；图中的进度值不是区域目标存在信念。](https://arxiv.org/pdf/1903.01602v1)|
|[FAST](https://arxiv.org/pdf/1903.02547v2)|离散 VLN|所选实验非 Habitat|[R2R 离散导航图搜索；仅作为回退表达的经典参考。](https://arxiv.org/pdf/1903.02547v2)|
|[Meta-Explore](https://arxiv.org/pdf/2303.04077v1)|VLN；扩展 ImageNav|混合|[PDF p17 明确扩展 ImageNav 用 Habitat/Gibson；Fig.13 属于此扩展，Fig.2 是 VLN 概念图。](https://arxiv.org/pdf/2303.04077v1)|
|[BSC-Nav](https://www.nature.com/articles/s41467-026-74358-5.pdf)|ObjectNav / 实例导航 / VLN-CE|是|[正式版 PDF p13 明确 Habitat 3.0；Fig.3b 是实例导航，Fig.4c 是指令导航。](https://www.nature.com/articles/s41467-026-74358-5)|
|[SG-Nav](https://arxiv.org/pdf/2410.08189v1)|ObjectNav|是（另含 RoboTHOR）|[论文实验含 MP3D / HM3D / RoboTHOR；官方代码明确 Habitat-Sim 0.2.4。](https://github.com/bagh2178/SG-Nav)|
|[VLFM](https://arxiv.org/pdf/2312.03275v1)|ObjectNav|是|[PDF p5 Datasets 明确 Habitat，使用 Gibson / HM3D / MP3D。](https://arxiv.org/pdf/2312.03275v1)|
|[BeliefMapNav](https://arxiv.org/pdf/2506.06487v1)|ObjectNav|是|[论文 p7 给出 HM3D / MP3D / HSSD；官方实现明确 Habitat-Sim / Habitat-Lab 0.2.5。](https://github.com/ZiboKNOW/BeliefMapNav)|
|[UniGoal](https://arxiv.org/pdf/2503.10630v3)|ObjectNav / ImageNav / TextNav|是（另含 RoboTHOR）|[PDF p6 Implementation Details 明确 Habitat；统一目标导航不等于路线指令跟随。](https://arxiv.org/pdf/2503.10630v3)|
|[CARe](https://arxiv.org/pdf/2409.04837v2)|预建地图上的 ObjectNav|是（VLMaps 分支）|[PDF p5、p7：VLMaps 分支用 HabitatSim；OpenMask3D 分支主要评价检索。](https://arxiv.org/pdf/2409.04837v2)|
|[L3MVN](https://arxiv.org/pdf/2304.05501v2)|ObjectNav|是|[PDF p5 Experiment Details 和 Fig.5 图注明确 Habitat。](https://arxiv.org/pdf/2304.05501v2)|
|[ESC](https://arxiv.org/pdf/2301.13166v3)|ObjectNav|未单独核实|[原文明确 MP3D / HM3D / RoboTHOR；本轮未找到足够明确的仿真器直接证据，不计入已确认 Habitat 数量。](https://arxiv.org/pdf/2301.13166v3)|
|[OpenFMNav](https://arxiv.org/pdf/2402.10670v2)|开放词汇 ObjectNav|是|[PDF p6 HM3D ObjectNav；官方代码明确 Habitat-Sim / Habitat-Lab challenge-2022。](https://github.com/yxKryptonite/OpenFMNav)|
|[HOV-SG](https://arxiv.org/pdf/2403.17846v2)|语言目标导航 / 分层建图|是（另含真实机器人）|[PDF p10 明确 Habitat 导航实验；Fig.7–8 使用 HM3DSem，不能只靠数据集名推断仿真器。](https://arxiv.org/pdf/2403.17846v2)|
|[VLMaps](https://arxiv.org/pdf/2210.05714v4)|语言空间目标导航|是（部分实验）|[PDF p4：空间/多目标导航用 Habitat；多机体实验用 AI2-THOR。本包没有把后者标为 Habitat。](https://arxiv.org/pdf/2210.05714v4)|
|[SEEK](https://arxiv.org/pdf/2405.09822v2)|ObjectNav / 巡检|混合|[PDF p8：Habitat、Gazebo 和真机三类实验；所选 Fig.9 是 Gazebo 场景，非 Habitat。](https://arxiv.org/pdf/2405.09822v2)|
|[ConceptGraphs](https://arxiv.org/pdf/2309.16650v1)|开放词汇建图 / 机器人规划|所选图非 Habitat 实验|[所选 Fig.2 是建图流程，Fig.3 是真实 Jackal 机器人；不将其算作 VLN-CE。](https://arxiv.org/pdf/2309.16650v1)|
|[DovSG](https://arxiv.org/pdf/2410.11989v6)|动态建图 / 移动操作|所选图为真实场景|[Fig.3 是人工改变真实场景后的地图适应；属于地图更新的邻近方向参考。](https://arxiv.org/pdf/2410.11989v6)|
|[SayPlan](https://proceedings.mlr.press/v229/rana23a/rana23a.pdf)|场景图任务规划|所选图非 Habitat 实验|[Fig.2 是场景图层次结构；仅借鉴空间与图节点的对应关系。](https://proceedings.mlr.press/v229/rana23a/rana23a.pdf)|
|[InnerMonologue](https://proceedings.mlr.press/v205/huang23c/huang23c.pdf)|语言机器人任务规划|所选图非 Habitat 实验|[Fig.1 包含真实机器人及桌面任务；作为反馈组织方式的补充。](https://proceedings.mlr.press/v205/huang23c/huang23c.pdf)|

## 按 Fig.8 的四个部分选择

- **局部重试：** SG-Nav 27、ETPNav 02、Regretful Agent 19。至少保留两次不同观测与触发原因；原图中的误检、避障、路线错误需转换成你的真实情形。
- **新增覆盖与负证据：** VLFM 31 看重叠视野，BeliefMapNav 39 看空间信念对照，SEEK 50–51 看房间搜索。用固定视角标“已覆盖 / 本次新增”，不能只换热图颜色。
- **跨区域恢复：** BSC-Nav 24、ETPNav 01、Regretful Agent 18、HOV-SG 46。主图应让读者看见起始区域、替代区域和连接路径。
- **验证与地图更新：** BSC-Nav 24、UniGoal 40、ConceptGraphs 52–53、DovSG 54。成功验证和新实例插图分开标时点；Before / After 旧节点保持原位置，新增节点和边再高亮。

## 建议保留的三种构图方案

1. **主地图居中，关键观测环绕。** 参考 BSC-Nav 24、Regretful Agent 18、BeliefMapNav 33。最适合强调“同一目标换到哪里继续找”，右下角留小型地图更新对照。
2. **三列时间序列，每列地图与 RGB 对齐。** 参考 CA-Nav 04、ETPNav 01、UniGoal 40。依次呈现局部重试、跨区域搜索、验证成功；更新前后图放在末列下方。
3. **一张大空间主图 + 两个机制局部放大。** 参考 VLFM 30、HOV-SG 46、BeliefMapNav 39、DovSG 54。左侧放覆盖去重，右侧放实例更新；更适合你希望 Fig.8 有明确空间结构的情况。

不建议把完整系统框图、长段 LLM 对话、多个互不对应的三维小场景组合成主图。它们很难同时说明局部失败、跨区域路线和新实例的变化。

## 版本、来源与复核

- 每张裁剪都来自本地保存的原论文，未做生成式补图或改动原图数据。子图与相邻两图均有标记。
- `sources.json` 记录 URL、页码、裁剪坐标、版本及 SHA-256。页码一律为 PDF 文件从 1 开始的页序，不是期刊印刷页码。
- 本包下载到 CA-Nav v4、MapNav v5、ETPNav v3；BeliefMapNav / SG-Nav 保留此前参考包的 v1，因此不能拿其他版本图号直接对应。
- BSC-Nav 使用 Nature Communications 2026 正式版；Fig.3b 与 Fig.4c 是不同任务的子图。
- AECNav（arXiv:2608.10817）检索到但 PDF 本轮下载超时，未查看实际配图，因此不计入 27 篇和参考图数量。
- 图像版权归原作者 / 出版方。本包用于来源明确的研究与构图参考；本项目论文与已有 Fig.8 文件没有改动。

复现：在本目录运行 `python build_pack.py`。下载器为 `download_papers.py`；它只读取明确列出的公开论文地址。

## 每项的具体借鉴与限制

### 01 · ETPNav Fig. 5 — 全局规划如何纠正走错的路线

[原图 PNG](figures/01_ETPNav_Fig5.png) · [裁剪 PDF](figures/01_ETPNav_Fig5.pdf) · [论文第 12 页](https://arxiv.org/pdf/2304.03047v3#page=12)

同一地图坐标系中逐步显示路线，下方对应 RGB 观测；适合解释跨区域重规划。

**区别：** 对比局部与全局规划，不是目标未检出导致的区域信念更新。

### 02 · ETPNav Fig. 6 — 被障碍卡住后的试探与恢复

[原图 PNG](figures/02_ETPNav_Fig6.png) · [裁剪 PDF](figures/02_ETPNav_Fig6.pdf) · [论文第 12 页](https://arxiv.org/pdf/2304.03047v3#page=12)

成对时间序列展示有无 Tryout 的行为差别，恢复动作能直接看见。

**区别：** 这是障碍规避；不要将其直接解释为语义目标验证失败。

### 03 · ETPNav Fig. 2 — 观测如何扩展拓扑图

[原图 PNG](figures/03_ETPNav_Fig2.png) · [裁剪 PDF](figures/03_ETPNav_Fig2.pdf) · [论文第 4 页](https://arxiv.org/pdf/2304.03047v3#page=4)

新增、已访问、候选节点采用稳定图例，适合借鉴新增节点的高亮。

**区别：** 更新的是导航路点图，不是物体实例场景图。

### 04 · CA-Nav Fig. 7 — 指令、观测和价值地图按时间对齐

[原图 PNG](figures/04_CA-Nav_Fig7.png) · [裁剪 PDF](figures/04_CA-Nav_Fig7.pdf) · [论文第 11 页](https://arxiv.org/pdf/2412.10137v4#page=11)

观察图与地图上下对齐；少量关键帧就能交代“看到什么、随后往哪里走”。

**区别：** 原图含方法对比且信息很多；Fig.8 可保留单条过程的三个关键时刻。

### 05 · CA-Nav Fig. 2 — 真实俯视场景上的任务状态与语义层

[原图 PNG](figures/05_CA-Nav_Fig2.png) · [裁剪 PDF](figures/05_CA-Nav_Fig2.pdf) · [论文第 2 页](https://arxiv.org/pdf/2412.10137v4#page=2)

房间底图、目标星号、路点和价值层共用空间参照，适合主图布局。

**区别：** 子指令推进不是同一目标的失败恢复；任务状态含义需改为本项目定义。

### 06 · CA-Nav Fig. 4 — 相机观测投到地图并更新

[原图 PNG](figures/06_CA-Nav_Fig4.png) · [裁剪 PDF](figures/06_CA-Nav_Fig4.pdf) · [论文第 5 页](https://arxiv.org/pdf/2412.10137v4#page=5)

视锥、当前观测和更新前后地图形成直接因果链。

**区别：** 价值融合不是你的新增覆盖去重规则，不能直接借用其公式。

### 07 · MapNav Fig. 7 — RGB 与带文字标注的地图同步演进

[原图 PNG](figures/07_MapNav_Fig7.png) · [裁剪 PDF](figures/07_MapNav_Fig7.pdf) · [论文第 14 页](https://arxiv.org/pdf/2502.13451v5#page=14)

两行对应同一步的观测和地图，关键物体标签有助于阅读。

**区别：** 原图比较有无 ASM；Fig.8 应换成单次恢复过程。

### 08 · MapNav Fig. 3 — RGB-D 到对象标签地图

[原图 PNG](figures/08_MapNav_Fig3.png) · [裁剪 PDF](figures/08_MapNav_Fig3.pdf) · [论文第 4 页](https://arxiv.org/pdf/2502.13451v5#page=4)

图像、点云、投影平面与文本标签逐层对应，适合说明新观测如何落图。

**区别：** 标注语义地图不等于新实例的确认和插入。

### 09 · MapNav Fig. 1 — 观察—地图—物体标签的空间对应

[原图 PNG](figures/09_MapNav_Fig1.png) · [裁剪 PDF](figures/09_MapNav_Fig1.pdf) · [论文第 1 页](https://arxiv.org/pdf/2502.13451v5#page=1)

同一位置在 RGB、俯视地图和 ASM 中保持对应，可减少纯文字解释。

**区别：** 本包使用 v5；小尺寸下标签过多，需要只保留相关对象。

### 10 · InstructNav Fig. 2 — 用观测反馈更新当前导航步骤

[原图 PNG](figures/10_InstructNav_Fig2.png) · [裁剪 PDF](figures/10_InstructNav_Fig2.pdf) · [论文第 4 页](https://arxiv.org/pdf/2406.04882v1#page=4)

任务输入、当前动作及下一轮更新有清晰连接，可参考反馈箭头。

**区别：** 这是通用导航步骤规划；整体文字偏多，不宜作为 Fig.8 主构图。

### 11 · InstructNav Fig. 3 — 多来源空间信息合成决策地图

[原图 PNG](figures/11_InstructNav_Fig3.png) · [裁剪 PDF](figures/11_InstructNav_Fig3.pdf) · [论文第 5 页](https://arxiv.org/pdf/2406.04882v1#page=5)

多个地图共享坐标和视觉语法，适合解释不同证据如何共同影响搜索。

**区别：** 原方法合成多种价值图，不是你的覆盖与区域信念机制。

### 12 · Open-Nav Fig. 2 — 候选方向与现场判断一一对应

[原图 PNG](figures/12_Open-Nav_Fig2.png) · [裁剪 PDF](figures/12_Open-Nav_Fig2.pdf) · [论文第 4 页](https://arxiv.org/pdf/2409.18794v2#page=4)

观测旁边摆候选方向，再连接决策，适合局部重试的候选说明。

**区别：** 长段推理文字占空间；建议只取候选与图像的对应方式。

### 13 · VLN-CE Fig. 5 — 短观测序列加小型轨迹地图

[原图 PNG](figures/13_VLN-CE_Fig5.png) · [裁剪 PDF](figures/13_VLN-CE_Fig5.pdf) · [论文第 14 页](https://arxiv.org/pdf/2004.02857v2#page=14)

在少量帧中保留连续环境中的移动过程，适合紧凑实验示例。

**区别：** 它展示指令执行，不单独解释语义恢复和地图更新。

### 14 · BEVBert Fig. 5 — 同一场景内对比路线与终点

[原图 PNG](figures/14_BEVBert_Fig5.png) · [裁剪 PDF](figures/14_BEVBert_Fig5.pdf) · [论文第 7 页](https://arxiv.org/pdf/2212.04385v2#page=7)

地图上用统一的起点、目标和错误终点符号，空间关系容易比较。

**区别：** 所选图属于离散 R2R；官方 CE 实现存在不代表这张图就是 Habitat 结果。

### 15 · BEVBert Fig. 3 — 图更新前后只突出变化部分

[原图 PNG](figures/15_BEVBert_Fig3.png) · [裁剪 PDF](figures/15_BEVBert_Fig3.pdf) · [论文第 5 页](https://arxiv.org/pdf/2212.04385v2#page=5)

旧节点保持样式，只标明新增节点及访问状态。

**区别：** 拓扑路点更新，不是物体确认；只参考视觉编码。

### 16 · DUET Fig. 5 — 全局动作与局部绕路的对照

[原图 PNG](figures/16_DUET_Fig5.png) · [裁剪 PDF](figures/16_DUET_Fig5.pdf) · [论文第 8 页](https://arxiv.org/pdf/2202.11742v1#page=8)

在同一底图上标出不同路径，虚线表达全局连接，适合解释恢复跨度。

**区别：** 离散 VLN 的全局图动作，不代表可直接飞行的连续轨迹。

### 17 · DUET Fig. 3 — 新观测触发节点插入

[原图 PNG](figures/17_DUET_Fig3.png) · [裁剪 PDF](figures/17_DUET_Fig3.pdf) · [论文第 3 页](https://arxiv.org/pdf/2202.11742v1#page=3)

三个连续小图固定旧节点布局，读者能立即找到新增部分。

**区别：** 导航图的节点含义不同，移植时必须换成实例节点与确认条件。

### 18 · RegretfulAgent Fig. 1 — 以一张主地图组织回退前后的观测

[原图 PNG](figures/18_RegretfulAgent_Fig1.png) · [裁剪 PDF](figures/18_RegretfulAgent_Fig1.pdf) · [论文第 1 页](https://arxiv.org/pdf/1903.01602v1#page=1)

中心地图负责空间主线，两侧观测展示关键转折，是很好的整体构图模板。

**区别：** 非 Habitat 的离散 VLN；进度估计不等于你的目标存在信念。

### 19 · RegretfulAgent Fig. 5 — 用连续第一视角明确展示纠错

[原图 PNG](figures/19_RegretfulAgent_Fig5.png) · [裁剪 PDF](figures/19_RegretfulAgent_Fig5.pdf) · [论文第 9 页](https://arxiv.org/pdf/1903.01602v1#page=9)

方向箭头和回退标记标在实际图像上，便于表达局部换视角。

**区别：** 原图有三个独立案例，不应把它们串成同一次任务；建议只借一种标记方式。

### 20 · FAST Fig. 1 — 同一俯视地图中的回退路径

[原图 PNG](figures/20_FAST_Fig1.png) · [裁剪 PDF](figures/20_FAST_Fig1.pdf) · [论文第 1 页](https://arxiv.org/pdf/1903.02547v2#page=1)

对称对比两条路径、统一起终点标记，适合看路线叙事是否清楚。

**区别：** 离散 R2R 搜索；原图不提供区域负证据更新。

### 21 · FAST Fig. 2 — 抽象展示可以回到哪些候选分支

[原图 PNG](figures/21_FAST_Fig2.png) · [裁剪 PDF](figures/21_FAST_Fig2.pdf) · [论文第 2 页](https://arxiv.org/pdf/1903.02547v2#page=2)

用很少的节点和浅色搜索区域解释局部与全局搜索差别。

**区别：** 是抽象搜索树，若单独使用仍缺少实际观测证据。

### 22 · Meta-Explore Fig. 2 — 恢复不局限于返回原节点

[原图 PNG](figures/22_Meta-Explore_Fig2.png) · [裁剪 PDF](figures/22_Meta-Explore_Fig2.pdf) · [论文第 2 页](https://arxiv.org/pdf/2303.04077v1#page=2)

地图中同时保留已访问和未访问候选，可以借鉴换区域后的候选表达。

**区别：** 这是 VLN 的局部目标搜索概念，不是单次未检出即换区域。

### 23 · Meta-Explore Fig. 13 — 卡住—回退—恢复探索的时间链

[原图 PNG](figures/23_Meta-Explore_Fig13.png) · [裁剪 PDF](figures/23_Meta-Explore_Fig13.pdf) · [论文第 19 页](https://arxiv.org/pdf/2303.04077v1#page=19)

上排轨迹、下排观测、底部模式条，适合直接展示恢复发生的时间。

**区别：** 这张图是 Habitat 的 ImageNav 扩展，不是语言指令实验；不能混称为 VLN-CE 结果。

### 24 · BSC-Nav Fig. 3b — 候选 A 被否定后继续到候选 B

[原图 PNG](figures/24_BSC-Nav_Fig3b.png) · [裁剪 PDF](figures/24_BSC-Nav_Fig3b.pdf) · [论文第 5 页](https://www.nature.com/articles/s41467-026-74358-5.pdf#page=5)

主地图、局部地图、RGB 和验证结果相互对应，最适合借鉴恢复过程的整体叙事。

**区别：** Fig.3b 为整图 b 子图；原任务为实例匹配，否定错误候选不等于你的区域覆盖负证据。

### 25 · BSC-Nav Fig. 4c — 长指令分段与地图关键点对应

[原图 PNG](figures/25_BSC-Nav_Fig4c.png) · [裁剪 PDF](figures/25_BSC-Nav_Fig4c.pdf) · [论文第 6 页](https://www.nature.com/articles/s41467-026-74358-5.pdf#page=6)

俯视主场景与子目标关键帧并置，适合参考空间主图的布局。

**区别：** Fig.4c 是完整 c 子图；原图是子任务推进，不要用来替代同一任务的恢复。

### 26 · BSC-Nav Fig. 2 — 真实三维环境里的多个目标候选

[原图 PNG](figures/26_BSC-Nav_Fig2.png) · [裁剪 PDF](figures/26_BSC-Nav_Fig2.pdf) · [论文第 4 页](https://www.nature.com/articles/s41467-026-74358-5.pdf#page=4)

场景中的候选位置连接到图像局部，读者可以比较候选而不丢失位置。

**区别：** 整体黑色点云背景较重；可借对应关系，不必照搬底色。

### 27 · SG-Nav Fig. 6 — 靠近再观察，否定误检，再找到目标

[原图 PNG](figures/27_SG-Nav_Fig6.png) · [裁剪 PDF](figures/27_SG-Nav_Fig6.pdf) · [论文第 9 页](https://arxiv.org/pdf/2410.08189v1#page=9)

上方案例呈现搜索，下方案例呈现误检纠正；推荐重点看下半部观测链。

**区别：** 本包保留完整 Fig.6；它的再感知处理误检，你的 Fig.8 还需表达未检出和覆盖去重。

### 28 · SG-Nav Fig. 9 — 给候选前沿着色并解释为何换方向

[原图 PNG](figures/28_SG-Nav_Fig9.png) · [裁剪 PDF](figures/28_SG-Nav_Fig9.pdf) · [论文第 16 页](https://arxiv.org/pdf/2410.08189v1#page=16)

候选分数附着在地图位置，决策理由紧挨对应地图。

**区别：** 前沿概率不是区域存在信念；理由文字应压缩为触发条件。

### 29 · SG-Nav Fig. 3 — 新节点与旧图的关系怎样画

[原图 PNG](figures/29_SG-Nav_Fig3.png) · [裁剪 PDF](figures/29_SG-Nav_Fig3.pdf) · [论文第 4 页](https://arxiv.org/pdf/2410.08189v1#page=4)

新节点单独着色，再展示连接和裁剪，适合新增物体节点的局部设计。

**区别：** 原文是图边生成与裁剪，不对应你执行后才插入实例的时序。

### 30 · VLFM Fig. 1 — 真实场景、轨迹和语义信息叠加

[原图 PNG](figures/30_VLFM_Fig1.png) · [裁剪 PDF](figures/30_VLFM_Fig1.pdf) · [论文第 1 页](https://arxiv.org/pdf/2312.03275v1#page=1)

一张场景图同时承载搜索路径和语义层，整体空间关系最直观。

**区别：** 只是视觉主场景参考，原图不解释负证据去重。

### 31 · VLFM Fig. 3 — 重叠视野如何表达

[原图 PNG](figures/31_VLFM_Fig3.png) · [裁剪 PDF](figures/31_VLFM_Fig3.pdf) · [论文第 3 页](https://arxiv.org/pdf/2312.03275v1#page=3)

两个相机视野保留清晰重叠区，适合设计已看区域与新增覆盖的区别。

**区别：** 原文为视角置信度加权融合；你的重复覆盖不重复计入负证据，规则不同。

### 32 · VLFM Fig. 4 — 两次观测怎样改变空间地图

[原图 PNG](figures/32_VLFM_Fig4.png) · [裁剪 PDF](figures/32_VLFM_Fig4.pdf) · [论文第 4 页](https://arxiv.org/pdf/2312.03275v1#page=4)

相同布局的两行展示 RGB、深度、视野和地图更新，因果关系清晰。

**区别：** 语义价值投影和你的区域置信度公式不同。

### 33 · BeliefMapNav Fig. 1 — 一张主地图串起六个观察时刻

[原图 PNG](figures/33_BeliefMapNav_Fig1.png) · [裁剪 PDF](figures/33_BeliefMapNav_Fig1.pdf) · [论文第 2 页](https://arxiv.org/pdf/2506.06487v1#page=2)

主图给出空间上下文，关键观测环绕布置，适合 Fig.8 整体视觉结构。

**区别：** 六个时刻对于单幅小图偏多，建议只保留未检出、换区域和成功三个时刻。

### 34 · BeliefMapNav Fig. 4 — 先验、可见性、后验并排对照

[原图 PNG](figures/34_BeliefMapNav_Fig4.png) · [裁剪 PDF](figures/34_BeliefMapNav_Fig4.pdf) · [论文第 9 页](https://arxiv.org/pdf/2506.06487v1#page=9)

三张一致视角的地图可以把信念变化与空间位置对应。

**区别：** 与 Fig.12 表达同一类机制，不应当作独立研究证据；本图较扁，宜优先看 Fig.12。

### 35 · BeliefMapNav Fig. 5 — 哪些图像区域能提供可靠负证据

[原图 PNG](figures/35_BeliefMapNav_Fig5.png) · [裁剪 PDF](figures/35_BeliefMapNav_Fig5.pdf) · [论文第 16 页](https://arxiv.org/pdf/2506.06487v1#page=16)

深度图和置信图配对，适合参考“未看到”背后的可见性说明。

**区别：** 像素/体素可见性置信不同于你的离散区域样本覆盖；不能迁移颜色数值。

### 36 · BeliefMapNav Fig. 9 — 观测序列与后验信念变化同时展示

[原图 PNG](figures/36_BeliefMapNav_Fig9.png) · [裁剪 PDF](figures/36_BeliefMapNav_Fig9.pdf) · [论文第 19 页](https://arxiv.org/pdf/2506.06487v1#page=19)

地图轨迹、RGB 时间序列和连续色条共同讲清搜索过程。

**区别：** 图中后验有自己的算法定义；仅借鉴时空对齐形式。

### 37 · BeliefMapNav Fig. 10 — 第二个搜索场景的时序布局

[原图 PNG](figures/37_BeliefMapNav_Fig10.png) · [裁剪 PDF](figures/37_BeliefMapNav_Fig10.pdf) · [论文第 19 页](https://arxiv.org/pdf/2506.06487v1#page=19)

作为不同场景纵横比例下的备用排版，可以比较哪些信息适合缩小。

**区别：** 与 Fig.9 是同论文的另一个案例，不增加方法覆盖范围。

### 38 · BeliefMapNav Fig. 11 — 目标较远时的搜索序列

[原图 PNG](figures/38_BeliefMapNav_Fig11.png) · [裁剪 PDF](figures/38_BeliefMapNav_Fig11.pdf) · [论文第 20 页](https://arxiv.org/pdf/2506.06487v1#page=20)

多时刻局部图通过统一路径图定位，适合跨房间恢复的备选构图。

**区别：** 不宜把所有关键帧都塞入 Fig.8。

### 39 · BeliefMapNav Fig. 12 — 固定三维视角对照信念更新

[原图 PNG](figures/39_BeliefMapNav_Fig12.png) · [裁剪 PDF](figures/39_BeliefMapNav_Fig12.pdf) · [论文第 20 页](https://arxiv.org/pdf/2506.06487v1#page=20)

相同相机和场景下展示先验、可见性、后验，适合你最难画清的空间证据变化。

**区别：** 原方法是体素贝叶斯信念；你的方法是区域有界置信度，应保留自己的去重规则。

### 40 · UniGoal Fig. 4 — 探索、候选定位、验证三个阶段

[原图 PNG](figures/40_UniGoal_Fig4.png) · [裁剪 PDF](figures/40_UniGoal_Fig4.pdf) · [论文第 7 页](https://arxiv.org/pdf/2503.10630v3#page=7)

每列同时包含局部地图、目标信息和图结构，阶段转换非常清楚。

**区别：** 原文按图匹配状态转换，不能照搬为你的局部预算触发恢复。

### 41 · UniGoal Fig. 3 — 锚点关系和图纠正的局部示意

[原图 PNG](figures/41_UniGoal_Fig3.png) · [裁剪 PDF](figures/41_UniGoal_Fig3.pdf) · [论文第 5 页](https://arxiv.org/pdf/2503.10630v3#page=5)

配对锚点、目标投影、图结构修正采用统一节点颜色。

**区别：** 物体图匹配纠正与确认新实例不是同一个操作。

### 42 · CARe Fig. 1–2 — 初始地图出错后为何需要重规划

[原图 PNG](figures/42_CARe_Fig1-2.png) · [裁剪 PDF](figures/42_CARe_Fig1-2.pdf) · [论文第 2 页](https://arxiv.org/pdf/2409.04837v2#page=2)

将初次检索、重复失败的候选及重规划依据放在一起，与你预建地图场景很接近。

**区别：** 保留相邻 Fig.1–2 与短图注作为一个参考单元；属于两个图号，不应当成单幅算法图。

### 43 · L3MVN Fig. 5 — Habitat 中搜索目标的紧凑横条

[原图 PNG](figures/43_L3MVN_Fig5.png) · [裁剪 PDF](figures/43_L3MVN_Fig5.pdf) · [论文第 5 页](https://arxiv.org/pdf/2304.05501v2#page=5)

观测与小地图配对，适合简洁展示前沿切换和最终发现目标。

**区别：** 原图地图较小，应放大关键帧，且不包含实例图更新。

### 44 · ESC Fig. 5 — 为什么离开当前房间继续找

[原图 PNG](figures/44_ESC_Fig5.png) · [裁剪 PDF](figures/44_ESC_Fig5.pdf) · [论文第 14 页](https://arxiv.org/pdf/2301.13166v3#page=14)

RGB 观测与地图上的下一搜索方向上下对齐，常识线索能落在真实位置。

**区别：** 原图依赖房间/物体共现推理；本轮未单独确认该示例的仿真器。

### 45 · OpenFMNav Fig. 2 — 指令、观察与地图的闭环关系

[原图 PNG](figures/45_OpenFMNav_Fig2.png) · [裁剪 PDF](figures/45_OpenFMNav_Fig2.pdf) · [论文第 3 页](https://arxiv.org/pdf/2402.10670v2#page=3)

语义地图位于闭环中央，能够参考观测与下一轮探索的连接方式。

**区别：** 系统框图的模块较多，适合作为流程补充而非 Fig.8 的主视觉。

### 46 · HOV-SG Fig. 8 — 多房间、多楼层中的语义目标路线

[原图 PNG](figures/46_HOV-SG_Fig8.png) · [裁剪 PDF](figures/46_HOV-SG_Fig8.pdf) · [论文第 9 页](https://arxiv.org/pdf/2403.17846v2#page=9)

大场景保留空间结构，目标和终点局部图从场景中直接引出，适合主地图。

**区别：** 定位和层次查询不代表失败恢复；多楼层对 Fig.8 若无必要可省略。

### 47 · HOV-SG Fig. 7 — 房间层、楼层、物体层叠在真实地图上

[原图 PNG](figures/47_HOV-SG_Fig7.png) · [裁剪 PDF](figures/47_HOV-SG_Fig7.pdf) · [论文第 8 页](https://arxiv.org/pdf/2403.17846v2#page=8)

可参考如何将抽象图节点附着在可辨认的空间位置。

**区别：** 节点很多；用于 Fig.8 更新示意时应只取相关房间和目标实例。

### 48 · VLMaps Fig. 2 — 自然语言空间关系在场景中直接落地

[原图 PNG](figures/48_VLMaps_Fig2.png) · [裁剪 PDF](figures/48_VLMaps_Fig2.pdf) · [论文第 2 页](https://arxiv.org/pdf/2210.05714v4#page=2)

把语言中的左右、前后等关系画在真实目标附近。

**区别：** 多步骤空间目标导航不是未检出恢复。

### 49 · VLMaps Fig. 3 — 观测语义如何融合到三维地图

[原图 PNG](figures/49_VLMaps_Fig3.png) · [裁剪 PDF](figures/49_VLMaps_Fig3.pdf) · [论文第 3 页](https://arxiv.org/pdf/2210.05714v4#page=3)

图像、深度和地图之间的连接清晰，可参考观测到地图的流程组织。

**区别：** 特征地图不是物体实例图；无需把完整建图系统搬入 Fig.8。

### 50 · SEEK Fig. 3 — 房间信念与跨区域搜索策略

[原图 PNG](figures/50_SEEK_Fig3.png) · [裁剪 PDF](figures/50_SEEK_Fig3.pdf) · [论文第 7 页](https://arxiv.org/pdf/2405.09822v2#page=7)

每个房间标概率，用局部循环箭头表示当前房间搜索，适合恢复区域选择。

**区别：** 这是全局搜索策略示意；本图不应被称为 Habitat 运行截图。

### 51 · SEEK Fig. 9 — 连续未找到后房间概率如何变化

[原图 PNG](figures/51_SEEK_Fig9.png) · [裁剪 PDF](figures/51_SEEK_Fig9.pdf) · [论文第 14 页](https://arxiv.org/pdf/2405.09822v2#page=14)

多次规划步骤保持相同房间顺序，能够比较负观测后概率重分配。

**区别：** 该例为 Gazebo；曲线机制和你区域样本覆盖不同，且柱图不适合占据主视觉。

### 52 · ConceptGraphs Fig. 2 — 多视角观测融合为三维实例与场景图

[原图 PNG](figures/52_ConceptGraphs_Fig2.png) · [裁剪 PDF](figures/52_ConceptGraphs_Fig2.pdf) · [论文第 3 页](https://arxiv.org/pdf/2309.16650v1#page=3)

从 RGB 分割到对象点云和图节点，连接方式适合实例融合部分。

**区别：** 这是静态场景图构建，不是执行后确认新增实例的完整实现。

### 53 · ConceptGraphs Fig. 3 — 检索候选后再用视觉验证

[原图 PNG](figures/53_ConceptGraphs_Fig3.png) · [裁剪 PDF](figures/53_ConceptGraphs_Fig3.pdf) · [论文第 6 页](https://arxiv.org/pdf/2309.16650v1#page=6)

候选物体与现场图像直接对照，适合解释为什么还需要验证。

**区别：** 所选示例是真实机器人；验证逻辑不是你任务的终止条件。

### 54 · DovSG Fig. 3 — 新观测使地图局部发生变化

[原图 PNG](figures/54_DovSG_Fig3.png) · [裁剪 PDF](figures/54_DovSG_Fig3.pdf) · [论文第 5 页](https://arxiv.org/pdf/2410.11989v6#page=5)

观测、场景变化和局部地图更新靠近摆放，适合你的 Fig.8 末端。

**区别：** DovSG 含重定位和场景适应；你的图必须明确成功验证在执行中、确认实例插图在执行后。

### 55 · DovSG Fig. 2 — 对象地图与图节点的双层表达

[原图 PNG](figures/55_DovSG_Fig2.png) · [裁剪 PDF](figures/55_DovSG_Fig2.pdf) · [论文第 4 页](https://arxiv.org/pdf/2410.11989v6#page=4)

物体语义、三维几何和图关系有对应线，适合新增对象与边的画法。

**区别：** 全系统结构太复杂，不宜整体移植到恢复机制图。

### 56 · SayPlan Fig. 2 — 空间场景与层次节点共同展示

[原图 PNG](figures/56_SayPlan_Fig2.png) · [裁剪 PDF](figures/56_SayPlan_Fig2.pdf) · [论文第 4 页](https://proceedings.mlr.press/v229/rana23a/rana23a.pdf#page=4)

楼层、房间、资产、对象的层次连接直观，适合表达锚点仍保留。

**区别：** 静态分层场景图，不包含新增实例确认。

### 57 · InnerMonologue Fig. 1 — 动作旁边立即呈现成功或失败反馈

[原图 PNG](figures/57_InnerMonologue_Fig1.png) · [裁剪 PDF](figures/57_InnerMonologue_Fig1.pdf) · [论文第 2 页](https://proceedings.mlr.press/v205/huang23c/huang23c.pdf#page=2)

反馈紧挨对应动作，恢复过程的因果关系容易沿阅读顺序展开。

**区别：** 非 VLN-CE，文字多；建议只借反馈摆放，不用对话框取代空间信息。
