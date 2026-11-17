# Room segmentation figure

论文配图，**88 × 76 mm，单栏宽；两行布局，上排 a–c、下排 d–e**。英文标注全部加粗，原始实测点云与四幅连续完整楼层地图，明亮区域色。PDF/SVG 中地图和文字为矢量，点云以 600 dpi 栅格化以控制文件大小；PNG 为 600 dpi，预览为 400 dpi。2026-10-24 按用户要求改为两行单栏，并更新 TASE 工作稿中的 Fig.4 和编译 PDF。所有子图保持相同完整楼层范围、比例与方向。

## 图的内容

- **(a) Point cloud**：墙提取实际使用的第二层实测点云缓存，该层全部缓存点均展示，无高度过滤、空间裁剪或下采样。采用与后续地图一致的 XY 俯视方向；近水平表面显示得浅一些以便看清竖直结构，未叠加提取墙或人工轮廓。
- **(b) Wall constraints**：提取后墙体约束与继承的原候选域。
- **(c) Supported space**：原候选域与独立观测支持取交集。支持来自近水平表面、多高度 FREE、多帧深度射线或轨迹，任一来源成立即可。玫瑰色表示从候选域排除，不表示室外或障碍物。
- **(d) Core seeds**：距离变换得到的连通内部核心，经过源代码中的净距和面积门槛。彩色种子是实际区域，白色细线帮助识别核心轮廓。
- **(e) Room regions**：受支持域约束的分水岭输出，包含代码中的几何合并检查。种子与分区颜色对应，灰色保留未分配的支持空间。

展示第二层的完整地图，五幅图范围、尺度和方向一致。**不裁剪房间、墙体或候选范围**；显示边界自动包含全部原始点云和非空栅格（包括孤立墙点），仅省去外围空白边距。版面加宽，避免把整层地图硬塞进半栏。颜色是区域身份，不是语义类别。

将四类支持证据、合并条件等解释移入图注，避免文字框和长连线占据图面。未展开上游墙提取和候选域构建细节。内部边界未手工修饰，未将真实栅格替换成理想化矩形。

配色采用浅蓝、薄荷绿、暖黄、淡紫、杏色和青色，种子与所属房间始终对应。点云使用蓝灰色，墙约束和支持域采用更浅的底色，让输出区域成为视觉重点。

## 代码与验证

分割依据为 `PRE_MAP_VLN/scripts/classify_coverage700_rooms_multiview.py` 的 `make_inputs()` 实际读取的 `outputs/coverage700_wall_room_preview/wall_structural_rooms/L*_constrained.npz`。

实现为 `scripts/run_wall_structural_rooms.py`；分水岭函数来自 `scripts/segment_rooms_wall_guided.py`；墙点来自 `scripts/run_wall_structural_trial.py` 生成的 `wall_structural_evidence`。这是实测离线检查点，不声称已替换默认在线分割。

没有混用旧 `runtime/room_segmentation_wall_guided` 的界面长度合并规则，也未使用 `shell_filtered_final`、未晋升的 `wall_structure_partition` 或人工修正边界。

绘图程序抽取实际源代码中的分割语句执行，仅插入中间变量快照；三层均验证墙投影、支持域、种子和最终标签与缓存逐格一致，无分区穿墙。源码与输入 SHA256、完整显示范围与无截断检查记录于 `mechanism_evidence.json`。

示例中合并检查与碎片隐藏均未触发，不编造合并前后的变化。该结构分区包含家具所在空间，不能当成无碰撞通行地图。

## Suggested caption

**Wall-supported room partitioning on the complete measured second floor.** (a) Recorded point cloud, shown in an XY projection. (b) Extracted wall barriers constrain the candidate domain. (c) Independent support from horizontal surfaces, multi-height free-space observations, multi-frame depth rays, or the recorded trajectory filters the candidate space; rose cells are excluded. (d) Connected high-clearance cores initialize the seeds. (e) Masked watershed propagation followed by geometric merge checks produces the room regions. Seed and region colors correspond; gray denotes supported but unassigned cells. All panels display the complete floor at the same scale and orientation; no point-cloud or map content is cropped. Horizontal surfaces in the point-cloud panel are rendered faintly to expose vertical structure.

## Visual reference

参考用户提供的 `画图/画图参考/第一阶段流程图.jpg` 的图形化流程表达，以及 [ROSE², Fig. 1](https://arxiv.org/pdf/2203.03519) 中依次展示空间表示变化的组织方式。只参考表达方式，未复制论文图片，也未将该论文算法当作本项目算法。

## Reproduce

```bash
cd /home/uav/map_VLN
/home/uav/miniconda3/envs/env_isaaclab/bin/python 画图/房间分割/render_figure.py
```

依赖现有实测缓存；绘图脚本只生成本目录的图件和审计记录，不运行探索或修改分割输入。此次论文集成为单独执行的复制与正文编辑；以后重新绘图不会自动更新论文。

Fig.4(b) 使用深蓝墙体与 0.45 pt 显示轮廓，候选区域底色减淡以突出墙约束；描边仅用于显示，不修改墙掩码或分割结果。
