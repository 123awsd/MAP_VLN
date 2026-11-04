# 真机主图 V7：实测占用体积总览与恢复搜索细节

本稿在扩展检查 21 例论文图件后重新制作，重点参考 FALCON 的“总览＋局部放大”结构和 USS-Nav 的紧凑观察列。完整研究记录位于论文目录下 `references/figure_examples/主图扩展研究_20261030/`。

## 文件

- `preview.jpg`：1800 像素宽预览。
- `main_figure.png`：3600 × 1659，508 dpi。
- `main_figure.pdf`：180 × 82.95 mm；文字、轨迹和相机视锥为矢量。
- `main_figure.svg`：自包含可编辑图件。
- `build.py`：从已经准备好的 `assets/` 重新生成全部导出，不需要原始运行数据。
- `render_views.py`：在 Blender 中生成三维总览和局部放大，同时导出与两台虚拟相机严格对应的投影坐标。

```bash
/usr/bin/python3 build.py
```

系统 Python 依赖 NumPy、Pillow、Cairo 和 PyGObject/Rsvg。

## 本版实际改变

三维底图由前几版的 RGB-D 点云显示，换成归档实测占用体积的表面渲染。输入是 `真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/voxel_snapshot/voxel_map.npz` 的 `raw_occupancy_zyx == 100` 单元，分辨率为 0.2 m。

对实际占用单元直接提取外边界面，去除相邻占用单元之间不可见的内部面；没有从房间多边形外推墙体，没有补造地板或家具。显示范围为世界 x ∈ (−43,21)、y ∈ (−10,18)、z ∈ (−0.3,1.9) m，保留 14,109 个占用单元。光照和材质只影响显示。原始地图和运行记录保留在论文项目之外。

几何生成脚本与可编辑 Blender 场景位于原工作区 `画图/主图/体素地图视觉试验_20261030/`。`render_views.py` 用该派生场景重新渲染两种视角，不改变地图坐标。

## 轨迹、相机和图像依据

- 总轨迹：记录的 `/ekf_quat/ekf_odom` 机体位置，按每 8 个样本取 1 个显示，未平滑或移动。它是机载估计，不是外部真值、规划路线或跟拍摄像机路径。
- 恢复段：阶段日志第 2 个到达锚点至第 4 个到达锚点，对应从原笔记本位置检查到跨房间找到目标。右侧只绘出这一段记录位置。
- 视锥：基于归档 `body_to_color_optical` 外参和相应记录位姿，统一使用 1.25 m 光学深度作显示。
- 投影：通过 Blender 相机的 `world_to_camera_view` 逐点导出。地图渲染与矢量路线使用同一相机，裁剪后按统一比例缩放。为便于阅读，路线和视锥作为二维投影叠加在地图上，不作墙体遮挡测试。
- 下排第三人称照片：跟拍视频 53、69、129、163 秒的笔记本、工具箱、微波炉与水杯检查。
- 右侧机载图：编码时间 35.5 秒的原位置检查、46 秒的笔记本目标视图。两个视频按语义事件近似关联，不声称严格硬件同步。详细出处见 `assets/evidence_sources.json`。
- 顶部指令：原始四任务指令的英文缩写。微波炉图仅表示检查，不据此声称食物加热状态已被验证。

没有房间名称或分区标签；不叠加多个跨帧无人机实例；不依赖固定第三人称机位。正文使用的 `figures/final/real_world_execution.png` 未被替换。

## Suggested caption

Real-world multi-task execution with LogicFly. The overview shows the recorded onboard EKF flight trajectory and calibrated observation poses over the archived occupancy map, rendered as a cutaway of measured occupied volume. Blue denotes the executed flight and orange highlights recovery search from the initial laptop check (A) to the subsequent observation (1). The right column enlarges this recovery segment and shows onboard observations before and after finding the laptop. The lower third-person photographs show laptop, toolbox, microwave and cup inspections.
