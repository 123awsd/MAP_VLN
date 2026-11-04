# 真机主图 V4：白底三维轨迹

按作者选择的“白底论文图：三维轨迹为主，实拍画面作辅助”重新构图。独立工作稿，未替换 `figures/final/` 或修改正文。

## 文件

- `preview.jpg`：1800 × 1145 预览。
- `main_figure.png`：3600 × 2290，508 dpi。
- `main_figure.pdf`：180 × 114.5 mm；轨迹、视锥、标记、参考网格和文字为矢量。
- `main_figure.svg`：自包含可编辑图件，照片与点云渲染已嵌入。
- `build.py`：版面源文件，使用已经准备好的 `assets/` 即可重新导出。
- `render_scene.py`：从归档实测点云和飞行记录重新生成两个视角、投影元数据和相机视锥，需要原工作区数据。

## 构图

白底主图展示彩色实测 RGB-D 场景和连续记录轨迹；下方只保留一排四个目标的真实观测画面。青色表示正常执行轨迹，橙色突出恢复搜索。标记 1–4 对应下方观测图；A 是初始笔记本检查位置。没有房间名称、分区色块、照片渐变、广告式大标题或地图与实拍的混合背景。

本版保留更多实测墙体与家具高度（世界 z 范围 −0.25 至 2.65 m），选择方位角 −10°、俯视角 50°。地图数据和轨迹没有重新估计或手工挪动。

## 来源与显示约定

- 点云来自 `画图/主图/local_vggt/mapping_rgbd/mapping_rgbd.ply`，是归档 RGB-D 按已记录光学相机位姿反投影得到的实测点云，不是生成式图像或纯 VGGT 输出。
- 路线来自 `/ekf_quat/ekf_odom` 记录的机体位置，每 8 个样本取 1 个用于绘图，保持原有坐标与高度。它是机载位置估计，不是外部真值，也不是规划路线或跟拍摄像机轨迹。
- 五个观察视锥使用相应日志阶段时刻的记录位姿，以及归档的 `body_to_color_optical` 外参；统一使用 1.45 m 光学深度作显示尺度。几何参数位于 `assets/scene_az-10.json`。
- 浅灰网格是 z = −0.125 m 的米制坐标参考平面，不是补建的地面几何；网格与比例尺只表达空间尺度。
- 目标照片沿用已提取真机画面：机载视频 46 秒的笔记本，跟拍视频 69、129、163 秒的工具箱、微波炉和水杯。矩形裁剪与缩放保持真实内容，不增加无人机或目标实例。
- 不同视频以语义事件近似关联，不声称硬件严格同步。微波炉画面仅展示检查动作。
- 顶部指令为原四任务指令的英文缩写。

## 生成

```bash
/usr/bin/python3 build.py
```

依赖系统 Python 的 NumPy、Pillow、Cairo 和 PyGObject/Rsvg。原始视频、原始点云和运行记录保留在论文项目之外。

## Suggested caption

Real-world multi-task execution with LogicFly. The recorded onboard EKF trajectory and calibrated observation poses are shown over the measured RGB-D scene. Orange highlights recovery search from the initial laptop check (A) to the subsequent laptop observation (1). The lower images show the laptop found from the onboard camera and third-person views of toolbox, microwave and cup inspections. The pale ground grid is a metric coordinate reference.
