# 真机主图 V3：实拍主场景与三维轨迹嵌图

这是结合 OnFly、AirHunt、See, Point, Fly 和 GRaD-Nav++ 实际图像研究后的新构图草稿。研究记录与原图来源位于论文目录下的 `references/figure_examples/主图参考_20261029/`。

## 交付文件

- `preview.jpg`：1800 × 1070 预览。
- `main_figure.png`：3600 × 2140，508 dpi。
- `main_figure.svg`：自包含可编辑图件，文字和路线为矢量，照片与点云渲染为嵌入图像。
- `main_figure.pdf`：180 × 107 mm，文字和路线为矢量。
- `scene_angles.jpg`：选择嵌图角度时实际查看的六种场景投影。

## 版面与依据

实拍主场景来自原跟拍视频 33 秒处的已提取画面，裁剪范围为原 3840 × 2160 图像中的 `(0, 500, 3840, 1940)`。照片保留当时拍到的无人机与空桌面，没有跨帧叠加无人机，没有生成式补绘。指令是四任务原始指令的英文缩写，不是全文逐字转写。

轨迹嵌图使用归档的实测 RGB-D 点云 `画图/主图/local_vggt/mapping_rgbd/mapping_rgbd.ply`。它由 334 个建图 RGB-D 帧按归档光学相机位姿反投影并体素聚合得到，不是纯 VGGT 输出，也没有用生成式图像美化几何。为了看到内部结构，仅作高度裁切和视野裁剪，具体参数写入 `assets/scene_az-15_el55.json`。图中小幅全局对比度调整仅用于显示。

路线直接取自 `/ekf_quat/ekf_odom` 记录的机体位置，转换到与点云一致的显示坐标系；未使用手绘路线、规划 MINCO 路径或跟拍摄像机运动代替真实飞行记录。标记使用阶段日志时刻的最近位姿。它是机载估计，不是外部动捕真值。

A 对应最初的笔记本检查；橙色轨迹为恢复搜索段；1–4 为后续四个目标的检查位置。下方第 1 张是机载 RGB 视频 46 秒处的目标视图，其余三张来自跟拍视频 69、129、163 秒。两个视频的事件对应为近似语义关联，不声称严格硬件同步。微波炉仅标为检查目标，不据图声称食物已加热完毕。

地图嵌图与主照片属于不同视角，嵌图位置只是版面安排，没有声称二者经过透视配准。主照片上没有投影或伪造飞行轨迹。

## 重新生成

准备好的派生资产已保存于 `assets/`，重新排版无需读取原始点云或视频：

```bash
/usr/bin/python3 build.py
```

系统 Python 需要 NumPy、Pillow、PyGObject/Rsvg、Cairo。`render_scene.py` 用于从原始实测点云重新渲染六个角度，需要原工作区中的上游点云与记录轨迹；其输出是派生图像和投影元数据，原始点云和视频仍保留在论文项目外。

尚未替换 `figures/final/` 或修改正文。

## Suggested caption

Real-world execution of a multi-task language instruction with LogicFly. The UAV first inspects the expected laptop location and continues searching when the laptop is absent. The inset shows the recorded onboard EKF flight trajectory over a measured RGB-D reconstruction, with the recovery segment highlighted in orange. Numbered target observations show the laptop found from the onboard camera, followed by toolbox, microwave and cup inspections. The main photograph and reconstruction inset are separate views of the real experiment.
