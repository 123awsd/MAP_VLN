# 三维语义地图九秒动画

本程序只读取已有结果并渲染展示，不运行建图、检测、分割或分类。场景中点云、轨迹、房间和物体框都沿用输入的 FALCON XYZ 米制坐标。楼层没有人为拉开；房间薄片位于配置文件 `floor_z_m` 指定的真实楼层高度。

## 重新生成

编辑 `config/semantic_map_animation_9s.json` 中的四个输入路径、楼层高度和输出目录，然后运行：

```bash
python scripts/make_semantic_map_video.py --config config/semantic_map_animation_9s.json
```

先看四张关键帧、暂不渲染全片：

```bash
python scripts/make_semantic_map_video.py --config config/semantic_map_animation_9s.json --preview-only
```

已有完整 PNG 序列时只重新压制 MP4：

```bash
python scripts/make_semantic_map_video.py --config config/semantic_map_animation_9s.json --encode-only
```

脚本需要 Blender 4.0 及 Python `numpy`、`imageio_ffmpeg`。Blender 可直接打开输出目录中的 `semantic_map.blend` 调整灯光、颜色和镜头。

## 输入约定

| 数据 | 支持格式 | 必需内容 |
|---|---|---|
| 点云 | NPZ、NPY、ASCII PLY/PCD | `points` 或 `p0,p1,...`，每点 XYZ |
| 物体 | JSON、Boxer 融合 CSV | JSON 的 `objects[].corners_xyz` 为 8×3；CSV 使用世界坐标中心、尺寸、四元数 |
| 房间 | 单个 NPZ 或包含 `L*_constrained.npz` 的目录 | `labels`、`origin_xy`、`resolution` |
| 轨迹 | NPY、JSONL、CSV | 本项目 NPY 是 `[time,x,y,z]`；也支持单独 `N×3` XYZ |

四类数据必须已经在同一坐标系中。程序会检查轨迹范围与点云是否基本一致，但不会做 ICP、镜像、平移或缩放。点云显示最多取 `max_render_points` 个真实点；这只降低可视化负载，不改变点位。房间区域从现有标签网格绘制，物体框使用已经输出的真实尺寸与旋转。

## 动画规格

1920×1080、30 FPS、270 帧。0–3 秒环绕高度渐变点云；3–5 秒类别彩色物体框渐显；5–7 秒点云材质连续淡出；7–9 秒正交相机平滑转成侧视，同时保留真实蓝色轨迹和半透明房间区域。正交画幅由输入边界自动计算。房间和物体文字标签由配置文件开关控制，默认关闭以保持画面简洁。
