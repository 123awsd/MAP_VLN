# Fig. 8：真实多房间场景中的恢复与地图更新

本版按“完整住宅的真实感 + 易读的机制示意”重绘。保留两间卧室、客厅、厨房、卫浴及连接空间，不再只截取最初两个房间。论文原图尚未替换。

## 交付文件

- `fig8_multiroom_en.pdf`：英文论文版本，184 × 133.22 mm；场景和相机图为位图，标注、轨迹、节点和文字为矢量。
- `fig8_multiroom_en.svg`：包含图像的自包含可编辑版本。
- `fig8_multiroom_en.png`：英文预览。
- `fig8_multiroom_zh.png`：中文标注预览；另附相同布局的中文 PDF / SVG。
- `fig8_textured_scene.blend`：已打包纹理的完整住宅剖切场景。
- `caption.tex`：与本版构图对应的英文图注。

## 场景与示意边界

底图来自原始 HM3D `00166-RaYrxWt5pR1` 纹理网格。天花板和 1.30 m 以上墙体作展示剖切，家具、门口和房间之间的布局保留；纹理的饱和度统一降到 0.68，以突出轨迹。图像没有经过生成模型补绘。

三张主要观测图和右侧两张靠垫细节，取自已有 Habitat 高清视图，在 SVG 内裁剪。第一、二次未检出视图均检查同一扶手椅；第三次视图展示客厅沙发上的靠垫。

路线是为表达设计的示意路线，不是本文算法的实验输出。使用该场景官方 navmesh 作为通道位置参考，保留经过实际门口的关系；曲线、视角数量和覆盖点作可读性简化。`geometry_check.json` 检查了平滑路线在地面上的投影与可通行空间的关系，并非飞行碰撞或实验成功率验证。

为说明新增实例，图中假定所示靠垫未包含在初始物体地图里。这是示意初始状态，不宣称来源录像中的真实运行曾将这个靠垫作为新实例插入。原有任务日志和场景图未作修改。

## 保留的机制关系

1. 一次未检出不直接结束定位任务，先换局部视角。
2. 达到局部限制后，依据地图形成客厅沙发这一替代假设，任务保持相同。
3. 新覆盖引入负证据；重复覆盖不重复降低区域信念。找不到靠垫不否定扶手椅本身。
4. 验证成功在执行过程中完成任务；多视角一致的新实例在执行后加入场景图。

## 重现

在项目根目录执行：

```bash
PRE_MAP_VLN/.envs/habitat/bin/python PRE_MAP_VLN/scripts/decode_hm3d_basis_glb.py \
  /shared/PRE_MAP_VLN_hm3d7_v2/scenes/hm3d/train/00166-RaYrxWt5pR1/RaYrxWt5pR1.basis.glb \
  /tmp/fig8_00166_decoded
blender -b -t 8 --python 画图/语义恢复与地图更新/multiroom_redraw/render_scene.py
/usr/bin/python3 画图/语义恢复与地图更新/multiroom_redraw/compose_figure.py
```

源数据、裁剪窗口、输出哈希和验证结果见 `provenance.json`。最小字号约 6.26 pt（184 mm 图宽），主要标签约 7–8 pt。
