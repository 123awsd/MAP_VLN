# Fig. 6 候选视角生成图

本目录包含当前论文 Fig. 6 的最终场景、模型、脚本、图例及输出。

## 最终输出

- `candidate_selection_two_insets.png`：完整预览。
- `candidate_selection_two_insets_highres.png`：高清版本。
- `candidate_selection_two_insets.pdf`：论文图件。
- 论文引用文件：`../../PRE_MAP_VLN/论文/TASE_英文初稿/figures/viewpoint_generation_en.pdf`。

## 最终 Blender 场景

- `algorithm_scene.blend`：全局场景。
- `occlusion_projected_fov.blend`：三维桌椅、二维视场遮挡示意。
- `collision_explanation.blend`：放大 1.8 倍且已按真机配色的无人机。
- `assets/`：下载的桌椅 GLB；`solidworks_model/`：STEP 转换的无人机模型。

## 重现当前定稿

最终材质和姿态保存在以上三个 blend 中；直接渲染最终场景，避免覆盖手工材质调整：

```bash
cd /home/uav/map_VLN/画图/候选视角生成_新版
blender -b -t 6 --python render_final_scenes.py
python compose_polished.py
cp candidate_selection_polished.png candidate_selection_five_chairs.png
python compose_two_insets.py
```

`compose_two_insets.py` 负责比例、圆角虚线框、局部红叉和四项英文图例。`cross_positions.json` 保存红叉位置。

## 生成与调整脚本

`replace_edelweiss.py` 导入餐桌；`d435_rgb_scene.py` 生成全局候选与投影，依赖仓库 `PRE_MAP_VLN/stage2/candidate_poses.py`；`occlusion_inset_geometry.py` 计算遮挡；`render_inset_projected_fov.py` 生成视场示意；`render_step_collision.py` 和 `render_collision_explanation.py` 生成碰撞局部。其余配色及放大脚本为调整过程记录，重复运行放大脚本会再次缩放，不用于重现定稿。

视场填色是水平截面的遮挡示意；无人机放大仅为展示。碰撞红色高亮采用表面邻近标记。灰点、保留候选、受遮挡视角和碰撞剔除的图例与后续规划选择区分。
