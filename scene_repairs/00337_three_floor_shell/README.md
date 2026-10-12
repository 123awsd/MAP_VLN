# 00337 三层探索仿真：可见虚拟封闭面

本配置针对 CFVBbU9Rsyb.basis.glb。原 GLB、原导航网格、原可用视频保留。
这是明确的虚拟场景封闭层，不宣称恢复了扫描缺失的真实建筑细节。

## 修复内容

- 加固二层上方约5.15m的封闭面，拦截此前从门框附近直接升出模型的路径。
- 为第三层外侧添加墙面、约7.96m的顶面和原地面下方的底面，避免外墙与底面之间留可穿越的缝。
- 根据原导航网格保留楼梯开口：参考 FALCON 坐标 x[-4.95,0.35]、y[7.85,9.10]。这些坐标只用于场景几何，不输入规划器目标选择。
- 使用 Habitat 静态对象加载，虚拟面同时参与RGB、深度与场景碰撞查询；没有修改或伪造点云测量。
- 修复后的仿真执行器对实际场景做采样扫掠检查；被几何挡住的指令只能前进到最后可达位置。
- 独立试跑容器将实际受阻反馈接入原有 blockViewpoint 和从里程计重规划流程，避免持续追逐同一受阻目标。
- 候选生成、观察收益评分、任务顺序和路径规划的源文件保持当前 v4 版本；哈希记录位于 outputs/exploration_scene_repaired_trial/planner_source_hashes.json。

OBJ 顶点使用 Habitat 世界坐标。repair.json 中的 FALCON 范围是使用指定起点换算的说明坐标。模型哈希不匹配时加载器拒绝套用这些修复面。

## 检查

在 /home/uav/map_VLN 执行：

```bash
PRE_MAP_VLN/.envs/habitat/bin/python PRE_MAP_VLN/scene_repairs/00337_three_floor_shell/validate_overlay.py
```

检查实际SDK深度响应、原穿模指令拦截、真实楼梯14段路径通行。
validation.json 保存最近的检查结果。after_rgb.jpg / before_rgb.jpg 是相同相机的真实渲染对照。

## 使用

在原 Habitat 运行命令中增加：

```bash
--scene-repair-config /home/uav/map_VLN/PRE_MAP_VLN/scene_repairs/00337_three_floor_shell/repair.json
```

启用修复配置同时启用场景碰撞检查。单独检查原模型而不加虚拟面，可用 --scene-collision-check。
未传这些选项时保留原来的跟随方式。

完整试跑需要仿真受阻反馈订阅。当前已在独立容器 exploration-three-floor-trial 接通；对应源文件、原始备份和初始化脚本位于 outputs/exploration_scene_repaired_trial。其他容器不会自动改变。

仿真执行检查使用规划坐标对应的相机中心及0.12m采样球半径，属于仿真工具的近似检查，不是对真实无人机完整外形的验证。
