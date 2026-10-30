# Figure sources

图件按 Fig. 1–9 编号归档；定稿文件与图号的对应关系见 `figure_manifest.json`。
每个目录的 `SOURCE_INFO.json` 记录迁移前的位置和当前安装图件的校验值。

## 日常修改

1. 在对应图号目录中修改版面脚本、SVG 或 Blender 场景。
2. 在源目录中生成和查看预览。
3. 图稿确认后，将选定输出复制到 `../final/` 中对应的既有文件名。
4. 同步修改 `../../manuscript/root.tex` 的图注，执行根目录 `make pdf`。

已有的原始 README 保留重绘流程和参数。原脚本涉及数据集或算法时，仍沿用其原始路径约定；迁移没有将这些实验代码复制为另一份运行系统。

## 当前工作稿

- `fig04_room_partitioning/single_column/`：正文当前采用的单栏两行两列版本，蛇形流程顺序，去掉底部图例，图内标题为 8.5 pt。
- `fig04_room_partitioning/single_row/`：保留底部图例的原单行四幅版本，合并支持域和内部核心。
- `fig04_room_partitioning/oblique_l2/`：第二层倾斜分区、SAM 3.1 融合 3D 框及各配色版本。
- `fig06_viewpoint_generation/`：候选视角生成的 Blender 场景、三维模型与版面脚本。
- `fig08_semantic_recovery/clear_rooms/`：当前物品核验与语义恢复图稿。

上述目录中的新工作稿不会自动覆盖 `../final/` 的当前论文版本。

## 外部输入

代码仓库：`/home/uav/map_VLN/PRE_MAP_VLN`。
原始视频和素材：`/home/uav/map_VLN/素材处理后`、`/home/uav/map_VLN/画图` 等原目录。
绘图依赖的观测、地图和算法环境仍由这些来源管理。

`migration_inventory.json` 逐项记录了复制文件及其原位置，并记录未复制的原始录制、点云、权重、临时缓存和备份文件。
