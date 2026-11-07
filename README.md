# LogicFly paper

论文的独立工作目录。后续正文、图件和图注修改以这里为准。

```text
LogicFly-paper/
├── manuscript/          正文、参考文献、IEEE 模板与编译输出
├── figures/
│   ├── final/           正文实际引用的 9 张图
│   └── source/          按图号整理的绘图脚本、可编辑文件和工作稿
├── references/          参考论文、参考配图及索引
├── Makefile
└── README.md
```

## 编译与阅读

在本目录执行：

```bash
make pdf
```

正文为 [manuscript/root.tex](manuscript/root.tex)，编译结果为 [manuscript/root.pdf](manuscript/root.pdf)。
需要 XeLaTeX、BibTeX、latexmk 和 TeX Gyre 字体。当前机器已具备这些依赖。
中间文件集中在 `manuscript/build/`；正文通过相对路径读取 `figures/final/`，独立编译无需原系统代码、数据集或绘图环境。

```bash
make check     # 检查正文引用的图件和项目结构
make bundle    # 生成可独立编译的投稿源码包
```

源码包输出为 `manuscript/LogicFly_TASE_source.zip`，包含相同的 `manuscript/`、`figures/final/` 层级和根目录 Makefile。解压后运行 `make pdf` 即可。

## 图件索引

| 图号 | 内容 | 定稿文件，位于 figures/final/ | 源文件，位于 figures/source/ |
| --- | --- | --- | --- |
| 1 | 真机执行主图 | `真机实验1.pdf` | `fig01_real_world/` |
| 2 | 系统总览 | `systemoverview.pdf` | `fig02_system_overview/` |
| 3 | 第一阶段建图流程 | `探索流程图.pdf` | `fig03_mapping_pipeline/` |
| 4 | 房间分割 | `房间分割.png` | `fig04_room_partitioning/` |
| 5 | 条件任务图 | `conditional_task_graph.pdf` | `fig05_task_graph/` |
| 6 | 候选视角生成 | `视角选择.png` | `fig06_viewpoint_generation/` |
| 7 | 联合规划与轨迹生成 | `联合规划_1.png` | `fig07_joint_planning/` |
| 8 | Habitat 执行示例 | `habitat_long_instruction_en.png` | `fig09_habitat_execution/` |
| 9 | 多场景真机实验：居家多目标、实验室三目标及桌面目标搜索 | `real_world_fig8_style.pdf` | `fig09_11_real_world/` |

`final/` 是正文引用图件的唯一入口。源目录里的草稿、预览和新变体不会自动替换定稿。
Fig. 4 最新的单行四幅工作稿位于 `fig04_room_partitioning/single_row/`；第二层倾斜分区及 SAM 3.1 目标框叠加图位于 `fig04_room_partitioning/oblique_l2/`。

每幅图的 `SOURCE_INFO.json` 记录原始来源；[figure_manifest.json](figures/source/figure_manifest.json) 记录定稿文件、图号和迁移时的校验值。

## 绘图和实验数据

绘图源文件包含 SVG、Blender 场景、版面脚本及相关图片。SVG/Blender 可直接编辑；部分脚本重算候选、分割或轨迹时仍需原项目的算法、环境及实验数据。源脚本保留其原来的数据路径约定，迁移本身没有重写这些算法或声称所有绘图都已脱离原始数据。

代码项目仍在 `/home/uav/map_VLN/PRE_MAP_VLN`。大模型权重、原始视频、点云及运行缓存继续由代码项目和原素材目录管理。
[migration_inventory.json](figures/source/migration_inventory.json) 记录文件原路径、新路径和留在上游的输入，便于追溯。

## 写作约定

- 目标识别和 MINCO 真机实现的论文描述以 `origin/real-fly` 为事实来源；详见 [IMPLEMENTATION_SOURCE.md](IMPLEMENTATION_SOURCE.md)。
- 正面、准确地描述方法、贡献和结果，避免无必要的制作备注与重复免责声明。
- 方法和系统实现章节写完整算法：目标识别为 SAM 3.1--Boxer，轨迹优化为 A* 几何引导、安全走廊约束和 MINCO 连续优化。真机当前接线程度和实验边界只在 Physical UAV Experiments 中仔细说明。
- 对比和消融结果按当前稿保留，补充时使用已核实的实验数据。
- 图件修改先在对应 `source/figXX_.../` 内完成，确认后再更新 `final/` 和正文图注。

## 迁移记录

2026-10-28 从 `PRE_MAP_VLN/论文/TASE_英文初稿/` 整理至此。正文内容和 9 张引用图件均保留当前版本，仅调整 LaTeX 图件路径。
旧目录与原绘图目录暂时保留为历史来源，后续论文维护使用本目录。

## 版本管理

本目录对应 `123awsd/MAP_VLN` 仓库的 `paper` 分支；该分支根目录采用上述论文结构。
系统代码由 `real-fly` 等代码分支维护。论文和代码分支按文件或明确的提交同步，不进行整分支合并。
后续论文提交和上传均在本目录进行。
