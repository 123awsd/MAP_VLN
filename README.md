# LogicFly paper

这是 `/home/uav/map_VLN/PRE_MAP_VLN` 同一个 Git 仓库中的 `paper` 分支。论文和系统代码通过切换分支使用，日常工作只使用这一个仓库。

```text
PRE_MAP_VLN/             # 当前分支：paper
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
| 1 | 真机执行主图 | `real_world_execution.png` | `fig01_real_world/` |
| 2 | 系统总览 | `system_overview.png` | `fig02_system_overview/` |
| 3 | 第一阶段建图流程 | `stage1_mapping_pipeline.jpg` | `fig03_mapping_pipeline/` |
| 4 | 房间分割 | `room_segmentation_en.pdf` | `fig04_room_partitioning/` |
| 5 | 条件任务图 | `conditional_task_graph.pdf` | `fig05_task_graph/` |
| 6 | 候选视角生成 | `viewpoint_generation_en.pdf` | `fig06_viewpoint_generation/` |
| 7 | 联合规划与轨迹生成 | `joint_task_viewpoint_planning_en.pdf` | `fig07_joint_planning/` |
| 8 | 语义恢复 | `semantic_recovery_en.pdf` | `fig08_semantic_recovery/` |
| 9 | Habitat 执行示例 | `habitat_long_instruction_en.png` | `fig09_habitat_execution/` |

`final/` 是正文引用图件的唯一入口。源目录里的草稿、预览和新变体不会自动替换定稿。
Fig. 4 最新的单行四幅工作稿位于 `fig04_room_partitioning/single_row/`；第二层倾斜分区及 SAM 3.1 目标框叠加图位于 `fig04_room_partitioning/oblique_l2/`。

每幅图的 `SOURCE_INFO.json` 记录原始来源；[figure_manifest.json](figures/source/figure_manifest.json) 记录定稿文件、图号和迁移时的校验值。

## 绘图和实验数据

绘图源文件包含 SVG、Blender 场景、版面脚本及相关图片。SVG/Blender 可直接编辑；部分脚本重算候选、分割或轨迹时仍需原项目的算法、环境及实验数据。源脚本保留其原来的数据路径约定，迁移本身没有重写这些算法或声称所有绘图都已脱离原始数据。

系统代码保存在同一仓库的 `real-fly`、`main` 等代码分支。大模型权重、原始视频、点云及运行缓存不纳入论文分支。本机已配置本地分支切换钩子：查看论文时，将残留的代码运行文件收纳到 `/home/uav/map_VLN/PRE_MAP_VLN-local-data/code/`；切回代码分支时，自动恢复到原路径。论文编译缓存则在代码分支下收纳到该数据目录的 `paper/` 中。
该数据目录不含 Git 仓库，保存的是原有本地文件；请与仓库一起保留。
[migration_inventory.json](figures/source/migration_inventory.json) 记录文件原路径、新路径和留在上游的输入，便于追溯。

## 写作约定

- 正面、准确地描述方法、贡献和结果，避免无必要的制作备注与重复免责声明。
- 作者已确认跨房间飞行、在线识别、条件分支、局部视点重试和跨房间恢复搜索均已完成线下真机验证，不将其列为未来工作。
- 对比和消融结果按当前稿保留，补充时使用已核实的实验数据。
- 图件修改先在对应 `source/figXX_.../` 内完成，确认后再更新 `final/` 和正文图注。

## 迁移记录

2026-10-28 从 `PRE_MAP_VLN/论文/TASE_英文初稿/` 整理至此。正文内容和 9 张引用图件均保留当前版本，仅调整 LaTeX 图件路径。
2026-10-30 将论文和代码的两份本地克隆统一回 `/home/uav/map_VLN/PRE_MAP_VLN`，保留最新论文提交和新图稿，并将原未提交代码保存到 `real-fly` 的本地检查点。原迁移清单和校验记录保留当时的历史路径。

## 版本管理

日常只在 `/home/uav/map_VLN/PRE_MAP_VLN` 操作 `123awsd/MAP_VLN` 仓库：

```bash
cd /home/uav/map_VLN/PRE_MAP_VLN
git switch paper       # 论文、图件和参考资料
git switch real-fly    # 系统代码和真机工作
git switch main        # main 分支的代码
```

切换前先提交或暂存当前修改。Git 管理的文件随分支切换；本机的 `.git/hooks/post-checkout` 另外负责上述本地运行文件的自动收纳和恢复。这是本机配置，不会随提交上传。不要再为论文另建克隆或工作树。
论文和代码分支按文件或明确的提交同步，不进行整分支合并。
