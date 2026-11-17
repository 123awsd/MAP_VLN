# System Overview selected assets

These are copied, paper-facing image assets. Original outputs remain in
`outputs/`; this directory intentionally contains no logs, bags, JSON files,
or processing scripts.

## Existing assets

| Pipeline block | Asset | Intended use |
|---|---|---|
| 3D exploration | `stage1/00337_truth_vs_falcon_3d.png` | Map construction evidence |
| 3D exploration | `stage1/00337_truth_vs_falcon_floor_projections.png` | Floor-wise coverage evidence |
| Cross-floor geometry | `stage1/00337_transition_diagnostic.png` | Stair/transition-space module |
| Room segmentation | `room_pipeline/00337_transition_room_summary.png` | Geometry-to-room output |
| Room semantics | `room_pipeline/00337_room_semantics.png` | Qwen room labels |
| Room semantics | `room_pipeline/00337_L1_room_semantics.png` | Single-floor detail |
| Room semantics | `room_pipeline/00337_L2_room_semantics.png` | Single-floor detail |
| Room semantics | `room_pipeline/00337_L3_room_semantics.png` | Single-floor detail |
| Task execution | `stage2/00337_stage2_task_execution_summary.png` | 3D trajectory and task results |
| Online perception | `stage2/00337_terminal_microwave.jpg` | RGB observation evidence |
| Online perception | `stage2/00337_terminal_coffee_table.jpg` | RGB observation evidence |
| Online perception | `stage2/00337_terminal_tv.jpg` | RGB observation evidence |

## Generated panels

The reusable generator is
`paper/system_overview/scripts/generate_system_overview_assets.py`. Its
default inputs are the current 00337 records and its outputs are written to
`selected/generated/`:

| Panel | Output |
|---|---|
| Task graph | `任务图_语言约束与条件分支.png/.pdf` |
| Candidate views | `候选观察位姿_三维视锥.png/.pdf` |
| Semantic recovery | `目标丢失_语义恢复搜索流程.png/.pdf` |
| 3D trajectory | `跨楼层执行轨迹_1至3层.png/.pdf` |
| Five-stage overview | `系统总览_五阶段流程.png/.pdf` |
| Map representation stack | `地图表示层_三维占据到房间语义.png/.pdf` |

## Completion status

The task-graph, candidate-viewpoint, joint-route, RGB-verification, semantic-
recovery, and assembled overview panels have now been generated. See the
Chinese master inventory at `paper/system_overview/素材清单.md` for the current
recommended files and their traceable source outputs. This selected directory
remains paper-facing; historical visual experiments stay under `candidates/`.

## Reviewed clean real-data library

Paper-facing source panels are under `clean_real_data/`. They contain floors
1--3 only, use pure-white backgrounds, omit axes and grid furniture, and are
generated from real occupancy, room, box, viewpoint, and execution records.
See `clean_real_data/README.md` for the intended role of each panel. Visual
experiments that did not pass review remain isolated under
`assets/candidates/clean_real_data/`.
