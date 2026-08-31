# Outputs

本目录存放实验输出，默认不提交 Git：

- `stage1_3d/<scene>/<run>/`：三维探索日志、episode、配置和 Bag 导出；
- `bags/`：Stage1 与 RViz 回放 Bag，通常是最大的文件；
- `boxer/`：2D/3D 开放词表检测及融合物体框；
- `room_pipeline/`、`wall_extraction/`、`occusg/`：多楼层房间、墙和 OccuSG 中间结果；
- `stage2_3d/prepared/<name>/`：可复制到另一台 clone 的最小 Stage2 输入；
- `stage2_3d/tasks/<run>/`：自然语言任务图和在线执行结果。

大型 Bag 和完整 episode 不应提交 Git。需要换机快速运行第二阶段时，只打包 `stage2_3d/prepared/<name>/`；具体命令见 `docs/新机器完整复现.md`。
