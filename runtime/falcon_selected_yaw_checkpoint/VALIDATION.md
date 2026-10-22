当前探索核心源码与配置已按exploration_threshold_double_half_504成功参考恢复。原速度504秒已重新跑完：1133.322m³，442.285m，三层覆盖；2号旁卫生间进出，9号右上未进入，不称历史全部轨迹完全复现。

3m/s提速组未通过：986.863m³，二楼体积下降28.8%，末段约101.6秒位置停留。默认保留原速度2m/s、3m/s²、5Hz、504秒与0.07m净空/碰撞半径，不采用提速组；房间专项优化按用户要求留后。实验结论见docs/EXPLORATION_BASELINE_SPEED_REVALIDATION.md。

局部规划归属与A*数值开发修订、提速配置和失败实验数据已按用户要求删除，只保留简要文字和指标归档：docs/experiment_archive/cleanup_20261015。最新基线卫生间分析见docs/EXPLORATION_BASELINE_BATHROOM_OMISSION.md。

7项ROS执行接口测试和90秒烟雾测试通过；504秒v2为855.101m³、1帧受阻，但两个卫生间未进入，并有固定区域覆盖退化。

成功历史参考仍为 outputs/exploration_threshold_double_half_504/source_snapshot（1139.735m³），本修订不能作为替换基线。

分析见 docs/EXPLORATION_GLOBAL_REGRESSION_REVIEW.md。
