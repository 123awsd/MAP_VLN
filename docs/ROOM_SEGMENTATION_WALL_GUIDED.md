# 当前房间分割：墙约束分水岭与几何合并

本版本保存2026-10-15当前三层房间分割流程，结果区域数为6/11/12。是项目自实现几何流程，不是论文完整复现；区域数不等于已核验真实房间数。

## 输入与范围

采用已接受实际yaw探索的截止503.9297003272秒观测，排除随后飞出建筑的部分。每层输入为：5cm三值占据图，以及用户认可的自动门板分离、多高度Hough补充、孤立墙碎片清理后的墙点。门楣实验不参与。没有读取人工房间编号、轮廓或场景语义真值。

输入文件路径与SHA256见runtime/room_segmentation_wall_guided/input_manifest.json。实验输入、点云和渲染缓存位于本地outputs目录，未把大型原始数据加入提交；换机器复现须取得对应文件。代码提交包含当前三层结果npz及预览，可直接查看/渲染。

## 自动流程

1. 墙点投影到5cm栅格；3×3闭运算连接小采样断点。
2. 已观测格（自由或占据）扣除墙，构成结构分区域；未知不填充。家具占据格也可以纳入房间范围，因此输出不能作为无人机通行地图。
3. 距离变换计算到域边界的净距。净距至少0.45m区域作为核心，核心最小面积0.12m²；没有核心的独立已观测域面积至少0.5m²时，用内部最大净距点补种子。
4. 标记控制分水岭沿负距离场扩展，在墙/未知边界停止。小房间核门槛从旧1m²降低，减少漏分。
5. 对区域间界面作几何合并：长度至少1.2m且长度/80分位净距至少6，或长度0.85–1.1m、净距不超过0.22m且比值至少4.5。规则对所有楼层相同，不含楼梯或房间编号例外。
6. 连续重编号并导出分割npz、区域及合并证据、三层点云叠加图。填色强度60%，原实测点云覆盖其上，未知不着色。

这里并不识别“楼梯间”语义；楼梯分裂经几何界面合并修正。合并阈值根据这次地图试验调整，其它场景的可靠性尚未验证。仍可能有小房间漏分、走廊串房间及误合并。用户反馈用于检查/调试，不作为运行时编号输入；没有房间GT，未提供IoU或准确率。

## 复现

仓库根目录运行，Python环境需NumPy、SciPy、OpenCV、scikit-image、Pillow。当前脚本使用/tmp/rose2_deps隔离依赖路径，换机器需安装相应依赖或调整路径。

```bash
OPENBLAS_NUM_THREADS=2 /home/uav/miniconda3/envs/vlm_nav/bin/python runtime/room_segmentation_wall_guided/segment.py
/home/uav/miniconda3/envs/vlm_nav/bin/python runtime/room_segmentation_wall_guided/refine_merge.py
/home/uav/miniconda3/envs/vlm_nav/bin/python runtime/room_segmentation_wall_guided/render.py adaptive_merge
```

第一步构建初始分割与墙栅格，第二步恢复小核并自动合并，第三步渲染。只渲染已提交结果可以跳过前两步，但仍需三层原点云outputs/room_apf_latest_trial/L*_input.npy。输出写入outputs/room_wall_guided_trial；原探索及墙输入不修改。
