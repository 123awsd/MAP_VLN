# 墙提取结构证据试验

基线三层逐点重现通过。独立修改允许具有至少1.8m高度跨度、1m水平延伸、低位和高位实际测量的平面组件替代部分顶面支持；逐列仍要求实际低高测量及原连续性条件。保留原平面拟合、密度检查；不填造墙点，不使用GT或坐标专用规则。这些阈值是试验先验，尚非最终方法。

原/新墙点：一楼108339→117777（新增9443、减少5）；二楼163793→181081（新增17288）；三楼80253→80378（新增125）。这些是点数变化，不是精度改善。

重点墙窗口仍为202/6229，没有改善。原算法拒绝曝光记录中该窗口768点曾因组件顶部种子不足被拒，0点进入逐列顶部拒绝；记录跨提案重叠，不能作互斥终因统计，其余点仍需检查提案/拟合/组件过滤。因此未证明局部顶部门槛是重点墙的主要原因，前述假设不应当作结论。

新增墙叠加实测点云：wall_structural_evidence/three_floors_walls_on_cloud.png，红原墙、绿新增、橙未保留竖直候选。家具与开口无语义真值核验，未晋升、不接入房间分割，不覆盖原版。

脚本extract_walls_structural_evidence.py、run_wall_structural_trial.py、render_wall_structural_trial.py；机器产物outputs/coverage700_wall_room_preview/wall_structural_evidence/audit.json及逐层NPZ。无新探索、未commit。
