# 2026-10-12 round5探索算法恢复

用户指定恢复昨天/前天效果较好的探索。来源为outputs/exploration_recovery_local_trial/manager_stable.cpp、astar_contacts.cpp、planner_contacts.cpp和outputs/exploration_room_entry_trial/frontier_before.cpp。10月13日视频素材所用round6_extended沿用这套算法。

不是全仓库git reset。仅安装manager、frontier、astar三份探索源码；planner guard与当前哈希一致。当前checkpoint及新Boxer、墙体、房间工作保留。保留6.25m高度限制作为独立安全检查；新运行需同步RGB-D、planner yaw及碰撞检查。

历史round5到三层但timeout；不承诺全三层覆盖或屋顶安全。manifest.json记录哈希。先编译、复跑验证后才评价恢复效果。
