# Selected exploration checkpoint

Actual sources from the720second yaw-consistency trial; source hashes in manifest.json. Install same-basename cpp files into matching FALCON package src directories and frontier_finder.h into exploration_preprocessing/include/exploration_preprocessing, then catkin_make. Use supplied config and launch scripts in a fresh trial directory, replacing recorded output paths consistently.

Actual RGB-D acquisition executes planning yaw by default. The round6 frontier algorithm and crossfloor cost18 are preserved. Execution-blocked targets are avoided for20seconds. No fixed room route or ground-truth map enters planning. Old heading behavior remains available with --legacy-motion-heading.

Result:1203.29m³ known volume,548.28m path,4 blocked frames,timeout,3597 aligned depth/RGB frames. Median yaw residual0.105°,P95 7.958° with ROS timestamp alignment; residuals include receipt delay. No complete-coverage or full dynamic-flight claim.

Presentation keeps first28seconds of the40second accelerated video. Full720second records and metrics are retained. Late movement beyond visible building structure requires separate containment analysis; staying in the configured map box does not prove staying inside the building.
