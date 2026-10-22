# 305实体核查：实际RGB与官方局部纹理表面

本次只读现有700秒RGB缓存、采集位姿与原始官方HM3D GLB。未改分割、控制、占据或旧版结果，未向在线算法输入GT。

## 已确认

实测685.9–693.7秒RGB结合保存位姿位于305主体附近，能够确认带斜顶的卫生间布局：淋浴区、洗手台、镜子及木质隔断。颜色点云中规整的斜面并不等于额外房间。官方局部纹理视图亦呈现卫生间及邻接木结构空间。

## 未确认

不能仅凭这些局部透视图，把305右侧凸出0.6675m²全称为屋檐、室外、墙厚或镜面伪影。朝凸出方向的视图有近处表面遮挡；诊断相机本身未做安全可达证明。现有实际RGB没有完成每个凸出体素的深度像素溯源。不得据此强制裁剪。

此前773个FREE体素三维连接是最终地图内部事实，不是官方几何或现实房间连通的证明。

## 渲染方式与限制

Habitat EGL离线新视角启动失败（无法找到CUDA设备，nvidia-smi显示驱动不可用）。改用Magnum CPU解码官方GLB几何和Basis纹理，按面积采样并点式深度投影，固定种子305。官方视图不是实际采集，不含14个实验修复物体；点式渲染有采样孔洞，不能把灰白孔洞当实体开口。只裁剪局部几何，非完整光线追踪。实际RGB另外单列，并保存最近采集时刻、位姿和时间误差。

outputs/coverage700_wall_room_preview/room305_official/：actual_rgb_selected.png及actual_rgb_manifest.json；official_cpu_views.png及cpu_manifest.json；official_colored_local.npz。

脚本inspect_room305_cpu_mesh.py、render_room305_actual_rgb.py；未成功的GPU尝试inspect_room305_official.py保留为故障复核。原分割继续使用wall_structural_rooms。当前实体边界尚未锁定，不把离线查看包装成已实现的根本修复。
