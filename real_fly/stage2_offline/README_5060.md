# RTX 5060 上运行 Boxer

NX 上的 CPU 推理用于验证链路；完整语义推理建议放到 RTX 5060 主机。只需要
传输导出的 ScanNet 子集和模型，不需要传输原始 4.5 GB bag。

在 NX 上打包（不会修改输入）：

```bash
cd /home/nv/SL_WS/PRE_MAP_VLN_real_fly
tar -C real_fly/stage2_offline/data/Drone_room -czf /tmp/scene9002_00.tgz scene9002_00
tar -C real_fly/stage2_offline/third_party/boxer -czf /tmp/boxer-ckpts.tgz ckpts
```

将两个压缩包和项目的 `real_fly/stage2_offline/` 脚本/配置复制到 5060 主机，
在已安装 CUDA 的环境中执行：

```bash
cd /path/to/project/real_fly/stage2_offline/third_party/boxer
python3 run_boxer.py \
  --input ../../data/Drone_room/scene9002_00 \
  --skip_n 3 --max_n 999 \
  --labels="chair,table,desk,cabinet,door,sofa,monitor,backpack,box" \
  --thresh2d 0.18 --thresh3d 0.20 --skip_viz --fuse \
  --output_dir ../../data/Drone_room/boxer_final_5060
```

预计 49 帧约为几分钟级，具体取决于显卡驱动、CUDA、PyTorch 和输入尺寸。完成
后将 `boxer_final_5060/scene9002_00/boxer_3dbbs_fused.csv` 复制回 NX，使用
现有 `build_boxer_scene_and_task.py`、`plan_stage2_mission.py` 和
`validate_drone_room_stage2.py` 重新生成最终场景图与规划。不要把 bag、认证文件
或飞控配置上传到 5060 主机。
