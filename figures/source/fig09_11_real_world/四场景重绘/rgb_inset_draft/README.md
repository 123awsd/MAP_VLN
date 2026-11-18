# Fig. 9 三行 RGB 小窗工作稿

最新位置调整：七个名称移回各自放大框内部左上方，保留 6 pt 单行、亮黄色字和黑色文字描边。为容纳完整名称，Electric steamer 与 Bottled water 两个小窗等比例扩大到 80×60 和 66×49.5 源图 pt；其余小窗尺寸及原有照片、加粗浅蓝色轮廓、深色外沿、白色虚线均保持。已核对 32 个图片流与上一版完全一致。

最新可读性修正：目标轮廓原先仍在，但排入论文后约 0.4 pt，现沿相同蒙版加粗至约 1 pt，并增加窄深色外沿，浅蓝色仍为 #80F0FF。名称保持 6 pt 单行和现有位置，改为亮黄色 #FFF04D、加强黑色文字描边，无底板。七张照片及分割蒙版经哈希核对保持不变，仅更新图形强调层。

最新标注排版（用户指定 6 pt）：七个目标名保持完整单行，置于小窗下方且仍在对应主画面内；左下角垃圾桶小窗的名称放在小窗上方。实验室水杯名称右对齐以避开黄色无人机圆圈。白字配细黑描边，无底板。局部照片、浅蓝色轮廓、白色虚线、小窗位置和任务指令均保持不变。继续覆盖现有文件名并编译到 manuscript/root.pdf。

排版调整：小窗目标名由排入论文后约 3.6 pt 增至约 5.8 pt，Electric steamer 与 Bottled water 分为两行，以避免溢出小窗。图像、局部提亮、分割蒙版和亮蓝描边均未改变。论文中 Fig. 9 使用灵活顶/底浮动并在结论前设置浮动边界，保持在真机实验的结果说明附近。

指令字号：按论文 0.94\textwidth 的插图缩放比例，将图源中的三条指令设置为约 12.865 pt，排入论文后为 8 TeX pt，与图注一致；同步增加指令行距和图片间距。已安装到 final、更新校验值并重新编译论文和投稿源码包，PDF 中实测指令与图注字号一致。

已采用：用户于 2026-11-11 确认替换论文 Fig. 9。当前 fig9_three_rows_rgb_insets_enhanced.pdf 已安装至 figures/final/real_world_fig8_style.pdf，正文图注已同步为三行任务并说明机载视图、独立参考照片和 AI 辅助增强的来源。原四行定稿保存在上级 previous_final/。已运行 make pdf 和 make bundle，并检查编译论文第 13 页排版。

当前颜色与位置：物体轮廓描边改为高亮电光蓝 #80F0FF，线宽不变；垃圾桶小窗位于该帧左下角，双连接虚线随位置更新。目标名字与放大虚线继续使用白色。

书包描边修正：蒙版已剔除误分割的台球桌袋口黑色包边及桌面游离带状区域；局部提亮与白色描边使用同一份修正蒙版。修正范围按原参考照片裁切坐标审阅，照片本身未重绘。

最新强调方式：增强版七个小窗中的目标沿现有分割轮廓叠加纯白细描边（约 0.67 pt），仍无矩形检测框。描边是单独的透明图形层 assets/outline_*.png，不替换目标像素；由 prepare_local_brightness.py 随分割蒙版重建。

当前版本：七个目标用 imagegen 透明抠图的 alpha 生成分割蒙版，仅对目标区域的既有 RGB 做 gamma 提亮，完全丢弃这次抠图生成的 RGB。小窗背景使用整幅提亮之前的素材，蒙版外像素逐像素不变。已移除小窗内的绿色检测框，白色名称、白色虚线小窗边界与双连接线保留。分割透明图、最终蒙版、局部提亮图均保存在 assets/；`python prepare_local_brightness.py` 可重现局部调亮，参数及背景校验见 local_brightness_manifest.json。随后运行 `python build.py --enhanced` 生成工作稿。

最新亮度调整：增强版中的冰箱水杯、矿泉水、实验室水杯、篮球和垃圾桶五个小窗使用独立的 imagegen 提亮试稿，目标约 +0.5–0.7 EV 并保留高光。锅和书包保持原亮度。五个输出另存 assets/brightened_*.png，原始素材与此前增强结果保留，来源记录在 source_manifest_enhanced.json；原帧版不加载这些生成式提亮资产。当前检测框为加粗绿色实线，白色虚线外框和双连接线保留，已去掉小窗白色实线底边。

最新线型：主图目标框、放大图边框、放大图内目标框和双连接线统一改为白色虚线。白色目标名、无白底和黄色无人机圆圈保留。

最新排版：七个目标名均置于放大图内部，使用白色文字、细黑描边且无白底；放大图内增加绿色目标框，主图与小窗使用两条黑色虚线连接。仅冰箱水杯额外收紧裁切至原小窗归一化坐标 (0.08, 0.04, 0.58, 0.54)，保持实验室桌面水杯当前裁切。垃圾桶改为第一视角 69.1 秒、原帧裁切 (590, 210, 1150, 630)，可完整显示桶口、桶身和底部，不做补绘。

本次工作稿：所有小窗下方按指令目标统一标为 Pot、Cup、Bottled water、Cup、Basketball、Trash can、Backpack，删除图内 Reference photo 字样。两处第三视角水杯框扩大，并将两处水杯小窗扩大到 57×42.75 pt。来源类型仍保留在清单和建议图注中。

按用户最新要求使用内置 imagegen 对锅、水杯、矿泉水做去噪和去模糊试稿。`python build.py` 输出带目标名的原帧版；`python build.py --enhanced` 输出独立的 `_enhanced.pdf/png` 增强版。增强版的三个输入与输出均保留，提示词见 IMAGEGEN_PROMPTS.md。增强版整体物体与背景布局在目视检查中保持对应，但无法保证模糊处细节完全不变，因此不替换论文定稿。其余小窗未做生成式增强，书包仍直接使用用户原照片。

台球桌任务的目标为书包，指令已改为 “Find the backpack on the pool table in the recreation room.”。已使用桌面 `7e972e6f8d63fe03f470ce949d76030c.jpg` 的原照片裁切加入最后一帧左上角；原照片副本保存在 assets/backpack_reference_original.jpg。书包不使用图像生成或透视重建。

宿舍目标重新检查了锅 25.8–28.1 秒、水杯 37.4–39.4 秒、矿泉水 62.2–67.0 秒的相邻帧，选用 27.1 秒、37.7 秒、65.9 秒，分别收紧为 440×330、480×360、440×330 像素等比例裁切，使目标在相同尺寸的小窗内更大。原始录像仍有对焦与噪声限制，未使用生成式清晰化。

- 按本次修改意图去掉 “Find the cup on the table in the lounge.” 整行任务，保留宿舍、实验室三任务和台球桌三行。
- 六个机载 RGB 小窗和一个书包参考照片小窗使用黑色虚线框与引线；黄色无人机标记保留。
- 机载 RGB 来自与各行相同的成功飞行场次。取目标清楚的独立观测时刻，未将其表述为第三视角画面的像素放大或严格同步帧。精确源路径、视频秒数和裁切坐标见 source_manifest.json。
- 小窗均为真实 RGB 的等比例裁切，不补绘、不合成目标。构图继续使用原版第三视角照片及既有目标框坐标。
- 台球桌场次已通过 `nv@100.89.13.29` 在 NX 上定位到 `real_fly/stage2_runtime/runtime/flight_bags/lab_room_new_v1/find_ball_on_pool_table_minobs5_speed12_clear040/flight_20261019_203610/`。其中 `flight_20261019_203610_rgb.mp4` 为 0 字节，`rgb_video.log` 也是空文件，无法提帧。该目录的 `.bag.active` 未发现 `sensor_msgs/Image`、`sensor_msgs/CompressedImage` 或 `/camera/color/image_raw` 标识。检查 `/home/nv` 中其他视频及 `/mnt/pre_map_usb` 后未找到有效副本，因此台球桌行采用用户提供的参考照片小窗，不将其表述为机载录像帧。
- 此目录为工作稿，未替换 figures/final 或修改正文。运行 `python build.py` 可复现 PDF 和 PNG；裁切后的图片保存在 assets/。

建议采用此稿后将图注改为：

Representative real-world UAV executions. From top to bottom: a multi-target household inspection, a three-target laboratory inspection, and a backpack-search task at a pool table. Each row shows successive third-person views; yellow circles mark the UAV. Black dashed boxes and connectors associate selected target regions with detail views. The first two rows use cropped onboard RGB observations from the same mission, selected independently to show the targets clearly. The last row includes a separately captured reference photograph of the backpack.
