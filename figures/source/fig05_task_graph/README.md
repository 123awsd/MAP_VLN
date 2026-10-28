# CPTG 任务解析与任务图 — 完整可复现包

此包以已确认的 **v15** 完整图为准，保留原有上、中、下三层排版、T1–T5 任务卡、T5 双 Anchor、底部任务关系和中间连接箭头。没有改变图的任何视觉内容。

## 打开现成的图（无需安装环境）

- `CPTG_TaskGraph.svg`：**完整、自包含**的 SVG。7 张图标/场景素材都已通过 Base64 嵌入，可以直接复制到其他电脑，用浏览器、Inkscape 或 Adobe Illustrator 打开，不依赖额外的素材路径。
- `CPTG_TaskGraph.png`：同一版本的 PNG 预览，1640 × 772 像素。

## 完整重新生成（不需要联网获取图片）

将整个文件夹复制到其他位置，进入该目录运行：

```bash
python -m pip install -r requirements.txt
python generate.py
python verify.py
```

上述命令输出 `build/CPTG_TaskGraph.svg` 与 `build/CPTG_TaskGraph.png`。

如果不想安装任何第三方包，只用 Python 标准库即可重建 **完整 SVG**：

```bash
python generate.py --no-png
```

若要指定输出文件夹：

```bash
python generate.py --output-dir /path/to/output
```

## 目录结构

```
CPTG_TaskGraph_Full_Reproducible/
├── CPTG_TaskGraph.svg           # 最终版 SVG（所有素材内嵌）
├── CPTG_TaskGraph.png           # 最终版 PNG
├── generate.py                  # 真实生成脚本，自动内嵌图片并渲染 PNG
├── verify.py                    # 校验最终文件的 SHA-256
├── requirements.txt             # 固定版本的 PNG 渲染环境依赖
├── assets/                      # 所有 7 张原始 PNG 素材
│   ├── parser_icon.png
│   ├── t1_kettle_stove.png
│   ├── t2_water_bottle_desk.png
│   ├── t3_tissue_sink.png
│   ├── t4_backpack_coffee_table.png
│   ├── t5_book_living_room.png
│   └── t5_book_bedroom.png
├── source/
│   ├── figure_template.svg      # 结构、位置、文字、颜色、箭头的 SVG 源模板
│   └── checksums.json           # 参考图的 SHA-256
├── build/                        # 生成结果
└── reference/
    └── arrow_detail.png         # 最上方连接箭头的局部参考
```

## 修改内容

- 编辑 `source/figure_template.svg` 可以改卡片尺寸、文本、箭头路径、边框和排版；保留 `{{BASE64:filename.png}}` 形式的占位符即可。
- 替换 `assets/` 中对应 PNG 可以更换场景图像和标题图标，不必改 SVG 代码。
- 运行 `python generate.py` 重新生成自包含 SVG。
- 注意：`CPTG_TaskGraph.svg` 是导出的最终文件，修改源模板比手工改导出文件更适合后续反复生成。

## 可复现性验证

本包用 SHA-256 对最终 SVG 和 PNG 校验：

- SVG 的文件字节在任何系统上都能 **完全一致**（前提是素材和模板未改）。
- PNG 在同一字体 / 渲染器版本环境下可逐字节一致；不同系统的字形栅格化、字体替代或 CairoSVG/Cairo 版本可能造成少量像素差异。已附带最终 PNG 保证视觉参考不丢失。
- SVG 自包含，移到其他目录、断网、单独发送都能正常显示图片。

推荐 Python 3.10+。Linux 上 CairoSVG 可能需要系统 Cairo 支持，Windows/macOS 根据包安装提示处理即可。
