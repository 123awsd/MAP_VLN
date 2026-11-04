# Readable single-column Fig. 6

保留原定稿的场景和视点位置，全局灰点、黄框和红叉放大约三成，视锥放大 12% 并加粗；将底部图例改为两行两列，字号 8.5 pt，符号宽约 7–12 pt。整图宽 88 mm，正文图注在 LaTeX 中局部设为 9 pt。

`base_figure.pdf` 是修改前正文实际使用的原图；导出时保留三维场景，使用原投影数据放大全局符号，去掉底部旧图例及外围空白并重新绘制矢量图例，不重新计算候选视点。原图校验值、显示范围和字号记录在 `layout_evidence.json` 中。

在项目根目录运行 `python figures/source/fig06_viewpoint_generation/readable_single_column/export_figure.py`，需要 PyMuPDF、Pillow、Fontconfig 和 Liberation Sans 字体。输出 PDF、SVG 和预览；将 PDF 安装到 `figures/final/viewpoint_generation_en.pdf`，同步校验值，再运行 `make pdf bundle`。
