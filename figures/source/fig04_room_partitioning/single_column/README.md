# Single-column room partitioning figure

正文当前采用的单栏两行两列版本，88 × 86 mm，去掉最底部的四项颜色图例。图内标题为 8.5 pt，正文图注为 9 pt。左上 (a) → 右上 (b) → 右下 (c) → 左下 (d)，三个箭头构成完整蛇形流程。

从 `../single_row/` 的 PDF 和 SVG 矢量图导出，不重算分割或点云。四幅地图均按原有完整显示范围分别提取，以相同比例放入新的版面，重新绘制大字号标题；`layout_evidence.json` 记录来源校验值、地图显示范围、字号和箭头位置。

在项目根目录运行 `python figures/source/fig04_room_partitioning/single_column/export_figure.py`，需要 PyMuPDF、Fontconfig 和 Liberation Sans 字体。生成后，将 PDF 安装到 `figures/final/`，同步校验记录，再执行 `make pdf bundle`。
