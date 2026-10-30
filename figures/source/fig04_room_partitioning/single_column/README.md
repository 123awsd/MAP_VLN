# Single-column room partitioning figure

正文当前采用的单栏单行四幅版本，88 × 23.91 mm，去掉最底部的四项颜色图例。四幅地图和标题保持完整，正文图注保留颜色解释。

从 `../single_row/` 的 PDF 和 SVG 矢量图导出，不重算分割或点云。原图底部图例区移除后，将整幅图等比缩放至 88 mm；`layout_evidence.json` 记录来源校验值和裁切范围。

在项目根目录运行 `python figures/source/fig04_room_partitioning/single_column/export_figure.py`，需要 PyMuPDF。生成后，将 PDF 安装到 `figures/final/`，同步校验记录，再执行 `make pdf bundle`。
