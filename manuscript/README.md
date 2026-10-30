# LogicFly — T-ASE manuscript

`root.tex` is the manuscript source; `root.pdf` is the compiled paper.
The comparison and ablation sections remain open for experimental results.

Build with `make pdf` (XeLaTeX and BibTeX), or select XeLaTeX and `root.tex` in Overleaf.
`../figures/final/` contains the figures cited by the manuscript. Editable sources
are organized by figure number under `../figures/source/`.

This is the paper branch of the shared repository at `/home/uav/map_VLN/PRE_MAP_VLN/`.
Run `make pdf` here or from the project root. See `../README.md` for the figure
index, source provenance, and submission packaging commands.

## 论文编辑约定

- 正面、准确地阐述方法、贡献与实验结果；避免无必要的自我否定、反复免责声明和代码解读式表述。
- 不因本地缺少记录或旧文档描述，就推断某项工作尚未完成。作者已确认跨房间飞行、在线识别、条件分支、局部视点重试和跨房间恢复搜索均已完成线下真机验证，不再将其列为未来工作。
- 必要的方法边界简洁说明；
- 未经明确要求，不改定稿图件；对比与消融结果暂留空。
