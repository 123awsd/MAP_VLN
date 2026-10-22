# LogicFly：T-ASE 英文工作稿

本目录是当前面向 **IEEE Transactions on Automation Science and Engineering (T-ASE)** 的英文稿，迁移日期为 2026-10-22。

## 打开与编译

- `root.tex`：主源文件。
- `root.pdf`：编译完成的 PDF。
- `references.bib`：从 ICRA 英文稿原样复制的参考文献。
- `figures/`：本稿使用的全部六张图片，内容与原始素材一致，使用英文文件名以方便上传。
- `IEEEtran.cls`、`IEEEtran.bst`：官方模板包中的原版文件，未修改。
- `LogicFly_TASE_source.zip`：独立编译用的源文件包，包含上述源文件、图片及编译说明，不包含中间文件。

```bash
cd /home/byq/Main_vln/MAP_VLN/论文/TASE_英文初稿
make
```

需要 XeLaTeX、BibTeX、latexmk 和 TeX Gyre 字体。`make` 会自动运行所需的文献及交叉引用编译步骤，将结果复制到 `root.pdf`；中间文件保存在 `build/`。也可以运行 `latexmk -xelatex -outdir=build root.tex`，此时 PDF 位于 `build/root.pdf`。上传到 Overleaf 时选择 `root.tex` 和 XeLaTeX 引擎。

## 迁移内容

源稿为 `../ICRA2027_英文初稿/root.tex`，不是 `paper/latex/main_en.tex` 中较早的 PRE-MAP-VLN 草稿。ICRA 版本保留原样。

- 采用官方 IEEEtran 的 `letterpaper,10pt,journal` 双栏期刊模式，移除 ieeeconf 的页边距覆盖、会议标题格式和首页插图专用命令。
- 保留论文标题、摘要、方法、实验结果、六张图、两张表和参考文献内容。摘要为 189 词。
- 在摘要后新增 169 词的 Note to Practitioners，说明应用场景、部署条件和现有验证的局限；随后新增 Index Terms。
- 使用匿名作者栏，启用期刊页眉和页码，不填写出版卷期、录用日期、DOI 或版权编号。
- 首页真机图改为标准双栏浮动图；仿真图改为标准浮动位置；消融表字体调整为 8 pt。
- 图片和样式文件均在本目录内，不依赖其他目录的软链接。

## 官方依据

- [T-ASE 作者指南](https://www.ieee-ras.org/publications/t-ase/information-for-authors/)：IEEE 双栏稿；Regular Paper 摘要不超过 200 词；摘要后添加 100–300 词的 Note to Practitioners 和关键词。
- [投稿检查表](https://www.ieee-ras.org/publications/t-ase/information-for-authors/author-checklist-for-papers-submitted-to-ieee-t-ase/)：双匿名评审；强调自动化问题、效率与可靠性及与既有方法的比较。
- [Note to Practitioners 指南](https://www.ieee-ras.org/publications/t-ase/information-for-authors/t-ase-note-to-practitioners/)：面向工程实践，区别于摘要，交代实际应用与局限。
- 官方模板压缩包与下载地址、校验值见 `../TASE_template/`。

## 当前状态

这是完成模板迁移的工作稿，尚未投稿。当前 PDF 为 9 页，含参考文献。新增的 Note to Practitioners 需要作者审阅；面向 T-ASE 的自动化贡献表述、近期文献覆盖、实验支撑、图中文字可读性及全部投稿材料仍应在定稿时审查。没有新增实验结果或改写既有实验结论。

原稿的实现及实验溯源材料仍保留在 `../ICRA2027_英文初稿/evidence_notes.md` 和 `audit_results.py`；这些历史材料不作为本次迁移中重新验证实验结论的证据。作者身份信息应按双匿名要求在初投稿文件及补充材料中处理，正式作者资料在投稿系统的相应字段中填写。
