# T-ASE 使用的 IEEE Transactions 官方 LaTeX 模板

下载日期：2026-10-22。

T-ASE 的作者指南要求 IEEE 双栏格式，并将期刊模板链接指向 IEEE Template Selector。本次 Template Selector 无法正常获取，已从 **IEEE Author Center 官方域名**直接下载 Transactions 模板包：

https://journals.ieeeauthorcenter.ieee.org/wp-content/uploads/sites/7/IEEE-Transactions-LaTeX2e-templates-and-instructions.zip

- `original_archives/`：未经修改的下载压缩包。
- `IEEEtran/`：官方 LaTeX 类、示例及说明；普通 Transactions 期刊参考 `bare_jrnl.tex`，使用 `IEEEtran` 的 `journal` 模式。
- `Transactions-Bibliography/`：官方 BibTeX 样式与示例。
- `Transactions-instructions-only.pdf`：原包附带的模板说明。
- `download_provenance.json`：官方指南、下载地址、原包 SHA-256 校验值。

这是 IEEE 通用 Transactions 官方模板，不声称下载到了 T-ASE 专用的独立类文件。T-ASE 特有的 Note to Practitioners、Index Terms 和匿名作者处理已应用到 `../TASE_英文初稿/root.tex`。
