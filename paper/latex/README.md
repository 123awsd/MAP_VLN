# LaTeX paper workspace

- `main_zh.tex`: Chinese rough draft.
- `main_en.tex`: English rough draft.
- `preamble.tex`: shared packages, macros, and figure paths.
- `references.bib`: verified BibTeX records only.
- `build/`: generated PDFs and intermediate files; do not place source assets here.

Build both versions:

```bash
cd paper/latex
make
```

The finished PDFs are copied to `paper/main_zh.pdf` and
`paper/main_en.pdf`. LaTeX intermediate files stay under `build/`.

Build one version:

```bash
make zh
make en
```

The System Overview figure is loaded from
`paper/system_overview/assets/final/system_overview.pdf`. Until that file exists,
the paper compiles with a placeholder box.
