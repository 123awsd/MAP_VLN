# ICRA 2027 English working draft

Title: **Joint Task Ordering and Viewpoint Selection for Language-Guided Indoor Aerial Navigation**

The draft uses the repository's unmodified IEEE conference class, US Letter paper, 10-point text, and two columns. The compiled PDF has **8 pages including references**. Images are existing repository assets used as explicitly identified draft placeholders.

## Build

```bash
cd /home/byq/Main_vln/MAP_VLN/论文/ICRA2027_英文初稿
xelatex -interaction=nonstopmode -halt-on-error root.tex
bibtex root
xelatex -interaction=nonstopmode -halt-on-error root.tex
xelatex -interaction=nonstopmode -halt-on-error root.tex
pdfinfo root.pdf
```

Alternatively, run `latexmk -xelatex root.tex`. XeLaTeX embeds Unicode OpenType TeX Gyre fonts for PDF-viewer compatibility. The source includes an IDE engine directive, and `.latexmkrc` selects XeLaTeX. No Chinese font packages are required. `paper_english.pdf` is a byte-identical alternate filename for opening a fresh preview when `root.pdf` is cached.

## Files

- `root.tex`: editable English manuscript.
- `root.pdf`: compiled eight-page draft.
- `references.bib`: bibliography, including S-Nav, AirHunt, scene-graph TAMP, and long-horizon VLN.
- `audit_results.py`: read-only, manifest-filtered check of the 2,160 formal episode records against the main table.
- `evidence_notes.md`: implementation and experiment provenance, qualifications, and remaining work.
- `tmp/english_qa/`: page renders used for visual checking.

## Scope

CPTG is the task representation; JTVP is the finite task-window and observation-pose solver. Methods describes the currently audited rolling runner, including the three-task default, representative selection, actual 3D-length time proxy, and conditional observation feedback. Historical result identifiers remain `map_vln` and are not renamed in the experimental data.

The main benchmark executes precompiled graphs. It does not test online language parsing. The historical 270-trial study is explicitly marked provisional because raw trials are absent and the planner was single-floor/2.5D. Native interface differences and metric coverage are retained in the paper.

The Chinese manuscript was replaced in the paper directory. A recoverable snapshot remains at `/home/byq/Main_vln/tmp/paper_revision_backup/ICRA2027_中文初稿/`.

The [ICRA 2027 technical-paper call](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/) states an eight-page submission limit including references (checked September 9, 2026). This is a working draft, not a submission-ready package: authors, affiliations, frozen-run hardware metadata, final figures, and current-3D matched ablations still need completion.
