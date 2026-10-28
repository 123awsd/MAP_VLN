# Paper workspace

This repository contains the LogicFly manuscript and paper assets. Read README.md before editing.

- Edit manuscript/root.tex and manuscript/references.bib for paper changes.
- The manuscript includes only figures/final/. Keep figure labels and captions consistent with installed assets.
- Editable figures and drafts live under figures/source/figXX_.../. Promote a new draft to final only when requested or already approved.
- Source scripts retain upstream experiment paths. Check SOURCE_INFO.json and the source README before regeneration; do not silently replace measured geometry or algorithm outputs.
- Run make pdf after manuscript or installed-figure changes. Run make bundle to refresh the submission archive.
- Build and package generation must work from this directory without the original code repository.
- Do not edit or switch the code repository's branches as part of ordinary paper work.
- Keep original recordings, checkpoints, raw point clouds, and runtime outputs outside this paper project.
