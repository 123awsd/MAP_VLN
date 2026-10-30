# Paper workspace

This is the paper branch of the shared LogicFly repository at /home/uav/map_VLN/PRE_MAP_VLN. Read README.md before editing.

- Edit manuscript/root.tex and manuscript/references.bib for paper changes.
- The manuscript includes only figures/final/. Keep figure labels and captions consistent with installed assets.
- Editable figures and drafts live under figures/source/figXX_.../. Promote a new draft to final only when requested or already approved.
- Source scripts retain upstream experiment paths. Check SOURCE_INFO.json and the source README before regeneration; do not silently replace measured geometry or algorithm outputs.
- Run make pdf after manuscript or installed-figure changes. Run make bundle to refresh the submission archive.
- Build and package generation must work from this directory while only the paper branch is checked out.
- Use this single repository for paper, real-fly, and main. Do not create another clone or worktree to separate ordinary paper and code work. Switch branches here when requested, saving pending changes first.
- Do not add original recordings, checkpoints, raw point clouds, or runtime outputs to the paper branch. Ignored local data may remain on disk across branch switches.
