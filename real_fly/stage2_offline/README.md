# Stage 2: offline real-flight planning

This directory is for the real-flight-only offline runner and its
experiment-specific configuration.  Reuse the shared planner modules under
the repository's `stage2/` package instead of copying them here.

Recommended data layout:

```text
data/<run_id>/
├── inputs/         # map snapshot, scene graph, task graph, candidates
├── execution/      # executed/replayed trajectory and motion logs
└── reports/        # distance, time, A* count, recovery, and completion
```

The first implementation should be replay/offline-only and must not publish
flight-control commands to the live vehicle.
