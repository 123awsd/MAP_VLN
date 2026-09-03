# Stage-1 runtime area

This directory is for generated FAST-LIO runtime files only. The mapper
creates its `Log/` and `PCD/` files here, and each real-data run should use a
separate runtime root under `data/<run_id>/mapping/` so an earlier run cannot
be overwritten accidentally.

Runtime output is ignored by Git. Do not place credentials, flight-control
configuration, or raw sensor data in the tracked repository.
