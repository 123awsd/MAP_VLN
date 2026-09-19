"""Synthetic 10 Hz geometry benchmark, no ROS/network/control."""
import json
import sys
import time
import resource
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from live_clearance_core import assess, cloud_xyz

count = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30
rng = np.random.RandomState(42)
points = rng.uniform(-10, 10, (count, 3)).astype("<f4")
msg = NS(width=count, height=1, row_step=count*12, point_step=12,
         data=points.tobytes(), is_bigendian=False,
         fields=[NS(name=n, offset=i*4, count=1, datatype=7) for i, n in enumerate("xyz")])
times = []
start, cpu = time.monotonic(), time.process_time()
while time.monotonic() - start < duration:
    t = time.perf_counter()
    assess(cloud_xyz(msg), [0, 0, 0], [0.5, 0, 0])
    times.append((time.perf_counter() - t)*1000)
    time.sleep(max(0, 0.1 - (time.perf_counter()-t)))
print(json.dumps(dict(synthetic_only=True, points=count, iterations=len(times),
    p95_ms=float(np.percentile(times, 95)), max_ms=max(times),
    cpu_one_core_percent=100*(time.process_time()-cpu)/(time.monotonic()-start),
    peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)))
