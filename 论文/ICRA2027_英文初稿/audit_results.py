"""Check manuscript table against existing formal episodes; no simulation or writes."""
from pathlib import Path
import collections
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[3]
MATRIX = ROOT / "VLN_Benchmark/2026_0907"
CASES = set(json.loads((MATRIX / "map_vln/manifest.json").read_text())["cases"])
EXPECTED = {
    ("map_vln", "single"): (168, [180,180,180,168], [8.750,1.541,.867,8.246]),
    ("map_vln", "multi"): (128, [177,144,144,128], [24.734,1.185,.718,12.101]),
    ("mapgpt", "single"): (67, [173,173,173,67], [11.500,6.853,.271,43.040]),
    ("osgp", "single"): (74, [78,78,78,74], [8.285,1.342,.874,159.751]),
    ("osgp", "multi"): (0, [7,7,7,0], [8.559,2.725,0.,None]),
    ("spatialgpt", "single"): (57, [172,172,172,57], [8.866,8.565,.235,112.851]),
    ("spatialnav", "single"): (33, [178,178,178,33], [9.399,9.009,.174,25.881]),
    ("vln_zero", "single"): (35, [180,180,180,35], [4.684,9.055,.153,138.532]),
}
for method in ("mapgpt", "spatialgpt", "spatialnav", "vln_zero"):
    EXPECTED[method, "multi"] = (0, [0,0,0,0], [None]*4)

groups = collections.defaultdict(list)
seen = set()
for path in sorted(MATRIX.glob("*/*/*/*/env_seed*/run*/native_result.json")):
    row = json.loads(path.read_text())
    if row.get("case_id") not in CASES:
        continue
    if path.parent.name not in {"run0", "run1"}:
        continue
    if row.get("environment_seed") not in {1,2,3}:
        continue
    family = "single" if "single_target" in row["case_id"] else "multi"
    key = row["method"], family
    if key not in EXPECTED:
        continue
    identity = row["method"], row["case_id"], row["environment_seed"], row["run_id"]
    assert identity not in seen, ("duplicate formal episode", path)
    seen.add(identity)
    groups[key].append(row)

fields = ("trajectory_length_m", "navigation_error_m", "spl", "task_completion_time_s")
report = {}
for key, (successes, coverage, values) in sorted(EXPECTED.items()):
    rows = groups[key]
    assert len(rows) == 180, (key, len(rows))
    actual_successes = sum(bool(r.get("task_metrics", {}).get("task_success")) for r in rows)
    assert actual_successes == successes, (key, actual_successes, successes)
    metrics = {}
    for field, count, expected in zip(fields, coverage, values):
        selected = [r for r in rows if r["status"] == "completed"]
        if field == "task_completion_time_s":
            selected = [r for r in selected if r.get("task_metrics", {}).get("task_success")]
        samples = [r.get("task_metrics", {}).get(field) for r in selected]
        samples = [v for v in samples if isinstance(v, (int,float)) and math.isfinite(v)]
        mean = statistics.mean(samples) if samples else None
        assert len(samples) == count, (key, field, len(samples), count)
        assert (mean is None and expected is None) or abs(mean-expected) <= .000501, (key,field,mean,expected)
        metrics[field] = {"coverage": len(samples), "mean": mean}
    report["/".join(key)] = {"planned":180,"successes":successes,
        "statuses":dict(collections.Counter(r["status"] for r in rows)),"metrics":metrics}
assert len(seen) == 2160
print(json.dumps({"status":"passed","formal_episodes":len(seen),"results":report}, indent=2))
