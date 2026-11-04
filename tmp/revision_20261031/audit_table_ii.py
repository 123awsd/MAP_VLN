"""Read-only recomputation of Table II; run with the workspace root as argument."""
import json
import statistics
import sys
from pathlib import Path

workspace = Path(sys.argv[1]).resolve()
report_path = workspace / 'VLN_Baselines/消融/outputs/benchmark_cptg_jtvp_full_20261009/report.json'
records = json.loads(report_path.read_text())['records']
key = lambda r: (r['case_id'], r['environment_seed'], r['run_id'])
full = {key(r): r for r in records if r['method'] == 'full_cptg_jtvp'}
result = []
for method in ('no_cptg_full', 'cptg_fixed_views', 'full_cptg_jtvp'):
    rows = [r for r in records if r['method'] == method and r['task_family'] == 'multi_target']
    assert len(rows) == 546 and len({key(r) for r in rows}) == 546
    spl, times, old_times = [], [], []
    for row in rows:
        metrics = row['task_metrics']
        if row['status'] != 'completed':
            # Verify these are failures with a defined geometric reference,
            # rather than negative-existence episodes for which SPL is N/A.
            assert row['status'] == 'native_timeout'
            assert not metrics['task_success']
            assert full[key(row)]['task_metrics']['spl'] is not None
            spl.append(0.0)
        elif metrics.get('spl') is not None:
            spl.append(metrics['spl'])
        if metrics['task_success']:
            old_times.append(metrics['task_completion_time_s'])
            if method == 'full_cptg_jtvp':
                original = json.loads(Path(row['source']['original_native_result']).read_text())
                assert key(original) == key(row)
                assert original['task_metrics']['task_success']
                times.append(original['task_metrics']['task_completion_time_s'])
            else:
                times.append(metrics['task_completion_time_s'])
    result.append(dict(method=method, episodes=len(rows), successes=len(times),
                       sr_percent=100*len(times)/len(rows), spl_n=len(spl),
                       spl_percent=100*statistics.fmean(spl),
                       outer_run_ct_s=statistics.fmean(times),
                       previous_mixed_ct_s=statistics.fmean(old_times)))
print(json.dumps(result, indent=2))
