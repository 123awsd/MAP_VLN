"""Read-only parse existing intact stage measurements; no double counted totals."""
import argparse,json,re,collections
from pathlib import Path
import numpy as np
parser=argparse.ArgumentParser();parser.add_argument('case',nargs='?',default='coverage_lifecycle_700_r1_20261016');parser.add_argument('--output',default='coverage700_stage_times.json');args=parser.parse_args()
root=Path(__file__).resolve().parents[1];case=root/'outputs'/args.case;text=re.sub(r'\x1b\[[0-9;]*m','',(case/'run.log').read_text(errors='replace'));patterns={
 'frontier_search':r'Frontier search t: ([\d.]+) ms, viewpoint t:\s+[\d.]+ ms, frontier update total t: [\d.]+ ms',
 'frontier_viewpoints':r'Frontier search t: [\d.]+ ms, viewpoint t:\s+([\d.]+) ms, frontier update total t: [\d.]+ ms',
 'frontier_update_total':r'Frontier search t: [\d.]+ ms, viewpoint t:\s+[\d.]+ ms, frontier update total t: ([\d.]+) ms',
 'hgrid_update_map':r'Hgrid update map time \(total, space decomp, conn graph\): ([\d.]+), [\d.]+, [\d.]+ ms',
 'hgrid_connection_graph':r'Hgrid update map time \(total, space decomp, conn graph\): [\d.]+, [\d.]+, ([\d.]+) ms',
 'hgrid_input_frontiers':r'Hgrid input frontiers time: ([\d.]+) ms',
 'hgrid_update_frontier_info':r'Hgrid update frontier info time: ([\d.]+) ms',
 'hgrid_cost_matrix':r'Hgrid cost matrix 2 time: ([\d.]+) ms, TSP 2 time: [\d.]+ ms',
 'hgrid_tsp':r'Hgrid cost matrix 2 time: [\d.]+ ms, TSP 2 time: ([\d.]+) ms',
 'sop_cost_matrix':r'SOP cost matrix time: ([\d.]+) ms',
 'sop_solve':r'SOP time: ([\d.]+) ms'}
stats={}
for k,pat in patterns.items():
 values=[float(m[1])for m in re.finditer(pat,text)];stats[k]={'complete_logged_samples':len(values),'sum_s':sum(values)/1000,'median_ms':float(np.median(values))if values else None,'p95_ms':float(np.percentile(values,95))if values else None,'max_ms':max(values,default=None)}
x=json.load(open(case/'search_round_full_audit.json'));rounds=[r for r in x['all_rounds']if 'compute_s'in r];result={'case':case.name,'manager_rounds':len(rounds),'manager_call_total_s':sum(r['compute_s']for r in rounds),'pre_local_candidate_total_s':sum(r.get('pre_candidate_s')or 0 for r in rounds),'timing_stats':stats,'limits':['Log interleaving may lose intact measurements; sample totals are observed lower sums, not complete profiling.','hgrid_connection_graph is included in hgrid_update_map: never add both.','Frontier structure updates before manager invocation are outside manager call total; frontier_search and frontier_viewpoints are included in frontier_update_total.','No final-scoring phase timing exists: cannot assign all residual time to gain evaluation.']};out=root/'outputs/consistency_validation_20261015'/args.output;out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
