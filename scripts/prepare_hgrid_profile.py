"""Build an isolated timing-only candidate from the verified700 dependency."""
import hashlib,json,re,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1];base=root/'runtime/consistency_validation_20261015';src=base/'A3_observation_lifecycle';dst=base/'A3_hgrid_profile';dep=root/'outputs/consistency_validation_20261015/hierarchical_grid_700_dependency.cpp';expected=json.load(open(root/'outputs/coverage_lifecycle_700_r1_20261016/complete_runtime_identity.json'))['source_sha256']['src/FALCON/falcon_planner/exploration_preprocessing/src/hierarchical_grid.cpp'];assert hashlib.sha256(dep.read_bytes()).hexdigest()==expected
if dst.exists():raise SystemExit('Fresh candidate directory required')
shutil.copytree(src,dst)
s=dep.read_text();helpers=r'''
#include "pathfinding/exploration_diagnostics.h"
#include <chrono>
namespace {
struct HgridProfileTotals { double astar_s=0,bfs_s=0; unsigned long long astar_calls=0,bfs_calls=0; };
thread_local HgridProfileTotals hg_profile;
struct HgridCostTimer {
 bool bfs; std::chrono::steady_clock::time_point begin=std::chrono::steady_clock::now();
 explicit HgridCostTimer(bool is_bfs=false):bfs(is_bfs){}
 ~HgridCostTimer(){double s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();if(bfs){hg_profile.bfs_s+=s;++hg_profile.bfs_calls;}else{hg_profile.astar_s+=s;++hg_profile.astar_calls;}}
};
struct HgridPhaseTimer {
 const char* phase;HgridProfileTotals before=hg_profile;std::chrono::steady_clock::time_point begin=std::chrono::steady_clock::now();
 explicit HgridPhaseTimer(const char*p):phase(p){}
 ~HgridPhaseTimer(){fast_planner::replayEvent("hgrid_phase",",\"phase\":\""+std::string(phase)+"\",\"elapsed_s\":"+std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count())+",\"astar_s\":"+std::to_string(hg_profile.astar_s-before.astar_s)+",\"astar_calls\":"+std::to_string(hg_profile.astar_calls-before.astar_calls)+",\"bfs_s\":"+std::to_string(hg_profile.bfs_s-before.bfs_s)+",\"bfs_calls\":"+std::to_string(hg_profile.bfs_calls-before.bfs_calls));}
};
}
'''
s=helpers+s
# Only add scope timers: same function bodies, arguments and return values.
pat=r'(double UniformGrid::getAStarCost\w*\([^{}]*?\)\s*\{)';s,n=re.subn(pat,r'\1\n  HgridCostTimer hgrid_cost_timer;',s);assert n>=8,n
for name,label in [('updateGridsFromVoxelMap','map_update'),('calculateCostMatrixSingleThread','cost_matrix')]:
 pat=r'(void UniformGrid::'+name+r'\([^{}]*?\)\s*\{)';s,n=re.subn(pat,r'\1\n  HgridPhaseTimer hgrid_phase_timer("'+label+r'");',s);assert n==1,(name,n)
start=s.index('void UniformGrid::calculateCostMatrixSingleThread(');end=s.index('void UniformGrid::calculateCostMatrixMultiThread(',start);part=s[start:end];part,n=re.subn(r'cost = connectivity_graph_->searchConnectivityGraphBFS\(cell_center_id1,\s*cell_center_id2, path\);',r'{ HgridCostTimer hgrid_bfs_timer(true); cost = connectivity_graph_->searchConnectivityGraphBFS(cell_center_id1, cell_center_id2, path); }',part);assert n==2,n;s=s[:start]+part+s[end:];(dst/'hierarchical_grid.cpp').write_text(s)
m={'base':'A3_observation_lifecycle','purpose':'Timing only; no search/scoring/threshold changes','verified_baseline_hgrid_sha256':expected,'source_sha256':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest()for p in dst.rglob('*')if p.is_file()and '__pycache__'not in p.parts},'rollback_requirement':'Restore exact baseline HGrid dependency explicitly before launching old checkpoint: base snapshot does not contain this dependency.'};(base/'A3_hgrid_profile_manifest.json').write_text(json.dumps(m,indent=2));print(json.dumps({'candidate':str(dst),'instrumented_hgrid_functions':n,'baseline_hash_verified':True,'note':'Not built or run; rollback dependency must be wired before any execution.'}))
