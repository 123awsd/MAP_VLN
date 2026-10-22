"""Same-call immutable buffers only: no path cache or map-version reuse."""
from pathlib import Path
import hashlib,json,shutil
root=Path(__file__).resolve().parents[1];base=root/'runtime/consistency_validation_20261015';dst=base/'A3_cost_buffers';src=base/'A3_observation_lifecycle';baseline=(base/'shared_dependencies/hierarchical_grid.cpp').read_text();assert hashlib.sha256(baseline.encode()).hexdigest()=='2fca25380ba13bd1551ca5b767b8f079ec5890a65cc82a29adcf524765b08896'
if dst.exists():raise SystemExit('fresh candidate required')
shutil.copytree(src,dst)
begin=baseline.index('void UniformGrid::calculateCostMatrixSingleThread(');end=baseline.index('void UniformGrid::calculateCostMatrixMultiThread(',begin);part=baseline[begin:end]
anchor='  // Between all zone centers\n';insert='''  // Build each cell's immutable centre sequence once per matrix invocation.
  // Never reused across map updates; preserve FREE then UNKNOWN ordering.
  std::vector<std::vector<Position>> matrix_centers(uniform_grid_.size());
  for (const GridCell &cell : uniform_grid_) {
    auto &centers = matrix_centers.at(cell.id_);
    centers.reserve(cell.centers_free_active_.size()+cell.centers_unknown_active_.size());
    centers.insert(centers.end(),cell.centers_free_active_.begin(),cell.centers_free_active_.end());
    centers.insert(centers.end(),cell.centers_unknown_active_.begin(),cell.centers_unknown_active_.end());
  }
''';assert part.count(anchor)==1;part=part.replace(anchor,insert+anchor)
a=part.index('    vector<Position> centers1;');b=part.index('    // Iterate over all centers in the grid_cell1',a);part=part[:a]+'    const auto &centers1 = matrix_centers.at(grid_cell1.id_);\n\n'+part[b:]
a=part.index('        vector<Position> centers2;');b=part.index('        // Iterate over all centers in the grid_cell2',a);part=part[:a]+'        const auto &centers2 = matrix_centers.at(grid_cell2.id_);\n\n'+part[b:]
(dst/'hierarchical_grid.cpp').write_text(baseline[:begin]+part+baseline[end:]);manifest={'base':'A3_observation_lifecycle','purpose':'Reuse immutable centre buffers only within one HGrid matrix call; no changes to searches, budgets, ordering or formulas','source_sha256':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest()for p in dst.rglob('*')if p.is_file()and '__pycache__'not in p.parts},'verified_baseline_hgrid_sha256':hashlib.sha256(baseline.encode()).hexdigest()};(base/'A3_cost_buffers_manifest.json').write_text(json.dumps(manifest,indent=2));print(dst)
