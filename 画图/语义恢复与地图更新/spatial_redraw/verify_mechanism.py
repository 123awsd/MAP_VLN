"""Check the depicted rules using existing production code and synthetic data."""
from pathlib import Path
import json
import math
import sys

OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent))
import mechanism_audit
from stage2.semantic_region_search import SemanticRegionBelief

mechanism_audit.OUT=OUT
report=mechanism_audit.audit()

# The inset shows 6 x 4 schematic samples, first two columns covered and then
# two additional columns covered. Run that exact set update through the code.
prior=list(range(8))
next_view=list(range(16))
base={'semantic_region_id':'figure_region', 'semantic_prior_score':.8,
      'base_geometry_cost':.1, 'region_sample_count':24}
a={**base,'id':'prior_view','visible_region_sample_ids':prior,'view_quality':8/24}
b={**base,'id':'next_view','visible_region_sample_ids':next_view,'view_quality':16/24}
belief=SemanticRegionBelief()
belief.register([a,b])
first=belief.observe(a,False)
repeat=belief.observe(a,False)
second=belief.observe(b,False)
assert first['new_belief']==repeat['new_belief']
assert math.isclose(second['incremental_coverage'],8/24)
assert second['new_belief']<first['new_belief']
assert belief.observed_samples['figure_region']==set(range(16))
assert base['semantic_prior_score']==.8
report['displayed_coverage_inset']={
    'sample_count':24,'prior_covered_ids':prior,
    'next_view_visible_ids':next_view,'newly_covered_ids':list(range(8,16)),
    'first_negative':first,'repeated_negative':repeat,'next_negative':second,
    'sample_geometry_and_visibility_are_schematic':True,
    'numeric_beliefs_not_displayed_as_experimental_results':True}

geometry=json.loads((OUT/'scene_geometry.json').read_text())
assert geometry['camera_pitch_deg']==0
assert geometry['camera_horizontal_fov_deg']==44
for name in ('q4_bbox','q5_bbox'):
    x0,y0,x1,y1=geometry[name]
    assert 0<x0<x1<1 and 0<y0<y1<1,(name,geometry[name])
for pos in geometry['positions_m'].values():
    assert math.isclose(pos[2],geometry['goals_m']['new'][2])
report['scene_checks']={
    'level_camera_pitch':True,'frustum_and_observation_fov_match':True,
    'projected_instance_boxes_within_both_observation_windows':True,
    'all_scene_coordinates_are_illustrative':True,
    'supporting_view_order_not_asserted':True,
    'arrows_show_search_order_not_motion_planner_output':True,
    'blender_geometry_and_sensor_views_not_experimental_evidence':True,
    'geometry_file':'scene_geometry.json'}
(OUT/'mechanism_evidence.json').write_text(json.dumps(report,indent=2)+'\n')
print('Local retry, exhaustion, permitted continuation, coverage deduplication, positive resolution and instance fusion checks passed.')
