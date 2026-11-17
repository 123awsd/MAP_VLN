import bpy, math, json
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
out=Path(__file__).resolve().parent
bpy.ops.wm.open_mainfile(filepath=str(out/'candidate_selection_scene.blend'))
s=bpy.context.scene
remove_materials={'Selected blue','Candidate gray','Collision red','Invisible purple','Sampling guides'}
for o in list(bpy.data.objects):
 if any(m and m.name in remove_materials for m in o.data.materials) if o.type in {'MESH','CURVE'} else False:
  bpy.data.objects.remove(o,do_unlink=True)
# Paper-white surroundings, softly lit oak, and a clearly separated green surface.
s.camera.location=(4,-6,8);s.camera.rotation_euler=(Vector((0,0,.70))-s.camera.location).to_track_quat('-Z','Y').to_euler();s.camera.data.ortho_scale=10.0
s.render.resolution_x=2000;s.render.resolution_y=1400;s.cycles.samples=96
s.render.film_transparent=True
for o in bpy.data.objects:
 if o.name=='Region perimeter':
  o.data.bevel_depth=.018
  m=o.data.materials[0];m.use_nodes=True;m.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value=(.01,.27,.06,1)
 if o.type=='MESH':
  for face in o.data.polygons:face.use_smooth=True
  o.data.use_auto_smooth=True
s.render.filepath=str(out/'polished_scene.png')
bpy.context.view_layer.update()
def proj(p):
 v=world_to_camera_view(s,s.camera,Vector(p));return [round(v.x*2000,3),round((1-v.y)*1400,3)]
def pos(r,a):a=math.radians(a);return (r*math.cos(a),r*math.sin(a),1.02)
# Illustrative spherical body: 0.20 m body radius + 0.10 m margin.
BODY_RADIUS_M=.20
SAFETY_MARGIN_M=.10
SAFE_RADIUS_M=BODY_RADIUS_M+SAFETY_MARGIN_M
obstacles=[o for o in bpy.data.objects if o.type=='MESH' and
           (o.name.startswith('Seat ') or o.name.startswith('Table '))]
def clearance(p):
 p=Vector(p);best=float('inf');nearest=None
 for obj in obstacles:
  inv=obj.matrix_world.inverted();q=inv@p
  ok,point,normal,index=obj.closest_point_on_mesh(q)
  distance=(obj.matrix_world@point-p).length if ok else float('inf')
  if distance<best:best=distance;nearest=obj.name
 return best,nearest


# Use the repository's generator and selector directly. Only the geometry
# provider is adapted from the voxel map to this synthetic Blender scene.
import sys, hashlib
root=Path('/home/uav/map_VLN/PRE_MAP_VLN')
sys.path.insert(0,str(root))
import stage2.candidate_poses as candidate_module
from stage2.candidate_poses import (
 generate_candidates,_region_samples,_automatic_observation_profile,
 _height_candidates_for_view,_visible_region_samples,
)
class BlenderGeometry:
 def is_state_valid(self,xyz):return clearance(xyz)[0]>SAFE_RADIUS_M
 def line_of_sight(self,start_xyz,end_xyz):
  # Match voxel_map_3d's 0.25 m target endpoint allowance. Lift a coplanar
  # tabletop ray 0.1 mm to avoid numerical self-intersection with its surface.
  start=Vector(start_xyz);end=Vector(end_xyz);delta=end-start
  if delta.length<1e-9:return True
  checked=max(0,delta.length-.25)
  start.z+=.0001;end=start+delta.normalized()*checked
  if checked==0:return True
  for obj in obstacles:
   evaluated=obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
   inv=obj.matrix_world.inverted();origin=inv@start;local_end=inv@end;local_delta=local_end-origin
   hit,*_=evaluated.ray_cast(origin,local_delta.normalized(),distance=local_delta.length)
   if hit:return False
  return True
geometry=BlenderGeometry()
anchor={'id':'table_anchor','label':'table','center_xyz_m':[0.,0.,.5],
        'size_xyz_m':[2.,1.,1.],'orientation_wxyz':[1.,0.,0.,0.],'probability':1.0}
scene_graph={'rooms':[{'id':1,'semantic_type':'dining_room','floor_id':0,
                      'camera_height_band_m':[.65,1.8],'objects':[anchor]}]}
task={'id':'inspect_tabletop','target':{'label':'table','object_id':'table_anchor'},
      'verification_label':'table','spatial_constraints':{
        'relation':'on','region_type':'support_surface','distance_m':None,
        'observation_detail':'normal','horizontal_fov_deg':69.,'vertical_fov_deg':42.,
        'visibility_required':True}}
# Figure-only spacing adjustment: shift middle ring outward by 0.15 m.
# Use the same adjusted profile for generation, scoring and plotted rings.
base_profile=_automatic_observation_profile
MIDDLE_RING_OFFSET_M=.15
def figure_profile(size,horizontal_fov_deg):
 near,far,middle=base_profile(size,horizontal_fov_deg)
 return near,far,min(far,middle+MIDDLE_RING_OFFSET_M)
candidate_module._automatic_observation_profile=figure_profile
_automatic_observation_profile=figure_profile
# Move the front-left chair toward the center until one additional sample
# ray is screened from the original between-chairs observation position.
probe=Vector((0.,-(.5*math.hypot(2.,1.)+figure_profile([2.,1.,1.],69.)[2]),1.))
probe_samples=_region_samples([0.,0.,.5],[2.,1.,1.],0.,'support_surface')
moving=[o for o in obstacles if o.name.startswith('Seat 1 ')]
baseline=[geometry.line_of_sight(probe,p) for p in probe_samples]
chosen_shift=0.
for step in range(1,41):
 for obj in moving:obj.location.x+=.005
 bpy.context.view_layer.update()
 current=[geometry.line_of_sight(probe,p) for p in probe_samples]
 chosen_shift=step*.005
 if sum(a and not b for a,b in zip(baseline,current))>=1:break
print('CHAIR SHIFT',chosen_shift,'baseline clear',sum(baseline),'after clear',sum(current))
# Slightly irregular, plausible chair positions: pushed in / pulled out,
# with individual rotations. Keep the tabletop and observation region fixed.
from mathutils import Matrix
# Add a fifth dining chair at the short end, farther from the table.
for src in [o for o in list(obstacles) if o.name.startswith('Seat 4 ')]:
 obj=src.copy();obj.data=src.data.copy();bpy.context.collection.objects.link(obj)
 obj.name=src.name.replace('Seat 4 ','Seat 5 ',1);obstacles.append(obj)
chair_layout=[('Seat 1 ',(-.67,-.84),-12),
              ('Seat 2 ',(.58,-1.30),14),
              ('Seat 3 ',(-.67,1.14),10),
              ('Seat 4 ',(.62,.79),-17),
              ('Seat 5 ',(1.55,.05),-77)]
for prefix,xy,angle_deg in chair_layout:
 parts=[o for o in obstacles if o.name.startswith(prefix)]
 seat_obj=next(o for o in parts if 'Chair seat' in o.name)
 old=seat_obj.location.copy();new=Vector((xy[0],xy[1],old.z))
 rot=Matrix.Rotation(math.radians(angle_deg),3,'Z')
 for obj in parts:
  obj.location=new+rot@(obj.location-old)
  obj.rotation_euler.z+=math.radians(angle_deg)
bpy.context.view_layer.update()
BUDGET=4
samples_for_selection=_region_samples(anchor['center_xyz_m'],anchor['size_xyz_m'],0.,'support_surface')
profile=figure_profile(anchor['size_xyz_m'],69.)
box_radius=.5*math.hypot(2.,1.)
pool=[]
for ring_index,offset in enumerate((profile[0],)):
 radius=box_radius+offset
 for azimuth_index in range(24):
  angle=2*math.pi*azimuth_index/24;xy=[radius*math.cos(angle),radius*math.sin(angle)]
  for z in _height_candidates_for_view(xy,samples_for_selection,task['spatial_constraints'],42.,[.65,1.8]):
   if not geometry.is_state_valid(xy+[z]):continue
   yaw=math.atan2(-xy[1],-xy[0]);ids=_visible_region_samples(geometry,xy,z,yaw,samples_for_selection,69.,42.)
   if not ids:continue
   # Angular footprint is an image-scale proxy, not measured pixel resolution.
   offsets=[math.atan2(math.sin(math.atan2(t[1]-xy[1],t[0]-xy[0])-yaw),math.cos(math.atan2(t[1]-xy[1],t[0]-xy[0])-yaw)) for t in samples_for_selection]
   detail=max(offsets)-min(offsets)
   pool.append({'id':f'ring{ring_index}_az{azimuth_index:02d}',
                'pose':{'x':xy[0],'y':xy[1],'z':z,'yaw':yaw},
                'candidate_angle_rad':angle,'distance_to_box_surface_m':offset,
                'visible_region_sample_ids':ids,'view_quality':len(ids)/9,
                'angular_footprint_rad':detail,'ring':ring_index,'azimuth_index':azimuth_index,
                'terminal_cost':None})
# First: most coverage, then largest angular footprint, then stable azimuth.
# Others: new coverage, largest minimum separation, coverage, image scale.
first=max(pool,key=lambda c:(len(c['visible_region_sample_ids']),round(c['angular_footprint_rad'],9),-c['azimuth_index'],-c['ring']))
selected=[first];first['selection_reason']='maximum coverage, then angular footprint'
covered=set(first['visible_region_sample_ids'])
while len(selected)<BUDGET:
 options=[c for c in pool if c not in selected]
 def key(c):
  separation=min(abs(math.atan2(math.sin(c['candidate_angle_rad']-old['candidate_angle_rad']),math.cos(c['candidate_angle_rad']-old['candidate_angle_rad']))) for old in selected)
  return (len(set(c['visible_region_sample_ids'])-covered),round(separation,9),len(c['visible_region_sample_ids']),round(c['angular_footprint_rad'],9),-c['azimuth_index'],-c['ring'])
 # Keep crosses as visibility-loss markers, not hard feasibility rejections.
 # Prioritize complete observation whenever it is available at this stage.
 full=[c for c in options if len(c['visible_region_sample_ids'])==9]
 best=max(full or options,key=key);best['selection_reason']='new coverage, then angular separation, coverage and angular footprint'
 selected.append(best);covered.update(best['visible_region_sample_ids'])
assert len(selected)==4

# Record inputs and the unmodified generator's full output for reproducibility.
source=root/'stage2/candidate_poses.py'
report={'generator':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'geometry':'synthetic Blender mesh LOS + conservative sphere/OBB collision',
        'endpoint_allowance_m':.25,'coplanar_ray_lift_m':.0001,
        'body_radius_m':BODY_RADIUS_M,'safety_margin_m':SAFETY_MARGIN_M,
        'chair_center_shift_m':chosen_shift,'chair_probe_before':baseline,'chair_probe_after':current,'figure_middle_ring_extra_offset_m':MIDDLE_RING_OFFSET_M,'figure_selection_constraint':'coverage then angular footprint for first; new coverage and angular separation for remaining; no preferred distance or forced far ring','figure_visibility_policy':'prefer fully covered candidates; yellow cross denotes any coverage loss','candidate_budget_per_location':BUDGET,'scene_graph':scene_graph,'task':task,
        'scenario':'inner ring; five chairs at varied table distances and rotations','selection_implementation':'figure-local proposed coverage/detail/diversity policy; not the production selector','selection_pool':pool,'selected_candidates':selected,'raw_samples':[]}
near,far,middle=_automatic_observation_profile(anchor['size_xyz_m'],69.)
R=.5*math.hypot(2.,1.)
samples=_region_samples(anchor['center_xyz_m'],anchor['size_xyz_m'],0.,'support_surface')
data={'rings':[],'dots':[],'views':[],'collisions':[],'occluded':[],'invisibles':[],
      'safe_radius_m':SAFE_RADIUS_M,'selected_candidates':selected,'audit':report['raw_samples']}
for ring_index,offset in enumerate((near,)):
 radius=R+offset
 # For this level support surface, normal observation produces z=1.0.
 ring_z=samples[0][2]
 data['rings'].append([proj((radius*math.cos(math.radians(a)),radius*math.sin(math.radians(a)),ring_z)) for a in range(361)])
 for index in range(24):
  angle=2*math.pi*index/24;xy=[radius*math.cos(angle),radius*math.sin(angle)]
  heights=_height_candidates_for_view(xy,samples,task['spatial_constraints'],42.,[.65,1.8])
  for z in heights:
   p=xy+[z];valid=geometry.is_state_valid(p);yaw=math.atan2(-xy[1],-xy[0])
   ids=_visible_region_samples(geometry,xy,z,yaw,samples,69.,42.) if valid else []
   audit={'ring':ring_index,'azimuth_deg':index*15,'position':p,'collision':not valid,
          'clearance_m':clearance(p)[0],'visible_ids':ids,'visible_samples':len(ids) if valid else None}
   report['raw_samples'].append(audit)
   if not valid:data['collisions'].append(proj(p))
   elif len(ids)<len(samples):
    data['occluded'].append(proj(p))
    if not ids:data['invisibles'].append(proj(p))
   else:data['dots'].append(proj(p))
for candidate in selected:
 q=candidate['pose'];p=Vector((q['x'],q['y'],q['z']));f=Vector((math.cos(q['yaw']),math.sin(q['yaw']),0));side=f.cross(Vector((0,0,1))).normalized();up=Vector((0,0,1));c=p+f*.42
 corners=[c+side*x*(.42*math.tan(math.radians(69/2)))+up*y*(.42*math.tan(math.radians(42/2))) for x,y in [(-1,-1),(1,-1),(1,1),(-1,1)]]
 data['views'].append({'p':proj(p),'corners':[proj(v) for v in corners],
                       'candidate_id':candidate['id'],'visible_samples':len(candidate['visible_region_sample_ids']),
                       'position':list(p),'terminal_cost':candidate['terminal_cost']})
assert len(selected)<=BUDGET
assert all(geometry.is_state_valid([q['pose'][k] for k in ('x','y','z')]) for q in selected)
print('ALGORITHM OUTPUT',[(round(math.degrees(q['candidate_angle_rad'])),round(q['distance_to_box_surface_m'],3),q['view_quality']) for q in selected])
print('GEOMETRY',len(data['audit']),'raw samples',len(data['collisions']),'collision',len(data['occluded']),'occluded',len(selected),'retained')
# Debug actual mesh intersections for the selected viewpoint closest to the chairs.
# Keep the diagnostic rays at the original between-chairs probe, even if
# the planner chooses a different viewpoint after the chair moves.
start=probe.copy()
data['debug_rays']={'source':proj(start),'position':list(start),'rays':[]}
for target_xyz in samples:
 target=Vector(target_xyz);origin=start+Vector((0,0,.0001));delta=target+Vector((0,0,.0001))-origin
 nearest=None;blocker=None;best=delta.length
 for obj in obstacles:
  inv=obj.matrix_world.inverted();local_start=inv@origin;local_end=inv@(origin+delta);local_delta=local_end-local_start
  hit,loc,*_=obj.evaluated_get(bpy.context.evaluated_depsgraph_get()).ray_cast(local_start,local_delta.normalized(),distance=max(0,local_delta.length-.00001))
  if hit:
   world=obj.matrix_world@loc;distance=(world-origin).length
   if distance<best:nearest=world;best=distance;blocker=obj.name
 data['debug_rays']['rays'].append({'target':proj(target),'target_xyz':list(target),'full_ray_blocked':nearest is not None,'blocker':blocker,'hit':proj(nearest) if nearest is not None else None,'generator_los':geometry.line_of_sight(start,target)})
report['debug_rays']=data['debug_rays']
(out/'algorithm_candidates.json').write_text(json.dumps(report,indent=2))
(out/'polished_projection.json').write_text(json.dumps(data))
bpy.ops.wm.save_as_mainfile(filepath=str(out/'algorithm_scene.blend'))
bpy.ops.render.render(write_still=True)
