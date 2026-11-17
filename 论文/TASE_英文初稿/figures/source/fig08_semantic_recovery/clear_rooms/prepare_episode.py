"""Author a forward-progressing multiroom task and smooth it with MINCO S4NU."""
from pathlib import Path
import json,math,heapq
import numpy as np
from PIL import Image,ImageDraw
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
import habitat_sim
from minco import interpolate

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[3]
cfg=json.loads((ROOT/'PRE_MAP_VLN/outputs/stage2_3d/prepared/00337_reference/generated_stage1_config.json').read_text())
reg=json.loads((OUT/'assets/registration.json').read_text())
T=np.array(reg['falcon_xy_to_pixel']);inv=np.linalg.inv(np.vstack([T,[0,0,1]]))
R=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
origin=np.array(cfg['initial_sensor_habitat_xyz'])
pf=habitat_sim.PathFinder();assert pf.load_nav_mesh(str(Path(cfg['scene']).with_suffix('.navmesh')))
lower,upper=map(np.asarray,pf.get_bounds());resolution=.04
floor=cfg['initial_agent_habitat_xyz'][1]+2.6
mask=np.asarray(pf.get_topdown_view(resolution,floor),bool)
clear=distance_transform_edt(mask)*resolution
def falcon(uv,z=3.9):
    q=inv@[*uv,1];return [float(q[0]),float(q[1]),z]
def habitat(q):return origin+R.T@(np.array(q)-[0,0,1])
def pixel(q):return (T@[*q[:2],1]).tolist()
def grid(q):
    h=habitat(q);return np.array([(h[0]-lower[0])/resolution,(h[2]-lower[2])/resolution])
def from_grid(xy,z=3.9):
    h=[lower[0]+(xy[0]+.5)*resolution,floor,lower[2]+(xy[1]+.5)*resolution]
    f=R@(np.array(h)-origin)+[0,0,1];f[2]=z;return f
def cell(q):return tuple(np.floor(grid(q)).astype(int))
def valid(q):
    x,y=cell(q)
    return 0<=y<mask.shape[0] and 0<=x<mask.shape[1] and mask[y,x]
def clearance(q):
    x,y=cell(q)
    return float(clear[y,x]) if 0<=y<mask.shape[0] and 0<=x<mask.shape[1] else 0

def astar(a,b):
    start,end=cell(a),cell(b)
    assert valid(a) and valid(b),(pixel(a),pixel(b))
    heap=[(0,start)];dist={start:0};prev={}
    while heap:
        _,u=heapq.heappop(heap)
        if u==end:break
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            v=(u[0]+dx,u[1]+dy);x,y=v
            if not(0<=x<mask.shape[1] and 0<=y<mask.shape[0] and mask[y,x]):continue
            if dx and dy and (not mask[u[1],x] or not mask[y,u[0]]):continue
            cost=math.hypot(dx,dy)*(1+1.3*math.exp(-clear[y,x]/.22))
            nd=dist[u]+cost
            if nd<dist.get(v,float('inf')):
                dist[v]=nd;prev[v]=u
                heapq.heappush(heap,(nd+math.hypot(x-end[0],y-end[1]),v))
    if end not in dist:raise RuntimeError('No route')
    path=[end]
    while path[-1]!=start:path.append(prev[path[-1]])
    path.reverse();points=[np.array(a)]+[from_grid(q) for q in path[1:-1]]+[np.array(b)]
    # Simplify while preserving a clearance margin along every chord.
    out=[points[0]];i=0
    while i<len(points)-1:
        j=i+1
        for k in range(i+2,len(points)):
            if np.linalg.norm(points[k][:2]-points[i][:2])>1.5:break
            samples=np.linspace(points[i],points[k],max(2,math.ceil(np.linalg.norm(points[k]-points[i])/.025)))
            limit=min(.20,clearance(points[i]),clearance(points[k]))-.01
            if all(clearance(q)>=max(.03,limit) for q in samples):j=k
        out.append(points[j]);i=j
    return out

# Official pixel positions define the visual story; projected to the native scene.
spec=[('start',(1062,900),(1120,860)),
      ('kitchen',(1098,648),(1170,720)),
      ('chair_first',(1075,620),(1165,625)),
      ('chair_retry',(1090,550),(1170,625)),
      ('sofa',(1075,470),(1210,500)),
      ('lamp',(230,855),(110,865))]
stages=[]
for name,uv,target in spec:
    p=falcon(uv);aim=falcon(target,z=3.15 if name.startswith('chair_') else 3.35)
    stages.append(dict(name=name,pose_falcon=p,target_falcon=aim,
                       yaw=math.atan2(aim[1]-p[1],aim[0]-p[0]),
                       official_pixel=list(uv),grid_pixel=grid(p).tolist()))
    print(name,'clearance',clearance(p),flush=True)

# Inspect the dining chairs, recover to their nearby sofa, then cross the hall
# to verify the bedside lamp in a bedroom that has not been visited earlier.
forced={4:[(1014,554),(872,592),(684,595),(540,588),(402,590),(322,584),
           (253,691),(251,758)]}
routes=[];lengths=[];coefficients=[]
for i in range(5):
    way=[stages[i]['pose_falcon']]+[falcon(uv) for uv in forced.get(i,[])]+[stages[i+1]['pose_falcon']]
    guides=[]
    for a,b in zip(way,way[1:]):
        path=astar(a,b);guides.extend(path if not guides else path[1:])
    # Fixed-time minimum-snap solve; refine guides if interpolation exits free space.
    for attempt in range(5):
        samples,co=interpolate(guides)
        bad=[q for q in samples if not valid(q)]
        if not bad:break
        guides=[q for a,b in zip(guides,guides[1:]) for q in (a,(np.array(a)+np.array(b))/2)]+[guides[-1]]
    assert not bad,f'MINCO segment {i} leaves navigable space'
    lengths.append(float(np.linalg.norm(np.diff(samples[:,:2],axis=0),axis=1).sum()))
    routes.append(dict(falcon=samples.tolist(),official_pixel=[pixel(q) for q in samples],grid_pixel=[grid(q).tolist() for q in samples]))
    co.update(guide_points_falcon=np.array(guides).tolist(),min_ground_clearance_m=min(clearance(q) for q in samples))
    coefficients.append(co);print('segment',i,'length',lengths[-1],'samples',len(samples),'clearance',co['min_ground_clearance_m'],flush=True)

# Endpoint must be separated from all earlier route samples except the final leg.
prior=np.concatenate([np.array(r['falcon'])[:,:2] for r in routes[:-1]])
endpoint=np.array(stages[-1]['pose_falcon'])[:2]
target_prior=cKDTree(prior).query(endpoint)[0]
out=dict(scene=cfg['scene'],scene_id='00337-CFVBbU9Rsyb',floor=2,
    instruction='Guests are arriving. Check the kettle on the kitchen counter, then look for a cushion on the dining chairs. If it is not there, keep looking nearby. Finally, check the lamp between the beds in the twin bedroom.',
    map=dict(api='habitat_sim.PathFinder.get_topdown_view',resolution_m=resolution,height_habitat=floor,shape=list(mask.shape),lower_habitat=lower.tolist(),role='Static native navigability reference'),
    stages=stages,routes=routes,route_length_m=sum(lengths),segment_lengths_m=lengths,
    registration=reg,final_pose_distance_to_earlier_route_m=float(target_prior),
    sequence_role='Authored task and guide sequence. Fixed-time MINCO S4NU equations transcribed from project backend. Not the complete production corridor/time optimization pipeline.',
    curve_checks=dict(all_sampled_ground_projections_navigable=True,max_sample_spacing_m=max(float(np.linalg.norm(np.diff(np.array(r['falcon'])[:,:2],axis=0),axis=1).max()) for r in routes)),
    qwen_role='Illustrative candidate, not a recorded Qwen inference.')
out['task_semantics']=dict(intent='Household object verification',
    ordered_targets=['kettle on kitchen counter','cushion initially on dining chair','lamp between twin beds'],
    recovery=dict(target='cushion',initial_anchor='dining chair',candidate_anchor='sofa',
                  candidate_room='eastern living room',
                  trigger='Local chair viewpoints exhausted without observing the cushion'),
    color_attributes_required=False)
(OUT/'episode_geometry.json').write_text(json.dumps(out,indent=2)+'\n')
(OUT/'minco_coefficients.json').write_text(json.dumps(dict(source='SUPER MINCO_S4NU::setParameters, fixed-time equations',phases=coefficients),indent=2)+'\n')
Image.fromarray(np.where(mask,255,43).astype('uint8')).save(OUT/'assets/habitat_topdown.png')
im=Image.open(OUT/'assets/official_l2.png');canvas=Image.new('RGBA',im.size,'white');canvas.alpha_composite(im);d=ImageDraw.Draw(canvas)
for i,r in enumerate(routes):
    d.line([tuple(q) for q in r['official_pixel']],fill=['#39BCA6','#7CBF65','#C2C955','#D5B64C','#DF9146'][i],width=5)
for s in stages:
    x,y=s['official_pixel'];d.ellipse((x-7,y-7,x+7,y+7),fill='purple');d.text((x+9,y),s['name'],fill='black')
canvas.save(OUT/'route_check.png')
