#!/usr/bin/env python3
"""Order exploration-derived anchors and route using MINCO's collision PCD."""
import argparse
import itertools
import json
import sys
from pathlib import Path
import numpy as np
from matplotlib.path import Path as Polygon
from scipy.spatial import cKDTree
from run_pcd_falcon import load_environment, astar_grid, nearest_free_cell
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'real_fly/stage2_runtime/scripts'))
from build_super_collision_pcd import read_binary_xyz


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-data', type=Path, required=True)
    p.add_argument('--preview', type=Path, required=True)
    p.add_argument('--collision', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--include-room', action='append', default=[])
    p.add_argument('--complete-corridors-from', type=Path)
    a = p.parse_args()
    m, occ, esdf, origin, res, allowed, *_ = load_environment(
        a.run_data/'voxel_snapshot', a.run_data/'approved_scene_graph.json', set(),
        a.run_data/'falcon_exploration_boundary.json')
    tree = cKDTree(read_binary_xyz(a.collision))
    z = int((1.1-origin[2])//res)
    yy, xx = np.indices(allowed.shape)
    centers = np.column_stack((origin[0]+(xx.ravel()+.5)*res,
                               origin[1]+(yy.ravel()+.5)*res, np.full(xx.size,1.1)))
    distances = tree.query(centers)[0].reshape(allowed.shape)
    boundary = json.loads((a.run_data/'falcon_exploration_boundary.json').read_text())
    boundary_distance = np.full(len(centers),np.inf)
    for vertices in [boundary['allowed_polygon_xy_m']] + [p['polygon_xy_m'] for p in boundary['excluded_polygons']]:
        vertices = np.asarray(vertices)
        for first,second in zip(vertices,np.roll(vertices,-1,axis=0)):
            edge=second-first
            ratio=np.clip((centers[:,:2]-first)@edge/max(float(edge@edge),1e-12),0,1)
            d=np.linalg.norm(centers[:,:2]-(first+ratio[:,None]*edge),axis=1)
            boundary_distance=np.minimum(boundary_distance,d)
    # Use actual edge sampling instead of a coarse worst-case inflation.
    # Required clearance stays .25; .02 is a guide construction margin.
    free = allowed & (occ[z] == 0) & (distances >= .27) & (boundary_distance.reshape(allowed.shape)>=.15)
    edge_cache = {}
    def edge_allowed(first, second):
        key = tuple(sorted((first,second)))
        if key not in edge_cache:
            start, end = [np.array([origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,1.1])
                          for y,x in (first,second)]
            edge_cache[key] = bool(tree.query(start+np.linspace(0,1,31)[:,None]*(end-start))[0].min()>=.27)
        return edge_cache[key]
    result = json.loads((a.preview/'result.json').read_text())
    anchors = [[0.,0.,1.1]] + [v['start_xyz'] for v in result['fallback_segments'][1:8]]
    base_count = len(anchors)
    supplements=[]
    scene=json.loads((a.run_data/'approved_scene_graph.json').read_text())
    safe_indices=np.flatnonzero(free.ravel())
    for name in a.include_room:
        room=next(r for r in scene['rooms'] if r['name']==name)
        ids=safe_indices[Polygon(room['polygon_xy_m']).contains_points(centers[safe_indices,:2])]
        if not len(ids):raise SystemExit(f'No safe point inside {name}')
        best=ids[np.argmin(np.linalg.norm(centers[ids,:2]-room['centroid_xy_m'],axis=1))]
        anchors.append(centers[best].tolist())
        supplements.append({'room':name,'xyz':anchors[-1]})
    if a.complete_corridors_from:
        old=json.loads(a.complete_corridors_from.read_text())
        old_points=np.vstack([s['points_xyz_m'] for s in old['segments']])
        route_tree=cKDTree(old_points[:,:2]); choices=[]
        for room in scene['rooms']:
            if not room['name'].startswith('Corridor'):continue
            ids=safe_indices[Polygon(room['polygon_xy_m']).contains_points(centers[safe_indices,:2])]
            if not len(ids):continue
            ds=route_tree.query(centers[ids,:2])[0]; best=ids[np.argmax(ds)]
            choices.append((float(ds.max()),room['name'],best))
        gap,name,best=max(choices)
        print(f'Largest allowed corridor gap: {name}, {gap:.3f}m',flush=True)
        if gap>.5:
            anchors.append(centers[best].tolist())
            supplements.append({'room':name,'xyz':anchors[-1],'previous_route_distance_m':gap})
    cells = []
    snapped = []
    for xyz in anchors:
        y,x = np.floor((np.asarray(xyz[:2])-origin[:2])/res).astype(int)[::-1]
        cell = nearest_free_cell((y,x),free)
        if cell is None:
            raise SystemExit('Anchor has no safe collision-PCD cell')
        y,x=cell
        point=np.array([origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,1.1])
        if np.linalg.norm(point-np.asarray(xyz))>.8:
            raise SystemExit('Anchor adjustment > .8m; inspect before proceeding')
        cells.append(cell); snapped.append(point)
    paths={}; costs={}
    for i,j in itertools.combinations(range(len(cells)),2):
        path=astar_grid(cells[i],cells[j],free,edge_allowed=edge_allowed)
        if not path:
            raise SystemExit(f'No safe connection between exploration anchors {i} and {j}')
        xyz=np.asarray([[origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,1.1] for y,x in path])
        paths[i,j]=xyz; paths[j,i]=xyz[::-1]
        costs[i,j]=costs[j,i]=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum())
    order=min(itertools.permutations(range(1,base_count)),
              key=lambda v:sum(costs[i,j] for i,j in zip((0,)+v,v)))
    order=[0,*order]
    for extra in range(base_count,len(cells)):
        alternatives=[order[:i]+[extra]+order[i:] for i in range(1,len(order)+1)]
        order=min(alternatives,key=lambda v:sum(costs[i,j] for i,j in zip(v[:-1],v[1:])))
    points=[np.array([0.,0.,1.1])]
    for i,j in zip(order[:-1],order[1:]):
        points.extend(paths[i,j])
    xyz=np.asarray(points)
    samples=[]
    for b,c in zip(xyz[:-1],xyz[1:]):
        n=max(1,int(np.ceil(np.linalg.norm(c-b)/.01)))
        samples.extend(b+np.linspace(0,1,n+1)[:,None]*(c-b))
    minimum=float(tree.query(np.asarray(samples))[0].min())
    if minimum<.25:
        raise SystemExit(f'Continuous PCD clearance failed: {minimum}')
    # Preserve dense edges and the anchor phases. The existing ascending
    # CIRI seed selector skips <.4m points; sparse corner-only input can make
    # that selector cut across a long following edge.
    visits=[]; segments=[]; previous=np.array([0.,0.,1.1]); pending=[]
    corridor_pass_ids={base_count+i for i,v in enumerate(supplements) if v['room'].startswith('Corridor')}
    for index,(i,j) in enumerate(zip(order[:-1],order[1:])):
        leg=np.vstack((previous,paths[i,j]))
        end=leg[-1]
        pending.extend(leg.tolist())
        previous=end
        if j in corridor_pass_ids and index+1<len(order)-1:
            continue
        visits.append({'task_id':f'exploration_{index}', 'candidate_id':f'anchor_{j}',
                       'dwell_s':.1,'pose':dict(zip(('x','y','z','yaw'),[*end,0]))})
        segments.append({'points_xyz_m':pending})
        pending=[]
    report={'route_source':'offline hybrid coverage anchors, shortest grid tour',
            'length_m':float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
            'pcd_clearance_m':minimum,'anchor_order':order,'height_m':1.1,
            'supplemental_visits':supplements,
            'automatic_frontier_completion_claimed':False}
    mission=dict(m['identity']); mission.update({
        'format':'pre_map_vln.mission_plan.v1','start_xyz_yaw':[0,0,1.1,0],
        'start_pose_explicitly_approved':True,'real_start_source':'explicit_offline_start_0_0_1_1',
        'safety_note':'Explicit planning start only; does not authorize flight or bypass live start checks.',
        'exploration_provenance':report,
        'visits':visits,'segments':segments})
    if a.output.exists(): raise SystemExit('Refusing to overwrite')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(mission,indent=2))
    (a.output.parent/'candidates.json').write_text('{"by_task":{}}')
    (a.output.parent/'exploration_route_report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
