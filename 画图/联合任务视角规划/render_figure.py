#!/usr/bin/env python3
"""Code-audited, editable vector illustration of rolling task–viewpoint planning."""
from pathlib import Path
from itertools import permutations, product
import hashlib
import inspect
import json
import math
import os
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/jtvp_figure_mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch, FancyArrowPatch, ConnectionPatch, Polygon
import numpy as np
from scipy.ndimage import distance_transform_edt

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sys.path.insert(0,str(ROOT/'PRE_MAP_VLN'))
from stage2.astar_3d import CoarseAstar3D
from stage2.voxel_map_3d import VoxelMap3D
from stage2.planning_contract import PlannerProfile, MapIdentity, OCCUPANCY_SEMANTICS_HASH
from stage2.motion_cost_oracle import MotionCostOracle
from stage2.joint_planner import plan_joint_mission, PlanningError, _motion_time
from stage2.representative_viewpoints import select_location_representatives
from stage2.mission_executor import MissionState

D=json.loads((OUT/'example_inputs.json').read_text())
W,H=184.,108.
PANEL_RADIUS=2.5
INK='#26343D'; MUTED='#687C89'
BACKGROUND='#FAFBFC'
WALL='#B1BBC4'
TRAJECTORY='#287A91'
NEW_ACTIVE='#EA844A'
CHECKED='#8B91A0'
RULE='#D5DCE2'; WALL_EDGE='#98A5AF'; HEADER='#EEF1F4'
OBJECT='#E1E6EB'; OBJECT_DETAIL='#F3F5F7'; OBJECT_EDGE='#A4AFBA'
FLOORS={'A':'#F6F3EF','B':'#EFF4F6','C':'#F1F5F0','D':'#F7F2EC'}
GRID='#DDE3E3'
WALL_LIGHT='#CDD4D8'; WALL_SHADE='#94A0A8'
WOOD='#CCB99D'; WOOD_LIGHT='#E2D4BE'; WOOD_SHADE='#AC9478'; WOOD_EDGE='#99866F'
PREVIOUS=CHECKED
TASK_COLORS={'A':TRAJECTORY,'B':TRAJECTORY,'C':TRAJECTORY,'D':NEW_ACTIVE}
TASK_PALE={task:BACKGROUND for task in TASK_COLORS}
plt.rcParams.update({'font.family':'Liberation Sans','font.size':7,
                     'pdf.fonttype':42,'svg.fonttype':'none','hatch.linewidth':.3,
                     'lines.solid_capstyle':'round','lines.solid_joinstyle':'round'})


def build_map():
    profile=PlannerProfile.load(ROOT/'PRE_MAP_VLN/config/uav_3d_planning_habitat.yaml')
    r=D['resolution']
    nx,ny,nz=[round(e/r) for e in D['extent']]
    z,y,x=np.meshgrid((np.arange(nz)+.5)*r,(np.arange(ny)+.5)*r,
                      (np.arange(nx)+.5)*r,indexing='ij')
    occupied=(x<.2)|(x>D['extent'][0]-.2)|(y<.2)|(y>D['extent'][1]-.2)|(z<.2)|(z>1.8)
    for a,b,c,d in D['walls']:
        occupied|=(x>=a)&(x<c)&(y>=b)&(y<d)
    for f in D['furniture']:
        a,b,c,d=f['box']
        occupied|=(x>=a)&(x<c)&(y>=b)&(y<d)&(z<1.4)
    raw=np.where(occupied,100,0).astype(np.int8)
    # Conservative distance to occupied voxel surfaces, not center distance.
    esdf=np.maximum(0,distance_transform_edt(~occupied)*r-r/2).astype(np.float32)
    identity=MapIdentity('jtvp-illustrative-scene',1,1,hashlib.sha256(raw.tobytes()).hexdigest(),
                         OCCUPANCY_SEMANTICS_HASH,profile.profile_hash)
    vox=VoxelMap3D(raw,esdf,[0,0,0],r,profile,identity)
    return vox,MotionCostOracle(CoarseAstar3D(vox))


def candidate_pool(vox):
    result={t['id']:[] for t in D['task_graph']['tasks']}
    for v in D['views']:
        x,y=v['xy'];ax,ay=v['aim']
        pose=dict(x=x,y=y,z=.9,yaw=math.atan2(ay-y,ax-x))
        assert vox.is_state_valid([x,y,.9]),v['id']
        result[v['task']].append(dict(id=v['id'],task_id=v['task'],object_id=v['location'],
             location_hypothesis_id=v['location'],pose=pose,terminal_cost=v['terminal']))
    return result


def rolling_plan(oracle,state,pool,current):
    """Illustrate the core DP on the admitted views, within a bounded window.

    This deliberately does not claim to run Habitat's one-representative default.
    The offline multi-target entry also passes multiple views to this core.
    """
    def task_bound(task_id):
        return min(math.dist(current[:3],[c['pose'][k] for k in ('x','y','z')])+
                   c['terminal_cost'] for c in pool[task_id])
    window=set(sorted(state.available(),key=lambda t:(task_bound(t),t))[:D['horizon']])
    plan=plan_joint_mission(oracle,D['task_graph'],pool,current,
                           active_task_ids=window,completed_task_ids=state.completed)
    return dict(available=sorted(state.available()),window=sorted(window),
                completed=sorted(state.completed),representative_limit=None,
                admitted={t:pool[t] for t in sorted(window)},start=current,plan=plan)


def enumerate_best(oracle,cycle):
    """Independent exhaustive audit of the small admitted planning problem."""
    best=math.inf;count=0
    tasks={t['id']:t for t in D['task_graph']['tasks']}
    for order in permutations(cycle['window']):
        done=set(cycle['completed'])
        if any(not set(tasks[t]['prerequisites'])<=(done|set(order[:i])) for i,t in enumerate(order)):
            continue
        for views in product(*(cycle['admitted'][t] for t in order)):
            current=cycle['start'];cost=0
            for c in views:
                pose=c['pose'];xyz=[pose[k] for k in ('x','y','z')]
                path=oracle.path(current[:3],xyz)
                if path is None:cost=math.inf;break
                cost+=path.length_m+.25*_motion_time(current,pose,path.length_m,1.,.5,1.)+2*c['terminal_cost']
                current=xyz+[pose['yaw']]
            count+=1;best=min(best,cost)
    assert math.isclose(best,cycle['plan']['objective_cost'],abs_tol=1e-8)
    return dict(enumerated_assignments=count,minimum_cost=best)


def build_evidence(include_minco=True):
    files=['joint_planner.py','representative_viewpoints.py','mission_executor.py',
           'motion_cost_oracle.py','astar_3d.py','voxel_map_3d.py']
    hashes={f:hashlib.sha256((ROOT/'PRE_MAP_VLN/stage2'/f).read_bytes()).hexdigest() for f in files}
    vox,oracle=build_map();pool=candidate_pool(vox);state=MissionState(D['task_graph'])
    initial=rolling_plan(oracle,state,pool,D['start'])
    visit=initial['plan']['visits'][0]
    observation=D['observation']
    assert visit['task_id']==observation['task_id']=='A','The observation must follow the actual first visit.'
    # A terminal search verdict after the configured local checks, not a claim
    # that any single negative camera frame immediately triggers the fallback.
    state.finish_task(observation['task_id'],observation['outcome'])
    current=[visit['pose'][k] for k in ('x','y','z','yaw')]
    updated=rolling_plan(oracle,state,pool,current)
    audits=[]
    for cycle in (initial,updated):
        audits.append(enumerate_best(oracle,cycle))
        for seg in cycle['plan']['segments']:
            pts=seg['points_xyz_m']
            assert all(vox.is_state_valid(p) for p in pts)
            assert all(vox.line_is_valid(a,b) for a,b in zip(pts,pts[1:]))
            assert all(abs(p[2]-.9)<1e-8 for p in pts),'Plan-view projection assumes level paths.'
    assert 'D' not in initial['window'] and 'D' in updated['window'] and 'A' not in updated['window']
    assert any(not vox.line_is_valid(seg['points_xyz_m'][0],seg['points_xyz_m'][-1])
               for seg in initial['plan']['segments']), 'Show at least one real obstacle detour.'
    for f,h in hashes.items():assert hashlib.sha256((ROOT/'PRE_MAP_VLN/stage2'/f).read_bytes()).hexdigest()==h
    branch_audit={}
    for outcome in ('found','not_found'):
        branch=MissionState(D['task_graph'])
        branch.finish_task('A',outcome)
        assert ('D' in branch.available())==(outcome=='not_found')
        assert branch.status['D']==('pending' if outcome=='not_found' else 'skipped')
        branch_audit[outcome]=dict(available=sorted(branch.available()),status=dict(branch.status))
    evidence=dict(scope=D['scope'],source_sha256=hashes,input=D,initial=initial,updated=updated,
                  state_event=state.events,exhaustive_audits=audits,
                  branch_audit=branch_audit,
                  observation_note='A terminal not_found outcome activates the explicit fallback D. Local retry/verification details are abstracted. A checked anchor does not mean the target was found.',
                  projection='Top-down view of the saved MINCO polynomials; A* is shown only as a guide in the trajectory inset.',
                  pool_note='Illustrative admitted candidate poses after feasibility filtering. Exactly one physical task anchor per room; all views of a task share its location_hypothesis_id. This illustrates the core subset-DP, not the default Habitat single-representative admission policy. Candidate generation is not rerun.',
                  optimality_scope='Only the current rolling task window and admitted representatives, under the computed A* motion costs.',
                  functions={f.__name__:dict(path=inspect.getsourcefile(f),line=inspect.getsourcelines(f)[1])
                             for f in (plan_joint_mission,select_location_representatives,MissionState.available,MissionState.finish_task)})
    for task in pool:
        assert len({c['location_hypothesis_id'] for c in pool[task]}) == 1
    if not include_minco:
        return evidence
    sys.path.insert(0,str(ROOT/'PRE_MAP_VLN/real_fly/stage2_runtime/scripts'))
    from saved_minco_artifact import read_trajectory
    provenance=json.loads((OUT/'minco_provenance.json').read_text())
    assert provenance['input_sha256']==hashlib.sha256((OUT/'example_inputs.json').read_bytes()).hexdigest()
    for name,cycle in [('initial',initial),('updated',updated)]:
        path=OUT/f'{name}_minco.txt'
        assert hashlib.sha256(path.read_bytes()).hexdigest()==provenance['cycles'][name]['sha256']
        start,phases=read_trajectory(path)
        assert np.allclose(start,cycle['start'],atol=1e-8)
        assert len(phases)==len(cycle['plan']['visits'])
        for phase,visit in zip(phases,cycle['plan']['visits']):
            assert np.allclose(phase['points'][-1],[visit['pose'][k] for k in ('x','y','z')],atol=1e-8)
            assert all(vox.is_state_valid(p) for p in phase['points'])
            assert all(vox.line_is_valid(a,b) for a,b in zip(phase['points'],phase['points'][1:]))
        cycle['minco']=phases
    evidence['minco_provenance']='minco_provenance.json'
    (OUT/'mechanism_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print('Initial:',[(v['task_id'],v['candidate_id']) for v in initial['plan']['visits']])
    print('Updated:',[(v['task_id'],v['candidate_id']) for v in updated['plan']['visits']])
    return evidence


def text(ax,x,y,s,size=7,color=INK,ha='left',weight='normal',**kw):
    return ax.text(x,y,s,fontsize=size,color=color,ha=ha,va='center',weight=weight,
                   zorder=30,**kw)


def line(ax,pts,color=INK,lw=.65,z=3,**kw):
    p=np.asarray(pts)
    return ax.plot(p[:,0],p[:,1],color=color,lw=lw,zorder=z,**kw)


def rect(ax,x,y,w,h,fc='none',ec=INK,lw=.6,z=2):
    p=Rectangle((x,y),w,h,fc=fc,ec=ec,lw=lw,zorder=z);ax.add_patch(p);return p


def rounded(ax,x,y,w,h,fc,ec,lw=.6,r=1,z=2):
    p=FancyBboxPatch((x,y),w,h,boxstyle=f'round,pad=0,rounding_size={r}',
                    fc=fc,ec=ec,lw=lw,zorder=z);ax.add_patch(p);return p


def arrow(ax,a,b,color=INK,lw=.65,head=5,z=10,**kw):
    ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=head,
                               lw=lw,color=color,zorder=z,**kw))


def axes(fig,bounds,xlim,ylim,aspect='equal'):
    x,y,w,h=bounds
    ax=fig.add_axes([x/W,y/H,w/W,h/H])
    ax.set(xlim=xlim,ylim=ylim,aspect=aspect);ax.axis('off');return ax


def badge(ax,x,y,label,color,fill=None,r=2.0,size=6.7):
    ax.add_patch(Circle((x,y),r,fc=fill or color,ec=color,lw=.55,zorder=20))
    text(ax,x,y,label,size,'white' if fill is None else color,ha='center',weight='bold')


def view(ax,xy,yaw,color,fill,radius=.36,selected=False,lw=.55,apex_radius=None):
    """A compact wireframe camera symbol, oriented by the actual camera yaw.

    The quadrilateral is a schematic image plane; it is not a metric FoV
    footprint. The vertex stays at the actual observation pose.
    """
    centre=np.asarray(xy,dtype=float)
    rotation=np.array([[math.cos(yaw),-math.sin(yaw)],
                       [math.sin(yaw), math.cos(yaw)]])
    plane=radius*np.array([[.80,-.50],[1.02,-.32],[1.02,.50],[.80,.32]])@rotation.T+centre
    opacity=1. if selected else .72
    if selected and fill!=BACKGROUND:
        ax.add_patch(Polygon(plane,closed=True,fc=color,ec='none',alpha=.09,zorder=12.8))
    # Fine internal rays and a stronger outer frame keep selected poses clear
    # without a solid sector obscuring the scene behind the symbol.
    for corner in plane:
        line(ax,[centre,corner],color,lw*.72,z=13,alpha=opacity)
    ax.add_patch(Polygon(plane,closed=True,fc='none',ec=color,
                        lw=lw*1.15,alpha=opacity,zorder=14))
    line(ax,[plane[0],centre,plane[2]],color,lw*1.15,z=14,alpha=opacity)
    if apex_radius is None:apex_radius=.064 if selected else .047
    ax.add_patch(Circle(xy,apex_radius,fc=color if selected else BACKGROUND,
                        ec=BACKGROUND if selected else color,lw=.45,zorder=15))


def drone(ax,pose,scale=1.):
    """Centred, symmetric plan-view quadrotor at the actual start pose."""
    x,y,_,yaw=pose
    rotation=np.array([[math.cos(yaw),-math.sin(yaw)],
                       [math.sin(yaw),math.cos(yaw)]])
    def place(points):
        return np.asarray(points)*scale@rotation.T+np.array([x,y])
    for ends in [[[-.17,-.17],[.17,.17]],[[-.17,.17],[.17,-.17]]]:
        line(ax,place(ends),BACKGROUND,2.0,z=19)
        line(ax,place(ends),TRAJECTORY,.8,z=20)
    for a,b in [(-.17,-.17),(-.17,.17),(.17,-.17),(.17,.17)]:
        centre=place([[a,b]])[0]
        ax.add_patch(Circle(centre,.093*scale,fc=BACKGROUND,ec=TRAJECTORY,lw=.55,zorder=21))
        line(ax,place([[a-.065,b],[a+.065,b]]),MUTED,.45,z=22)
    # Equal front/rear extents and paired bevels keep the tiny symbol balanced.
    body=place([[-.13,-.05],[-.10,-.075],[.10,-.075],[.13,-.05],
                [.13,.05],[.10,.075],[-.10,.075],[-.13,.05]])
    ax.add_patch(Polygon(body,closed=True,fc=OBJECT,ec=INK,lw=.65,zorder=23))
    ax.add_patch(Circle(place([[.085,0]])[0],.023*scale,fc=TRAJECTORY,
                        ec='none',zorder=24))


def wall_block(ax,bounds):
    """Bevels inside the original footprint suggest depth without moving walls."""
    a,b,c,d=bounds;w,h=c-a,d-b
    rect(ax,a,b,w,h,WALL,WALL_EDGE,.45,z=4)
    if w>=h:
        rect(ax,a,b,w,min(.065,h*.3),WALL_SHADE,'none',z=4.1)
        rect(ax,a,b+h*.68,w,h*.32,WALL_LIGHT,'none',z=4.1)
        line(ax,[[a,b+h*.68],[c,b+h*.68]],WALL_EDGE,.25,z=4.2)
    else:
        rect(ax,a,b,min(.055,w*.25),h,WALL_SHADE,'none',z=4.1)
        rect(ax,a+w*.73,b,w*.27,h,WALL_LIGHT,'none',z=4.1)
        line(ax,[[a+w*.73,b],[a+w*.73,d]],WALL_EDGE,.25,z=4.2)


def furniture_icon(ax,f):
    """Draw recognizable furniture entirely inside its collision footprint."""
    a,b,c,d=f['box'];w,h=c-a,d-b
    if f['kind']=='table':
        # Legs, apron, and a bevelled top replace a flat generic rectangle.
        for x in [a+.13*w,c-.20*w]:
            rect(ax,x,b,.07*w,.38*h,WOOD_SHADE,WOOD_EDGE,.3,z=5)
        rect(ax,a+.05*w,b+.19*h,.90*w,.14*h,WOOD_SHADE,WOOD_EDGE,.35,z=5.2)
        top=np.array([[a,b+.32*h],[c-.06*w,b+.32*h],[c,d],[a+.06*w,d]])
        ax.add_patch(Polygon(top,fc=WOOD_LIGHT,ec=WOOD_EDGE,lw=.5,zorder=6))
        line(ax,[[a,b+.32*h],[c-.06*w,b+.32*h]],WOOD_SHADE,.7,z=6.1)
        for dy in [.56,.77]:
            line(ax,[[a+.11*w,b+dy*h],[c-.13*w,b+dy*h]],WOOD,.22,z=6.1)
    elif f['kind']=='sofa':
        rect(ax,a+.025*w,b,w*.95,h*.92,OBJECT_EDGE,'none',z=5)
        rounded(ax,a,b+.08*h,w,h*.92,'#D5DDE0',OBJECT_EDGE,.5,r=.055,z=5.2)
        rounded(ax,a+.035*w,b+.74*h,.93*w,.23*h,'#BAC6CD',OBJECT_EDGE,.35,r=.04,z=6)
        for x in [a+.10*w,a+.51*w]:
            rounded(ax,x,b+.19*h,.39*w,.50*h,'#E8EDEF',OBJECT_EDGE,.3,r=.035,z=6)
        for x in [a+.02*w,c-.10*w]:
            rounded(ax,x,b+.12*h,.08*w,.62*h,'#C4CFD5',OBJECT_EDGE,.3,r=.025,z=6.2)
    else:
        rect(ax,a,b,w,h,WOOD_SHADE,WOOD_EDGE,.5,z=5)
        rect(ax,a,b+.74*h,w,h*.26,WOOD_LIGHT,WOOD_EDGE,.35,z=6)
        for x in [a+.035*w,a+.515*w]:
            rect(ax,x,b+.07*h,.45*w,.62*h,WOOD,WOOD_EDGE,.3,z=6)
        for x in [a+.45*w,a+.55*w]:
            line(ax,[[x,b+.32*h],[x,b+.49*h]],WOOD_EDGE,.6,z=7)


def draw_room(ax,labels=True,anchors=True,small=False,cycle=None):
    rect(ax,0,0,12,9,BACKGROUND,'none',z=0)
    for room in D['rooms']:
        a,b,c,d=room['box'];task=room['task']
        rect(ax,a,b,c-a,d-b,FLOORS[task],'none',z=0)
        if labels:
            text(ax,a+.3,d-.42,room['name'],4.3 if small else 6.1,
                 CHECKED if cycle is not None and task in cycle['completed'] else TASK_COLORS[task],weight='bold')
    # Architectural grid only: this does not depict the planner's voxel size.
    for x in np.arange(.5,12,.5):line(ax,[[x,.2],[x,8.8]],GRID,.25,z=1)
    for y in np.arange(.5,9,.5):line(ax,[[.2,y],[11.8,y]],GRID,.25,z=1)
    walls=[[0,0,12,.2],[0,8.8,12,9],[0,0,.2,9],[11.8,0,12,9]]+D['walls']
    for bounds in walls:wall_block(ax,bounds)
    for f in D['furniture']:furniture_icon(ax,f)
    if anchors:
        for room in D['rooms']:
            task=room['task'];x,y=room['anchor']
            completed=cycle is not None and task in cycle['completed']
            inactive=cycle is not None and task not in cycle['available'] and not completed
            color=CHECKED if completed or inactive else TASK_COLORS[task]
            badge(ax,x,y,task,color,fill=BACKGROUND,
                  r=.28 if small else .19,size=4.3 if small else 5.5)


def direction(ax,points,color,lw=.7,head=4.2):
    p=np.asarray(points)[:,:2]
    lengths=np.linalg.norm(np.diff(p,axis=0),axis=1);cum=np.r_[0,np.cumsum(lengths)]
    def at(t):
        j=min(len(p)-2,max(0,int(np.searchsorted(cum,t)-1)))
        return p[j]+(p[j+1]-p[j])*(t-cum[j])/max(1e-9,lengths[j])
    mid=.56*cum[-1]
    arrow(ax,at(max(0,mid-.16)),at(min(cum[-1],mid+.16)),color,lw,head,z=12,shrinkA=0,shrinkB=0)


def route(ax,cycle,first_only=False,underlay=True):
    phases=cycle['minco'][:1] if first_only else cycle['minco']
    for i,phase in reversed(list(enumerate(phases))):
        p=np.asarray(phase['points'])[:,:2]
        col=TRAJECTORY
        if underlay:
            line(ax,p,BACKGROUND,2.7,z=8+i*.01)
        line(ax,p,col,1.25,z=9+i*.01)
        direction(ax,p,col)


def draw_views(ax,cycle,small=False):
    chosen={v['candidate_id'] for v in cycle['plan']['visits']}
    for v in D['views']:
        task=v['task'];selected=v['id'] in chosen
        x,y=v['xy'];dx=v['aim'][0]-x;dy=v['aim'][1]-y
        angle=math.atan2(dy,dx)
        reached=(task in cycle['completed'] and
                 np.allclose([x,y],cycle['start'][:2]))
        if reached:
            # The visited pose matches the selected A pose in panel a exactly.
            view(ax,[x,y],angle,CHECKED,BACKGROUND,radius=.52,selected=True,lw=.75)
        elif selected:
            # Stronger wireframes encode selected poses; the vertex is exact.
            view(ax,[x,y],angle,TASK_COLORS[task],TASK_COLORS[task],radius=.52,selected=True,lw=.8)
        else:
            view(ax,[x,y],angle,WALL,BACKGROUND,radius=.43,selected=False,lw=.45)
    x,y=cycle['start'][:2]
    if cycle['completed']:
        ax.add_patch(Circle((x,y),.085,fc=INK,ec=BACKGROUND,lw=.5,zorder=17))
    else:
        drone(ax,cycle['start'])
        text(ax,x-.36,y,'Start',4.8 if small else 5.5,INK,ha='right')


def order_strip(ax,cycle,xs,y):
    for j,(x,v) in enumerate(zip(xs,cycle['plan']['visits'])):
        t=v['task_id'];col=TASK_COLORS[t]
        rounded(ax,x-3,y-2.3,6,4.6,col if j==0 else TASK_PALE[t],col,.55,r=.7)
        text(ax,x,y,t,6.6,'white' if j==0 else col,ha='center',weight='bold')
        if j+1<len(xs):arrow(ax,[x+3.6,y],[xs[j+1]-3.6,y],MUTED,.5,4)


def merged_phases(e):
    """Executed prefix, replaced connection, and current remaining plan.

    Only the first initial visit has executed. The unchanged C-to-B visit
    connection is drawn once, using the updated polynomial, since replanning
    need not reproduce the exact same continuous trajectory.
    """
    initial,updated=e['initial'],e['updated']
    assert [v['task_id'] for v in initial['plan']['visits']]==['A','C','B']
    assert [v['task_id'] for v in updated['plan']['visits']]==['D','C','B']
    assert updated['completed']==['A']
    assert np.allclose(initial['minco'][0]['points'][-1],updated['start'][:3])
    assert initial['plan']['visits'][1]['candidate_id']==updated['plan']['visits'][1]['candidate_id']
    assert initial['plan']['visits'][2]['candidate_id']==updated['plan']['visits'][2]['candidate_id']
    return initial['minco'][0],initial['minco'][1],updated['minco']


def joint_panel(fig,master,e):
    scene=axes(fig,(4,24,98,98*9.1/12.1),(-.05,12.05),(-.05,9.05))
    draw_room(scene,cycle=e['updated'])
    _,previous,_=merged_phases(e)
    # The replaced route is still the saved initial MINCO polynomial.
    # Keep it behind the new plan without offsetting or hand-editing geometry.
    old=np.asarray(previous['points'])[:,:2]
    line(scene,old,PREVIOUS,.95,z=8,ls=(0,(3.0,2.1)))
    route(scene,e['updated'],underlay=False)
    route(scene,e['initial'],first_only=True,underlay=False)
    draw_views(scene,e['updated'])
    drone(scene,e['initial']['start'])
    sx,sy=e['initial']['start'][:2]
    text(scene,sx-.33,sy,'Start',6.3,ha='right')
    text(scene,sx-.26,sy-.35,'Executed',5.7,MUTED,ha='right')

    # Attach the state-change annotation to the reached A observation pose,
    # not to the furniture anchor or to a point elsewhere along the route.
    rounded(scene,6.38,5.65,2.04,1.00,BACKGROUND,NEW_ACTIVE,.6,r=.10,z=18)
    observation=e['state_event'][0]
    outcome_label=observation['outcome'].replace('_',' ')
    text(scene,7.40,6.35,f"A: {outcome_label}",6.4,INK,ha='center',weight='bold')
    arrow(scene,[6.67,5.95],[6.95,5.95],MUTED,.5,4,z=22,shrinkA=0,shrinkB=0)
    text(scene,7.03,5.95,'Activate D',6.1,TASK_COLORS['D'],weight='bold')
    current=np.asarray(e['updated']['start'][:2])
    line(scene,[[8.42,6.13],[9.12,6.13],current],NEW_ACTIVE,.6,z=18)
    text(scene,10.26,2.46,'Newly active',6.3,TASK_COLORS['D'],ha='center',weight='bold')

    # A compact time strip separates the executed prefix from future visits.
    text(master,5,18.7,'Executed',6.2,MUTED)
    text(master,26,18.7,'Start',6.2,ha='center')
    arrow(master,[31.0,18.7],[35.8,18.7],TRAJECTORY,.65,4)
    badge(master,39,18.7,'A',CHECKED,fill=BACKGROUND,r=2.3,size=6.4)
    line(master,[[48,15.0],[48,22.4]],RULE,.6)
    text(master,52,18.7,'Updated plan',6.2,MUTED)
    order_strip(master,e['updated'],[83,99,115],18.7)
    return scene


def first_visit_guide(cycle):
    """Show the exact terminal connector also appended by the route exporter."""
    guide=np.asarray(cycle['plan']['segments'][0]['points_xyz_m'],dtype=float)
    start=np.asarray(cycle['start'][:3],dtype=float)
    pose=cycle['plan']['visits'][0]['pose']
    goal=np.array([pose[k] for k in ('x','y','z')])
    if not np.allclose(guide[0],start,atol=1e-8,rtol=0):
        guide=np.vstack([start,guide])
    if not np.allclose(guide[-1],goal,atol=1e-8,rtol=0):
        guide=np.vstack([guide,goal])
    return guide


def schematic_guide(cycle):
    """Display-only simplification; never used by the planner or MINCO.

    Preserve the actual endpoints and dominant obstacle detour while omitting
    sub-grid stair steps and the small cell-centre-to-goal reversal. The inset
    explicitly says 'schematic'; the saved raw guide remains unchanged.
    """
    def simplify(points,tolerance):
        if len(points)<=2:return points
        delta=points[-1]-points[0]
        fraction=np.clip((points-points[0])@delta/max(delta@delta,1e-12),0,1)
        distances=np.linalg.norm(points-(points[0]+fraction[:,None]*delta),axis=1)
        pivot=int(np.argmax(distances))
        if distances[pivot]<=tolerance:return points[[0,-1]]
        return np.vstack([simplify(points[:pivot+1],tolerance)[:-1],
                          simplify(points[pivot:],tolerance)])
    return simplify(first_visit_guide(cycle),2*D['resolution'])


def trajectory_detail(fig,bounds,cycle):
    """One enlarged map compares the guide and generated trajectory."""
    ax=axes(fig,bounds,(8.5,11.9),(5.3,8.65))
    draw_room(ax,labels=False,anchors=False)
    v=cycle['plan']['visits'][0];p=v['pose']
    guide=schematic_guide(cycle)[:,:2]
    line(ax,guide,CHECKED,1.,z=7.5)
    route(ax,cycle,first_only=True,underlay=False)
    view(ax,[p['x'],p['y']],p['yaw'],TRAJECTORY,TRAJECTORY,radius=.44,selected=True,lw=.75)
    badge(ax,10.,7.4,'A',TRAJECTORY,fill=BACKGROUND,r=.12,size=6.4)
    ax.add_patch(Circle(cycle['start'][:2],.07,fc=INK,ec=BACKGROUND,lw=.4,zorder=18))
    return ax


def minco_panel(fig,master,e):
    text(master,147,95.5,'Start → A',6.8,INK,ha='center',weight='bold')
    detail=trajectory_detail(fig,(115,27,64,64*3.35/3.4),e['initial'])
    line(master,[[115,22.5],[121,22.5]],CHECKED,1.)
    text(master,123,22.5,'A* guide (schematic)',6.0,MUTED)
    line(master,[[151,22.5],[157,22.5]],TRAJECTORY,1.25)
    text(master,159,22.5,'MINCO trajectory',6.0,TRAJECTORY)
    rounded(master,115,5.,64,10.3,HEADER,WALL,.5,r=.8)
    text(master,147,11.9,'MINCO optimization',6.8,TRAJECTORY,ha='center',weight='bold')
    text(master,147,8.2,'within safe corridors',6.2,MUTED,ha='center')
    return detail


def link_trajectory_detail(fig,master,scene,detail):
    """Link the source ROI to the whole (b) panel, including its explanation."""
    xmin,xmax=detail.get_xlim();ymin,ymax=detail.get_ylim()
    scene.add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,
        fc='none',ec='#111111',lw=1.25,linestyle=(0,(3.2,1.6)),zorder=26))
    inset=PANEL_RADIUS*(1-1/math.sqrt(2))
    for y,corner in [(ymin,(111+inset,.8+inset)),
                     (ymax,(111+inset,H-.8-inset))]:
        fig.add_artist(ConnectionPatch(xyA=(xmax,y),xyB=corner,
            coordsA='data',coordsB='data',axesA=scene,axesB=master,
            arrowstyle='-',lw=1.0,color='#111111',linestyle=(0,(3.2,1.6)),
            zorder=40,clip_on=False))


def legend(ax):
    # These encodings belong to panel a. Keep the key inside its frame,
    # below the execution strip, rather than inside the MINCO inset panel.
    line(ax,[[5,13.0],[123,13.0]],RULE,.5)
    for x,label,selected in [(8,'Candidate',False),(37,'Selected',True)]:
        view(ax,[x,9.2],0,TRAJECTORY if selected else WALL,TRAJECTORY if selected else BACKGROUND,
             radius=2.0,selected=selected,apex_radius=.24,lw=.7 if selected else .45)
        text(ax,x+3.2,9.2,label,6.0,MUTED)
    view(ax,[66,9.2],0,CHECKED,BACKGROUND,radius=2.0,selected=True,lw=.7,apex_radius=.24)
    ax.add_patch(Circle((66,9.2),.31,fc=INK,ec=BACKGROUND,lw=.4,zorder=17))
    text(ax,69.2,9.2,'Reached',6.0,MUTED)
    badge(ax,96,9.2,'A',CHECKED,fill=BACKGROUND,r=1.10,size=4.2)
    text(ax,99.2,9.2,'Checked',6.0,MUTED)
    line(ax,[[8,4.5],[15,4.5]],PREVIOUS,.95,z=20,ls=(0,(3.0,2.1)))
    text(ax,18,4.5,'Previous plan',6.0,MUTED)
    line(ax,[[66,4.5],[73,4.5]],TRAJECTORY,1.25,z=20)
    text(ax,76,4.5,'Updated plan',6.0,MUTED)


def render(e):
    vox,_=build_map()
    guide=schematic_guide(e['initial'])
    original=first_visit_guide(e['initial'])
    assert np.allclose(guide[[0,-1]],original[[0,-1]])
    assert all(vox.line_is_valid(a,b) for a,b in zip(guide,guide[1:])), 'Schematic guide must remain collision-free.'
    fig=plt.figure(figsize=(W/25.4,H/25.4),facecolor=BACKGROUND)
    ax=axes(fig,(0,0,W,H),(0,W),(0,H),aspect='auto')
    panels=[(1,104,'Joint task–viewpoint planning'),
            (111,72,'Trajectory generation')]
    for i,(x,w,title) in enumerate(panels):
        rounded(ax,x,.8,w,H-1.6,BACKGROUND,'#111111',.65,r=PANEL_RADIUS,z=-5)
        text(ax,x+3,H-4.6,'ab'[i],9.,'#111111',ha='center',weight='bold')
        text(ax,x+6.3,H-4.6,title,8.0 if i==0 else 7.4,INK,weight='bold')
    text(ax,102,H-4.6,'Length + time + view cost',6.0,TRAJECTORY,ha='right')
    a_labels=axes(fig,(0,0,105,H),(0,127),(0,H),aspect='auto')
    scene=joint_panel(fig,a_labels,e)
    detail=minco_panel(fig,ax,e)
    legend(a_labels)
    link_trajectory_detail(fig,ax,scene,detail)
    # The guide and MINCO share one coordinate system and the exact endpoints.
    fig.canvas.draw()
    trajectory=np.asarray(e['initial']['minco'][0]['points'])
    assert np.allclose(guide[[0,-1],:2],trajectory[[0,-1],:2],atol=1e-8)
    for ext in ['pdf','svg','png']:
        fig.savefig(OUT/f'joint_task_viewpoint_planning_en.{ext}',dpi=500,facecolor=BACKGROUND)
    fig.savefig(OUT/'joint_task_viewpoint_planning_en_preview.png',dpi=220,facecolor=BACKGROUND)
    plt.close(fig)


if __name__=='__main__':
    evidence=build_evidence()
    render(evidence)
    print(OUT/'joint_task_viewpoint_planning_en.pdf')
