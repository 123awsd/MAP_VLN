#!/usr/bin/env python3
"""English-only, sequential explanation of the production viewpoint mechanism.

All illustrations are original vector geometry. Numeric example inputs are
shared with the mechanism audit; no earlier figure or rendering is imported.
"""
from pathlib import Path
from collections import Counter
import hashlib
import inspect
import json
import math
import os
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/viewpoint_sequence_mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle, Polygon, Wedge, FancyArrowPatch, FancyBboxPatch, PathPatch
from matplotlib.path import Path as MplPath

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sys.path.insert(0, str(ROOT / 'PRE_MAP_VLN'))
from stage2.candidate_poses import (
    generate_candidates, _height_candidates_for_view,
    _visible_region_samples, _region_samples, _automatic_observation_profile,
)

D = json.loads((OUT / 'example_inputs.json').read_text())
W, H = 184., 84.
INK = '#26343D'
MUTED = '#687C89'
BLUE = '#2485D2'
BLUE_PALE = '#DCEEFF'
BLUE_LINE = '#AACEEB'
GREEN = '#159D77'
GREEN_PALE = '#D9F4E7'
CORAL = '#F16C50'
FAIL = '#BC4560'
FAIL_PALE = '#F9E4E8'
AMBER = '#C29249'
WOOD = '#F3D9A9'
WOOD_PALE = '#FCF4E7'
GRAY = '#CDD7DE'
RULE = '#D9E1E5'
# Color identifies the four operations; semantic marks retain their colors
# across panels so the region, proposals and retained views remain traceable.
STAGE_COLORS = [
    {'edge': '#24AF88', 'head': '#B4EED3', 'body': '#EFFAF4'},
    {'edge': '#409AE4', 'head': '#BEDFFF', 'body': '#EEF6FE'},
    {'edge': '#DEB137', 'head': '#FFE594', 'body': '#FFFBEA'},
    {'edge': '#F08062', 'head': '#FFC8B7', 'body': '#FFF3ED'},
]
plt.rcParams.update({
    'font.family': 'Liberation Sans', 'font.size': 7,
    'pdf.fonttype': 42, 'svg.fonttype': 'none',
    'hatch.linewidth': .3, 'lines.solid_capstyle': 'round',
    'mathtext.fontset': 'dejavusans',
})


class Scene:
    def __init__(self):
        self.boxes = [(g['name'], np.array(g['lo']), np.array(g['hi'])) for g in D['geometry']]

    def is_state_valid(self, p):
        p = np.asarray(p)
        return all(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0)) >= D['clearance']
                   for _, lo, hi in self.boxes)

    def hit(self, a, b):
        a, b = np.array(a, float), np.array(b, float)
        delta = b - a
        hits = []
        for name, lo, hi in self.boxes:
            left, right = 0., 1.
            for j in range(3):
                if abs(delta[j]) < 1e-12:
                    if not lo[j] <= a[j] <= hi[j]:
                        right = -1.
                        break
                else:
                    ts = ((lo[j] - a[j]) / delta[j], (hi[j] - a[j]) / delta[j])
                    left, right = max(left, min(ts)), min(right, max(ts))
            if left <= right and right > 1e-6 and left < 1 - 1e-6:
                hits.append((max(0., left), name))
        return min(hits, default=None)

    def line_of_sight(self, a, b):
        return self.hit(a, b) is None


def example_row(angle_deg, offset):
    return next(r for r in D['rows'] if abs(r['theta']-math.radians(angle_deg))<1e-7
                and abs(r['distance']-offset)<1e-7)


def angular_window(row):
    """Map code's bearing/elevation tests into a normalized angular-FoV box.

    This is not a pinhole RGB projection: its rectangle exactly represents the
    independent horizontal and vertical angle bounds used by the generator.
    """
    p=np.array(row['xyz'])
    coords=[]
    for sample in D['samples']:
        q=np.array(sample)-p
        angle=math.atan2(q[1],q[0])-row['yaw']
        u=math.atan2(math.sin(angle),math.cos(angle))/math.radians(45.)
        v=math.atan2(q[2],math.hypot(q[0],q[1]))/math.radians(35.)
        coords.append([u,v])
    return np.array(coords)


def height_intervals(row):
    """Audit the automatic, normal-detail height derivation from the code."""
    samples=np.asarray(D['samples'])
    distances=np.linalg.norm(samples[:,:2]-np.asarray(row['xyz'][:2]),axis=1)
    reach=distances*math.tan(math.radians(D['task']['spatial_constraints']['vertical_fov_deg'])/2)
    fov=[float(np.max(samples[:,2]-reach)),float(np.min(samples[:,2]+reach))]
    room=list(D['room']['camera_height_band_m'])
    feasible=[max(fov[0],room[0]),min(fov[1],room[1])]
    assert feasible[0]<=feasible[1]
    region_height=float(np.mean(samples[:,2]))
    selected=float(np.clip(region_height,*feasible))
    return dict(fov=fov,room=room,feasible=feasible,
                region_height=region_height,selected=selected)


def height_fov_limit():
    """The most restrictive sample fixes the upper height at fixed pitch."""
    row=example_row(0,1.4)
    samples=np.asarray(D['samples'])
    xy=np.asarray(row['xyz'][:2])
    distances=np.linalg.norm(samples[:,:2]-xy,axis=1)
    half_angle=math.radians(D['task']['spatial_constraints']['vertical_fov_deg']/2)
    upper_bounds=samples[:,2]+distances*math.tan(half_angle)
    index=int(np.argmin(upper_bounds))
    upper=float(upper_bounds[index])
    angles=np.arctan2(samples[:,2]-upper,distances)
    assert np.all(np.abs(angles)<=half_angle+1e-12)
    assert math.isclose(float(angles[index]),-half_angle,abs_tol=1e-12)
    return dict(xy=xy.tolist(),upper=upper,half_angle=half_angle,
                limiting_index=index,limiting_sample=samples[index].tolist(),
                section_samples=[p.tolist() for p in samples if abs(p[1])<1e-8])


def clearance_section(row):
    """Reflect world Y into a side view of the sofa's back face."""
    back=next(g for g in D['geometry'] if g['name']=='sofa_back')
    xyz=np.asarray(row['xyz'])
    lo,hi=np.asarray(back['lo']),np.asarray(back['hi'])
    assert lo[0]<=xyz[0]<=hi[0] and lo[2]<=xyz[2]<=hi[2]
    distance=float(np.linalg.norm(np.maximum(np.maximum(lo-xyz,xyz-hi),0)))
    return dict(origin=[float(hi[1]-xyz[1]),float(xyz[2])],
                face_y=float(hi[1]),distance=distance,
                margin=distance-D['clearance'])


def validate():
    scene = Scene()
    task, room = D['task'], D['room']
    source_hash = hashlib.sha256(Path(D['source']).read_bytes()).hexdigest()
    profile = _automatic_observation_profile([1.36,.92,.9], 'below_region', 'normal', 90.)
    offsets = sorted(set(profile))
    assert np.allclose(offsets, D['distances'], atol=1e-10, rtol=0)
    assert np.allclose(np.array(offsets)+.5*math.hypot(1.36,.92),D['radii'],atol=1e-10,rtol=0)
    selected = generate_candidates(scene, {'rooms': [room]}, task, max_candidates=8)
    best = generate_candidates(scene, {'rooms': [room]}, task, max_candidates=1000)
    assert selected == D['selected'] and best == D['best_per_azimuth']
    assert _region_samples([0, 0, .45], [1.36, .92, .9], 0, 'below_region') == D['samples']
    for row in D['rows']:
        xyz = row['xyz']
        radius=.5*math.hypot(1.36,.92)+row['distance']
        assert np.allclose(xyz[:2],radius*np.array([math.cos(row['theta']),math.sin(row['theta'])]),atol=1e-10,rtol=0)
        yaw=math.atan2(-xyz[1],-xyz[0])
        assert abs(math.atan2(math.sin(row['yaw']-yaw),math.cos(row['yaw']-yaw)))<1e-10
        heights = _height_candidates_for_view(xyz[:2], D['samples'], task['spatial_constraints'],
                                             70., room['camera_height_band_m'])
        visible = []
        if not heights:
            status = 'height'
        elif not scene.is_state_valid([*xyz[:2], heights[0]]):
            status = 'collision'
        else:
            assert abs(xyz[2] - heights[0]) < 1e-8
            visible = _visible_region_samples(scene, xyz[:2], heights[0], row['yaw'], D['samples'], 90., 70.)
            status = 'valid' if visible else 'occlusion'
        assert status == row['status'] and visible == row['visible']
    counts=Counter(r['status'] for r in D['rows'])
    assert counts=={'height':32,'collision':3,'occlusion':1,'valid':36}
    assert (len(D['rows']),len(best),len(selected))==(72,23,8)
    pairs={
        'field_of_view':(example_row(0,1.4),example_row(0,.35)),
        'clearance':(example_row(60,1.4),example_row(45,1.4)),
        'line_of_sight':(example_row(90,1.4),example_row(90,.6)),
    }
    assert np.all(np.abs(angular_window(pairs['field_of_view'][0]))<=1)
    outside=np.any(np.abs(angular_window(pairs['field_of_view'][1]))>1,axis=1)
    assert np.count_nonzero(outside)==3
    assert scene.is_state_valid(pairs['clearance'][0]['xyz'])
    assert not scene.is_state_valid(pairs['clearance'][1]['xyz'])
    clearance_details=[]
    for row,passed in zip(pairs['clearance'],[True,False]):
        section=clearance_section(row)
        assert math.isclose(section['distance'],max(0.,-section['origin'][0]),abs_tol=1e-12)
        assert (section['margin']>=0)==passed
        nearest=min(float(np.linalg.norm(np.maximum(np.maximum(lo-row['xyz'],
                    np.asarray(row['xyz'])-hi),0))) for _,lo,hi in scene.boxes)
        assert math.isclose(section['distance'],nearest,abs_tol=1e-12)
        clearance_details.append(section)
    assert scene.line_of_sight(pairs['line_of_sight'][0]['xyz'],D['samples'][4])
    assert not scene.line_of_sight(pairs['line_of_sight'][1]['xyz'],D['samples'][4])
    for row,expected in zip(pairs['line_of_sight'],[True,False]):
        origin,full,visible=visibility_fan(scene,row)
        assert MplPath(np.vstack([origin,full])).contains_point([0,.08])
        assert bool(MplPath(np.vstack([origin,visible])).contains_point([0,.08]))==expected
        assert np.isfinite(full).all() and np.isfinite(visible).all()
    height_row=example_row(0,1.4)
    height_detail=height_intervals(height_row)
    generated_heights=_height_candidates_for_view(
        height_row['xyz'][:2],D['samples'],task['spatial_constraints'],
        task['spatial_constraints']['vertical_fov_deg'],room['camera_height_band_m'])
    assert np.allclose(generated_heights,[height_detail['selected']],atol=1e-6,rtol=0)
    height_limit=height_fov_limit()
    assert math.isclose(height_limit['upper'],height_detail['fov'][1],abs_tol=1e-12)
    # The inset shows only the above-ground FoV-compatible band. Both its
    # endpoints satisfy every sample's vertical angular constraint.
    samples=np.asarray(D['samples'])
    distances=np.linalg.norm(samples[:,:2]-np.asarray(height_limit['xy']),axis=1)
    for height in [0.,height_limit['upper']]:
        assert np.all(np.abs(np.arctan2(samples[:,2]-height,distances))<=height_limit['half_angle']+1e-12)
    assert _height_candidates_for_view(
        height_row['xyz'][:2],D['samples'],task['spatial_constraints'],
        task['spatial_constraints']['vertical_fov_deg'],
        [height_limit['upper']+.01,height_limit['upper']+.10])==[]
    assert hashlib.sha256(Path(D['source']).read_bytes()).hexdigest()==source_hash
    evidence = dict(D)
    evidence['validation'] = '72 proposals, all visible sample IDs, 23 azimuth winners and 8 retained poses match the production generator.'
    evidence['source_sha256'] = source_hash
    evidence['figure_scope'] = 'English-only abstract mechanism: task to region, proposals, feasibility rejection examples, azimuth pruning and angular diversity.'
    evidence['presentation_note'] = 'Figure labels omit numerical sampling settings, counts, ratios and budget values. Points and FoV sectors illustrate the operations, not prescribed cardinalities or experimental results. Numeric records are retained only as reproducible internal evidence for mechanism consistency.'
    evidence['height_detail'] = height_detail
    evidence['height_detail_note'] = 'Internal audit only: the production automatic normal-detail branch intersects the fixed-pitch FoV height interval with room bounds and clamps mean region height to the result. Room bounds and selection-interval mechanics are intentionally omitted from the paper illustration.'
    evidence['height_fov_limit'] = height_limit
    evidence['height_illustration_note'] = 'The side-view height construction shows the upper vertical-FoV-compatible camera height at a fixed horizontal position and level pitch. The section is reflected horizontally to place the drone on the left and region samples on the right; distances, elevations and angles are unchanged. Open upper/lower boundary rays depict the FoV without a curved cap. The lower edge exactly touches the circled limiting sample; all other samples satisfy the vertical angular test at this height. A bracket on the left shows the above-ground portion of the FoV-compatible interval. The camera at the upper bound illustrates the limiting geometry, not the production-selected height. The below-ground part of the geometric interval, room constraints and obstacle tests are omitted from this detail. The drone uses a strict side elevation with a shallow gray shell, slender dark arms and tapered propellers: overlapping rotor pairs share two silhouettes; the camera window is integrated into the nose, with its flush lens at the exact optical origin. Drone dimensions and upper-ray display length are illustrative.'
    evidence['audit_scope'] = {
        'branch':'below_region, automatic distance/height, normal detail, one mapped anchor, budget 8',
        'geometry_backend':'Illustrative analytic AABB solids and exact segment intersections, not the production Habitat/FALCON voxel map.',
        'checks':['Automatic distance profile','72 exact XY coordinates and target-facing yaws',
                  'Height, collision and all visible sample IDs','23 full per-azimuth outputs',
                  '8 complete retained pose records','Height-interval overlap and nominal selection',
                  'Limiting sample and vertical FoV height bound',
                  'Unchanged production source'],
        'rejection_counts':dict(counts),
        'production_functions':{
            fn.__name__:{'file':str(Path(inspect.getsourcefile(fn)).resolve()),
                         'line':inspect.getsourcelines(fn)[1]}
            for fn in [_region_samples,_automatic_observation_profile,
                       _height_candidates_for_view,_visible_region_samples,generate_candidates]
        },
    }
    evidence['filter_comparisons']={
        name:{'pass':good,'reject':bad} for name,(good,bad) in pairs.items()
    }
    evidence['clearance_sections']=clearance_details
    evidence['clearance_illustration_note']='Two matched side views replace the plan-view safety circles. Sofa back and base use their actual YZ extents; each camera origin is the reflected Y coordinate and exact height of the audited pose. The clear example has an obstacle distance exceeding the clearance requirement. The rejected example lies inside the sofa back, with the occupied area around the rejected pose highlighted in raspberry. Drone silhouettes locate the poses; their dimensions and depicted headings are illustrative and do not define a mesh collision test. No path or visibility cone is drawn in this positional check.'
    evidence['fov_cone_note']='Schematic projected frusta: exact normalized bearing/elevation sample coordinates on a common display plane. At z=0.65 m, all 9 samples are inside for the far proposal and 3 are outside for the near proposal. The display plane is not a metric range limit.'
    evidence['occlusion_cone_note']='Metric vertical sections through x=0, fixed level pitch, +/-35 degree FoV. Filled visible fans stop at first analytic 3D solid intersections; hatched remainders are shadows. Both paired central target rays are checked against the same production-generator scene interface.'
    evidence['pose_marker_note']='Small solid dots mark exact viewpoint positions. Short circular FoV sectors open along the computed yaw and span 90 degrees in plan. Display radii are illustrative, not sensing-range limits. Projected and occlusion cones use only an optical-center dot, with no additional direction icon. The clearance pair uses side-view drone silhouettes to locate the actual projected poses; their body dimensions and heading are illustrative. The height inset uses the same drone glyph with its camera at the exact optical origin.'
    (OUT / 'mechanism_evidence.json').write_text(json.dumps(evidence, indent=2) + '\n')
    return scene


def text(ax, x, y, s, size=7, color=INK, ha='left', weight='normal', **kw):
    kw.setdefault('zorder', 30)
    return ax.text(x, y, s, fontsize=size, color=color, ha=ha, va='center', weight=weight, **kw)


def line(ax, points, color=INK, lw=.65, z=3, **kw):
    p = np.array(points)
    return ax.plot(p[:, 0], p[:, 1], color=color, lw=lw, zorder=z, **kw)


def rect(ax, x, y, w, h, fc='none', ec=INK, lw=.6, z=2, **kw):
    p = Rectangle((x, y), w, h, fc=fc, ec=ec, lw=lw, zorder=z, **kw)
    ax.add_patch(p)
    return p


def dot(ax, xy, color=BLUE, radius=.035, filled=True, edge=None, lw=.6, z=10):
    ax.add_patch(Circle(xy, radius, fc=color if filled else 'white', ec=edge or color, lw=lw, zorder=z))


def cross(ax, xy, radius=.04, color=INK, lw=.9, z=15):
    x, y = xy
    line(ax, [(x-radius, y-radius), (x+radius, y+radius)], color, lw, z)
    line(ax, [(x-radius, y+radius), (x+radius, y-radius)], color, lw, z)


def check(ax,xy,radius=.05,color=BLUE,lw=.8,z=18):
    x,y=xy
    line(ax,[(x-radius,y),(x-.22*radius,y-.65*radius),(x+radius,y+.85*radius)],color,lw,z)


def outcome(ax,x,y,label,passed):
    col=BLUE if passed else FAIL
    if passed:check(ax,[x-4.3,y],radius=.65,color=col,lw=.8,z=30)
    else:cross(ax,[x-4.3,y],radius=.53,color=col,lw=.8,z=30)
    text(ax,x+.4,y,label,5.8,col,ha='center')


def arrow(ax, a, b, color=INK, lw=.65, head=6, z=10, **kw):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=head,
                               lw=lw, color=color, zorder=z, **kw))


def axes(fig, box, xlim, ylim, aspect='equal'):
    x, y, w, h = box
    ax = fig.add_axes([x/W, y/H, w/W, h/H])
    ax.set(xlim=xlim, ylim=ylim, aspect=aspect)
    ax.axis('off')
    return ax


def plane(ax, bounds=(-.544, -.368, 1.088, .736), dots=True, z=6):
    rect(ax, *bounds, GREEN_PALE, GREEN, .65, z=z)
    if dots:
        for p in D['samples']:
            dot(ax, p[:2], GREEN, .038, lw=.3, z=z+1)


def table_plan(ax, light=False):
    rect(ax, -.68, -.46, 1.36, .92, WOOD_PALE, AMBER, .6, z=3)
    for x in [-.57, .57]:
        for y in [-.35, .35]:
            rect(ax, x-.045, y-.045, .09, .09, WOOD, AMBER, .45, z=8)
    plane(ax)


def view_sector(ax,xy,yaw,radius,half_angle,color,fill,z=5,lw=.55,point_radius=.03,edge=None):
    """Position + opening FoV, without a separate heading arrow.

    The circular cap is a compact display boundary, not a sensor range limit.
    Plan sectors preserve the horizontal angular test; the side inset uses
    the vertical angular test. Both are centered on the actual pose.
    """
    angle=math.degrees(yaw)
    ax.add_patch(Wedge(xy,radius,angle-half_angle,angle+half_angle,
                       fc=fill,ec=edge or color,lw=lw,joinstyle='round',zorder=z))
    dot(ax,xy,color,point_radius,edge='white',lw=.35,z=z+2)


def projected_frustum(ax,row,passed):
    """Compact 3D cone with the code's angular sample coordinates on its face.

    The far rectangle is an illustrative depth cap. Samples on the face
    preserve exact normalized angular membership, not metric scene distances.
    """
    col=BLUE if passed else FAIL
    fill=BLUE_PALE if passed else FAIL_PALE
    def project(depth,u,v):
        return np.array([1.15*depth+.45*u,.24*u+.70*v])
    boundary=[(-1,-1),(1,-1),(1,1),(-1,1)]
    far=np.array([project(1,u,v) for u,v in boundary])
    origin=np.array([0.,0.])
    # Broad translucent surfaces show the sensing volume rather than a ray.
    for i in [0,3,1,2]:
        j=(i+1)%4
        ax.add_patch(Polygon([origin,far[j],far[i]],
                            fc=fill,ec=col,lw=.45,alpha=.65,zorder=2))
    ax.add_patch(Polygon(far,fc='white',ec=col,lw=.75,alpha=.80,zorder=3))
    # The cone apex itself identifies position; the volume supplies direction.
    dot(ax,origin,col,.035,edge='white',lw=.4,z=8)
    for u,v in angular_window(row):
        p=project(1,u,v)
        dot(ax,p,GREEN,.030,lw=.25,z=8)
        if abs(u)>1 or abs(v)>1:
            ax.add_patch(Circle(p,.047,fc='none',ec=FAIL,lw=.55,zorder=9))
    return far


def visibility_fan(scene,row,x_stop=.20,z_top=1.15):
    """First-hit free cone and its full footprint in the x_world=0 section."""
    xyz=np.array(row['xyz']);origin=np.array([-xyz[1],xyz[2]])
    assert abs(xyz[0])<1e-8
    angles=list(np.linspace(-math.radians(35),math.radians(35),361))
    # Include exact section-corner bearings so narrow free wedges stay sharp.
    for _,lo,hi in scene.boxes:
        if lo[0]<=0<=hi[0]:
            for s in [-hi[1],-lo[1]]:
                for z in [lo[2],hi[2]]:
                    angle=math.atan2(z-origin[1],s-origin[0])
                    if abs(angle)<math.radians(35):
                        angles.extend([angle-1e-7,angle,angle+1e-7])
    angles=sorted(set(angles))
    full=[];visible=[]
    for theta in angles:
        u=np.array([math.cos(theta),math.sin(theta)])
        length=(x_stop-origin[0])/u[0]
        if u[1]>1e-12:length=min(length,(z_top-origin[1])/u[1])
        elif u[1]<-1e-12:length=min(length,-origin[1]/u[1])
        end=origin+length*u
        hit=scene.hit(xyz,[0,-end[0],end[1]])
        full.append(end)
        visible.append(origin+(end-origin)*(hit[0] if hit is not None else 1))
    return origin,np.array(full),np.array(visible)


def occlusion_cone(ax,scene,row,passed):
    col=BLUE if passed else FAIL
    fill=BLUE_PALE if passed else FAIL_PALE
    p,full,visible=visibility_fan(scene,row)
    # Gray hatching remains only where an actual first-hit obstacle casts a
    # shadow. No isolated target ray is needed to carry the explanation.
    ax.add_patch(Polygon(np.vstack([p,full]),fc='#E6E8E7',ec='#BBC2C6',
                         hatch='////',lw=.35,zorder=0))
    ax.add_patch(Polygon(np.vstack([p,visible]),fc=fill,ec=col,lw=.6,zorder=2))
    for name,lo,hi in scene.boxes:
        if lo[0]<=0<=hi[0] and name in {'chair_back_seat','chair_back_back','table_top'}:
            rect(ax,-hi[1],lo[2],hi[1]-lo[1],hi[2]-lo[2],WOOD,AMBER,.55,z=5)
    dot(ax,p,col,.043,edge='white',lw=.4,z=8)
    # The same region-center point is lit for the far view, shadowed for near.
    dot(ax,[0,.08],GREEN,.043,edge='white',lw=.6,z=10)
    if not passed:ax.add_patch(Circle((0,.08),.063,fc='none',ec=FAIL,lw=.65,zorder=11))
    line(ax,[[p[0]-.10,0],[.20,0]],RULE,.45,6)
    text(ax,.12,.22,'R',5.2,GREEN,ha='center')


def iso(p):
    x, y, z = p
    return np.array([.9*x-.54*y, .27*x+.36*y+z])


def poly3(ax, vertices, fc, ec=AMBER, lw=.6, z=3, **kw):
    p = Polygon([iso(v) for v in vertices], closed=True, fc=fc, ec=ec, lw=lw, zorder=z, **kw)
    ax.add_patch(p)
    return p


def cuboid(ax, lo, hi, z=3):
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    poly3(ax, [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)], '#E9C78E', z=z)
    poly3(ax, [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)], '#F5DEB5', z=z)
    poly3(ax, [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)], WOOD_PALE, z=z+1)


def region_stage(fig, master):
    # The instruction is the input; no unknown backpack is placed in the map.
    rect(master, 2, 62, 37, 11, '#DCF5E8', 'none', z=0)
    text(master, 20.5, 69.6, 'Find a backpack', 7.3, ha='center')
    text(master, 20.5, 65.4, 'under the table', 7.3, GREEN, ha='center', weight='bold')
    arrow(master, (20.5, 61.3), (20.5, 57.6), MUTED, .6)
    ax = axes(fig, (2, 26, 37, 32), (-1.03, 1.03), (-.36, 1.42))
    # An axonometric table separates the mapped anchor from the ground region.
    for x, y in [(-.57, .35), (.57, .35)]:
        cuboid(ax, [x-.045,y-.045,0], [x+.045,y+.045,.8], z=1)
    corners = [(-.544,-.368,.08),(.544,-.368,.08),(.544,.368,.08),(-.544,.368,.08)]
    poly3(ax, corners, GREEN_PALE, GREEN, .75, z=3)
    for p in D['samples']:
        dot(ax, iso(p), GREEN, .026, lw=.35, z=4)
    for x, y in [(-.57, -.35), (.57, -.35)]:
        cuboid(ax, [x-.045,y-.045,0], [x+.045,y+.045,.8], z=5)
    cuboid(ax, [-.68,-.46,.8], [.68,.46,.9], z=8)
    p=iso([.50,.32,.90])
    line(ax,[p,[.71,1.29],[.95,1.29]],AMBER,.5,11)
    text(ax,.95,1.40,'Mapped table',5.9,AMBER,ha='right',zorder=12)
    p=iso([.45,-.24,.08])
    line(ax,[p,[.69,-.24],[.98,-.24]],GREEN,.55,11)
    text(ax,.98,-.36,'Search region',6.1,GREEN,ha='right',zorder=12)
    # Repeat the same nine samples as the discrete output of this step.
    arrow(master, (20.5, 25), (20.5, 21.6), GREEN, .6)
    grid=axes(fig,(7,7.5,20,12),(-.76,.76),(-.46,.46))
    plane(grid)
    text(master,32,13.2,'Region\nsamples',6.3,GREEN,ha='center',linespacing=1.2)
    text(master,20.5,3.7,'Task-defined region',7.0,'#137555',ha='center',weight='bold')


def drone_side(ax,camera,scale=1.,col='#43515B',fill='#C4CFD6'):
    """Side-view pose glyph; the inset camera is its optical origin."""
    # Strict side elevation: overlapping front/rear rotor pairs share two
    # silhouettes. No perspective or four-rotor plan icon is mixed into it.
    camera=np.asarray(camera)
    stroke=math.sqrt(scale)
    def body_pts(points):
        return scale*np.asarray(points)+camera
    # Low motor mounts and slender arms keep the fuselage visually light.
    for xx,attach in [(-.50,-.31),(.035,-.11)]:
        line(ax,body_pts([[attach,.045],[xx,.105]]),col,1.0*stroke,11)
        rect(ax,*body_pts([xx-.022,.095]),.044*scale,.068*scale,col,col,.25*stroke,z=12)
        line(ax,body_pts([[xx,.16],[xx,.19]]),col,.45*stroke,13)
        ax.add_patch(Polygon(body_pts([[xx-.16,.19],[xx-.045,.201],
                                       [xx+.16,.19],[xx+.045,.179]]),
                             fc=col,ec=col,lw=.25*stroke,zorder=13))
    # A shallow curved shell replaces the heavy angular block. The shell
    # stays behind the lens, so the view originates at the nose.
    shell=body_pts([[-.405,.015],[-.39,.055],[-.32,.085],[-.24,.085],
                    [-.14,.085],[-.085,.060],[-.025,.045],
                    [0,.027],[0,-.027],[-.12,-.038],[-.28,-.043],
                    [-.36,-.025],[-.405,-.015],[-.405,.015],[-.405,.015]])
    codes=[MplPath.MOVETO,*([MplPath.CURVE4]*6),MplPath.LINETO,MplPath.LINETO,
           *([MplPath.CURVE4]*3),MplPath.CURVE3,MplPath.CURVE3,MplPath.CLOSEPOLY]
    ax.add_patch(PathPatch(MplPath(shell,codes),fc=fill,ec=col,lw=.55*stroke,zorder=14))
    # The camera window is inset into the nose. Its flush lens remains at
    # the exact optical origin shared by both FoV boundaries.
    rect(ax,*body_pts([-.062,-.022]),.062*scale,.048*scale,col,col,.3*stroke,z=16)
    line(ax,body_pts([[-.008,-.012],[-.008,.016]]),'#E7F3FC',.55*stroke,17)


def draw_height_fov(fig,master):
    """Construct the height bound from the sample touching the FoV edge."""
    demo=height_fov_limit()
    text(master,65.5,30.,'Determine camera height',6.3,MUTED,ha='center')
    ax=axes(fig,(44.5,7.7,42,21.2),(-1.03,3.03),(-.20,1.78))
    camera=np.array([0.,demo['upper']])
    # Reflect the side section so the viewing direction follows left-to-right
    # reading. Horizontal distances, elevation and FoV angles are unchanged.
    sample=np.array([demo['xy'][0]-demo['limiting_sample'][0],demo['limiting_sample'][2]])
    region=np.asarray(demo['section_samples'])[:,[0,2]]
    region[:,0]=demo['xy'][0]-region[:,0]
    # The pale fan links all samples to the same camera. Its outermost ray is
    # the controlling FoV edge; the other rays stay inside that angular bound.
    ax.add_patch(Polygon([region[0],camera,region[-1]],fc='#E0EFFB',ec='none',zorder=1))
    for p in region:
        line(ax,[p,camera],BLUE_LINE,.4,2)
    # Open rays expose the FoV angle without a curved cap or closed glyph.
    # The lower boundary continues to the controlling region sample.
    upper_end=camera+.78*np.array([math.cos(demo['half_angle']),
                                    math.sin(demo['half_angle'])])
    lower_end=camera+.78*np.array([math.cos(demo['half_angle']),
                                    -math.sin(demo['half_angle'])])
    line(ax,[camera,upper_end],BLUE,.75,5)
    line(ax,[camera,lower_end],BLUE,.75,5)
    line(ax,[lower_end,sample],BLUE,.6,5)
    line(ax,[camera,[1.04,camera[1]]],MUTED,.45,5,ls=(0,(3,2)))
    text(ax,1.03,camera[1]+.15,'Fixed pitch',5.1,MUTED)
    text(ax,.97,.45,'FoV edge',5.1,BLUE,ha='center',rotation=-math.degrees(demo['half_angle']),
         bbox={'facecolor':STAGE_COLORS[1]['body'],'edgecolor':'none','pad':.5})
    drone_side(ax,camera)
    # Same region samples as the main plan, shown in the central side section.
    line(ax,[[-.86,0],[2.98,0]],RULE,.5,2)
    line(ax,[region[0],region[-1]],GREEN,1.,8)
    for p in region:
        dot(ax,p,GREEN,.044,edge='white',lw=.45,z=9)
    ax.add_patch(Circle(sample,.077,fc='none',ec=BLUE,lw=.65,zorder=10))
    text(ax,2.21,-.14,'Region samples',5.1,GREEN,ha='center')
    # The bracket spans the above-ground part of the FoV-only interval.
    # The pose at its upper end illustrates the limiting case, not selection.
    band_x=-.84
    line(ax,[[band_x+.10,0],[band_x,0],[band_x,camera[1]],[band_x+.10,camera[1]]],BLUE,.65,7)
    line(ax,[[band_x,camera[1]],camera],BLUE_LINE,.45,8,ls=(0,(2,2)))
    text(ax,-.32,-.14,'FoV height range',5.0,BLUE,ha='center')


def proposals_stage(fig, master):
    text(master,65.5,70.6,'Sample distances & angles',6.5,MUTED,ha='center')
    ax=axes(fig,(44,33,43,35),(-2.42,2.42),(-2.42,2.42))
    table_plan(ax)
    for radius in D['radii']:
        ax.add_patch(Circle((0,0),radius,fc='none',ec=BLUE_LINE,lw=.6,zorder=1))
    # All proposals are shown before geometric rejection. FoV sectors open
    # inward; their tiny apex dots encode the exact sampled positions.
    for row in D['rows']:
        view_sector(ax,row['xyz'][:2],row['yaw'],.15,45,BLUE,'#DAECFA',
                    lw=.34,point_radius=.026,edge='#84B7E0')
    text(ax,0,-1.76,'Face region',5.7,BLUE,ha='center',
         bbox={'facecolor':STAGE_COLORS[1]['body'],'edgecolor':'none','pad':.4},zorder=20)
    # Rings communicate multiple distances without enumerating settings.
    draw_height_fov(fig,master)
    text(master,65.5,3.7,'Candidate views',7.0,'#21679E',ha='center',weight='bold')


def clearance_side(ax,row,passed):
    """A visible gap or occupied pose, using the same metric side projection."""
    detail=clearance_section(row)
    camera=np.asarray(detail['origin'])
    col=BLUE if passed else FAIL
    fill=BLUE_PALE if passed else FAIL_PALE
    for name in ['sofa_base','sofa_back']:
        g=next(g for g in D['geometry'] if g['name']==name)
        lo,hi=np.asarray(g['lo']),np.asarray(g['hi'])
        x=detail['face_y']-hi[1]
        rect(ax,x,lo[2],hi[1]-lo[1],hi[2]-lo[2],
             WOOD_PALE if name=='sofa_base' else WOOD,AMBER,.6,z=3)
    # Upholstery seams identify the sofa without changing its occupied outline.
    line(ax,[[.22,.40],[.71,.40],[.71,.21]],AMBER,.35,4)
    line(ax,[[.045,.47],[.045,.96],[.145,.96]],AMBER,.35,4)
    if passed:
        # The small dimension line spans the actual pose-to-obstacle gap.
        y=camera[1]-.10
        line(ax,[[camera[0],y],[0,y]],BLUE,.6,6)
        for x in [camera[0],0]:
            line(ax,[[x,y-.025],[x,y+.025]],BLUE,.6,6)
    drone_side(ax,camera,scale=.50,col=col,fill=fill)
    if not passed:
        # Highlight occupied material around the rejected pose. Draw the
        # drone above the highlight so its intrusion remains easy to see.
        rect(ax,0,camera[1]-.055,.19,.19,FAIL_PALE,FAIL,0,z=5,
             hatch='////',alpha=.65)
        line(ax,[[0,camera[1]-.055],[0,camera[1]+.135]],FAIL,.8,6)




def reject_stage(fig,master,scene):
    # Every row pairs passing and failing examples from the original proposals.
    # Blue + check = passes this condition; raspberry + cross = fails it.
    for y,title in [(70.6,'Fits in view'),(47.4,'Avoids obstacles'),(27.6,'Sees the region')]:
        text(master,94,y,title,6.6,INK,weight='bold')
    text(master,131,27.6,'Any sample',5.1,MUTED,ha='right')
    # Projected frusta make the sensing volume explicit. Angular samples on
    # their far display faces preserve all previous inside/outside decisions.
    for left,passed,row in [(94,True,example_row(0,1.4)),(114,False,example_row(0,.35))]:
        ax=axes(fig,(left,54.9,17,13.8),(-.12,1.85),(-1.13,1.03))
        projected_frustum(ax,row,passed)
    outcome(master,102.5,53.0,'Inside',True)
    outcome(master,122.5,53.0,'Outside',False)
    text(master,112.5,50.5,'At minimum camera height',5.2,MUTED,ha='center')
    line(master,[(94,49.1),(131,49.1)],'#E9DCA8',.45,1)
    # Matched side views make the positional rejection visible without
    # asking the reader to interpret isolated safety-radius circles.
    for left,passed,row in [(94,True,example_row(60,1.4)),(114,False,example_row(45,1.4))]:
        collision=axes(fig,(left,31.1,17,14.5),(-.62,.84),(.03,1.11))
        clearance_side(collision,row,passed)
    outcome(master,102.5,30.1,'Clear',True)
    cross(master,[117.4,30.1],radius=.53,color=FAIL,lw=.9,z=30)
    text(master,122.9,30.1,'Collision',5.8,FAIL,ha='center')
    line(master,[(94,28.8),(131,28.8)],'#E9DCA8',.45,1)
    # Two distinct filled cones avoid overlapping sight lines. Their colors
    # stop at the first solid; hatched shadows show why the region is hidden.
    for left,passed,row in [(94,True,example_row(90,1.4)),(114,False,example_row(90,.6))]:
        start=-row['xyz'][1]
        los=axes(fig,(left,10.3,17,15.2),(start-.18,.24),(-.08,1.23))
        occlusion_cone(los,scene,row,passed)
    text(master,112.5,10.2,'Hatched: occluded',5.1,MUTED,ha='center')
    outcome(master,102.5,7.8,'Clear',True)
    outcome(master,122.5,7.8,'Blocked',False)
    text(master,112.5,3.7,'Feasible views',7.0,'#21679E',ha='center',weight='bold')


def mini_visibility(master,x,y,ids):
    # Same 3-by-3 sample indexing as the production region, shown in plan.
    for index,p in enumerate(D['samples']):
        point=[x+p[0]*3.4,y+p[1]*3.4]
        dot(master,point,GREEN if index in ids else GRAY,.32,filled=True,lw=.25,z=10)


def select_stage(fig,master):
    text(master,159.5,70.6,'Best view per angle',6.5,MUTED,ha='center')
    near=next(r for r in D['rows'] if abs(r['theta']-math.pi/3)<1e-7 and abs(r['distance']-.6)<1e-7)
    far=next(r for r in D['rows'] if abs(r['theta']-math.pi/3)<1e-7 and abs(r['distance']-1.4)<1e-7)
    assert len(near['visible'])==2 and len(far['visible'])==3
    # A rotated schematic of the same 60-degree ray: only distance changes.
    rect(master,138.5,58.7,4.2,4.2,GREEN_PALE,GREEN,.5,z=3)
    text(master,140.6,60.8,'R',6.5,GREEN,ha='center')
    line(master,[(143,60.8),(180.5,60.8)],BLUE_LINE,.6,2)
    for x,row in [(154,near),(175,far)]:
        mini_visibility(master,x,66.4,row['visible'])
        text(master,x,63.6,'More visible' if row is far else 'Fewer visible',5.3,GREEN,ha='center')
        view_sector(master,[x,60.8],math.pi,2.7,45,BLUE,
                    '#C9E5FA' if row is far else '#E6F2FC',
                    z=10,lw=.6,point_radius=.36)
    # A compact check distinguishes the per-angle winner from the nearer view.
    line(master,[(177,60.5),(177.6,59.9),(178.8,61.5)],BLUE,.8,20)
    text(master,159.5,55.3,'Visibility first · then cost',5.9,MUTED,ha='center')
    arrow(master,(159.5,53.5),(159.5,49.8),INK,.65)
    text(master,159.5,47.8,'Select a diverse subset',6.2,CORAL,ha='center')
    # Actual output positions; ghost marks show the other azimuth winners.
    ax=axes(fig,(138,7,43,39),(-2.42,2.42),(-2.42,2.42))
    table_plan(ax)
    for r in [D['radii'][1],D['radii'][2]]:
        ax.add_patch(Circle((0,0),r,fc='none',ec=RULE,lw=.45,zorder=1))
    for c in D['best_per_azimuth']:
        p=c['pose']
        dot(ax,[p['x'],p['y']],GRAY,.025,lw=.2,z=5)
    for c in D['selected']:
        p=c['pose'];xy=np.array([p['x'],p['y']])
        # Compact plan sections show the 90-degree horizontal field of view.
        # Their short display depth is not a maximum camera sensing range.
        view_sector(ax,xy,p['yaw'],.45,45,CORAL,'#FBD4C7',z=9,lw=.6,point_radius=.050)
    text(ax,0,-.75,'Angular spread',5.7,CORAL,ha='center',zorder=20,
         bbox={'facecolor':STAGE_COLORS[3]['body'],'edgecolor':'none','pad':.5})
    text(master,159.5,3.7,'Retained views',7.0,'#AC3F2C',ha='center',weight='bold')


def render(scene):
    fig=plt.figure(figsize=(W/25.4,H/25.4),facecolor='white')
    master=axes(fig,(0,0,W,H),(0,W),(0,H),aspect='auto')
    # The only dominant reading order is left to right. Lettered panels carry
    # the mechanism; figures demonstrate each operation immediately below.
    stages=[(2,39,'Define region'),(44,87,'Propose views'),(94,131,'Reject invalid'),(138,181,'Spread views')]
    for i,(a,b,title) in enumerate(stages,1):
        palette=STAGE_COLORS[i-1]
        panel=FancyBboxPatch((a-1.1,.8),b-a+2.2,81.5,
            boxstyle='round,pad=0,rounding_size=1.5',
            fc=palette['body'],ec=palette['edge'],lw=.9,zorder=-5)
        master.add_patch(panel)
        # Clip flat header/footer fills into the rounded vector panel outline.
        header=rect(master,a-1.1,74.7,b-a+2.2,7.6,palette['head'],'none',z=-4)
        header.set_clip_path(panel)
        footer=rect(master,a-1.1,.8,b-a+2.2,5.6,palette['head'],'none',z=-4)
        footer.set_clip_path(panel)
        line(master,[(a-1.1,74.7),(b+1.1,74.7)],palette['edge'],.55,1)
        step_color=[GREEN,BLUE,'#E8B82C',CORAL][i-1]
        dot(master,[a+1.8,78.5],step_color,1.8,lw=.4)
        text(master,a+1.8,78.5,'abcd'[i-1],6.7,INK if i==3 else 'white',ha='center',weight='bold')
        text(master,a+5.1,78.5,title,7.5,INK,weight='bold')
    region_stage(fig,master)
    proposals_stage(fig,master)
    reject_stage(fig,master,scene)
    select_stage(fig,master)
    for ext in ['pdf','svg','png']:
        fig.savefig(OUT/f'viewpoint_generation_en.{ext}',dpi=500,facecolor='white')
    fig.savefig(OUT/'viewpoint_generation_en_preview.png',dpi=220,facecolor='white')
    plt.close(fig)


if __name__=='__main__':
    scene=validate()
    render(scene)
    print('Verified: 72 proposals → 36 feasible views → 23 azimuth winners → 8 retained views.')
    print(OUT/'viewpoint_generation_en.pdf')
