"""Overlay measured SAM3.1 + Boxer fused L2 OBBs on the latest room map."""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR','/tmp/room_partition_figure_mpl')
import importlib.util,json,hashlib,argparse
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgb,to_hex

OUT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--editor-default',action='store_true')
parser.add_argument('--neutral-floor',action='store_true')
args=parser.parse_args()
variant=('_default_colors' if args.editor_default else '')+('_neutral_floor' if args.neutral_floor else '')
stem='room_partition_L2_SAM31_boxes'+variant
spec=importlib.util.spec_from_file_location('floor_view',OUT/'render_oblique.py')
floor=importlib.util.module_from_spec(spec);spec.loader.exec_module(floor)
ROOT=OUT.parents[2]
BOX_SOURCE=ROOT/'PRE_MAP_VLN/outputs/boxer/coverage700_sam31_expanded_20261019/fusion_fixed/boxer_summary.json'
summary=json.loads(BOX_SOURCE.read_text())
objects=[o for o in summary['objects'] if o.get('floor')==2]
GROUND=2.595
palette=['#3478F6','#F44870','#00B99A','#FF9B21','#9456E8','#10BAD1',
         '#E8BC18','#E84BB2','#38B64C','#FF6543','#6954E8','#05A6B3',
         '#BA58B7','#C08C0C','#3595DA','#B1B51A']
categories=sorted(Counter(o['label'] for o in objects),key=lambda k:(-sum(o['label']==k for o in objects),k))
colors={k:palette[i%len(palette)] for i,k in enumerate(categories)}
editor=None
if args.editor_default:
    editor_path=ROOT/'素材处理后/第一阶段/目标框检测/3D物体框调节.html'
    page=editor_path.read_text();start=page.index('const DATA=')+len('const DATA=')
    editor,_=json.JSONDecoder().raw_decode(page[start:])
    defaults={b['label']:b['color'] for b in editor['boxes']}
    cspec=importlib.util.spec_from_file_location('editor_colors',ROOT/'PRE_MAP_VLN/scripts/render_l2_room_box_standalone.py')
    cm=importlib.util.module_from_spec(cspec);cspec.loader.exec_module(cm)
    aliases={'painting':'picture','chandelier':'lamp','faucet':'sink',
             'wardrobe':'cabinet','shelf':'cabinet'}
    def editor_color(label):
        name=aliases.get(label,label)
        return defaults.get(name,cm.category_color(name))
    raw_colors={k:editor_color(k) for k in categories}
    def styled_color(hex):
        c=np.array(to_rgb(hex));hi,lo=max(c),min(c);light=(hi+lo)/2;chroma=hi-lo
        if chroma<1e-8:return to_hex(c)
        sat=chroma/max(1e-8,1-abs(2*light-1))
        target_light=light-(light-.5)*.05 if light>.5 else light
        target_chroma=min(1,sat*1.25)*(1-abs(2*target_light-1))
        saturated=np.floor(np.clip(target_light+(c-light)*target_chroma/chroma,0,1)*255+.5)
        return to_hex(np.floor(saturated+(255-saturated)*.10+.5)/255)
    colors={k:styled_color(v) for k,v in raw_colors.items()}
camera=np.array([np.cos(floor.e)*np.cos(floor.a),
                 np.cos(floor.e)*np.sin(floor.a),np.sin(floor.e)])
def project(v):
    out=np.asarray(v)[:,:2]@floor.projection.T
    out[:,1]+=(np.asarray(v)[:,2]-GROUND)*np.cos(floor.e)
    return out

projected=[project(np.asarray(o['corners_xyz'])) for o in objects]
extent=np.vstack([floor.extent,*projected])
lo,hi=extent.min(axis=0),extent.max(axis=0);span=hi-lo;pad=.055*max(span)
fig=plt.figure(figsize=(12,12*(span[1]+2*pad)/(span[0]+2*pad)),facecolor='white')
ax=fig.add_axes([0,0,1,1]);ax.set_aspect('equal');ax.axis('off')
ax.set_xlim(lo[0]-pad,hi[0]+pad);ax.set_ylim(lo[1]-pad,hi[1]+pad)
for i in range(16,0,-1):floor.draw(ax,floor.footprint,'#46515B',0,alpha=.0035,offset=(.004*i,-.012*i))
floor.draw(ax,floor.data['constrained_domain'],'#DAE0E5',1)
for rid,color in floor.colors.items():
    pale='#E0E8EE' if args.neutral_floor else to_hex(np.array(to_rgb(color))*.63+.37)
    floor.draw(ax,floor.labels==rid,pale,2)
if args.neutral_floor:
    import cv2
    for rid in floor.colors:
        contours,_=cv2.findContours((floor.labels==rid).astype('uint8'),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if len(contour)<3:continue
            world=(contour[:,0,:]+.5)*floor.resolution+floor.origin
            xy=world@floor.projection.T;xy=np.vstack([xy,xy[0]])
            ax.plot(xy[:,0],xy[:,1],color='white',lw=.65,zorder=2.5)
floor.draw(ax,floor.data['wall_barrier'],'#81909A',3)

edges=[(i,i^b) for i in range(8) for b in (1,2,4) if i<(i^b)]
faces=[([0,2,6,4],0),([1,3,7,5],1),([0,1,5,4],2),
       ([2,3,7,6],3),([0,1,3,2],4),([4,5,7,6],5)]
ordered=sorted(objects,key=lambda o:np.asarray(o['center_xyz'])@camera)
surfaces=[]
for index,o in enumerate(ordered):
    v=np.asarray(o['corners_xyz'],float);assert v.shape==(8,3) and np.isfinite(v).all()
    xy=project(v);color=colors[o['label']];z=5+index*.02
    # Draw camera-facing faces only; edge wireframes retain all 12 OBB edges.
    center=v.mean(axis=0)
    for ids,_ in faces:
        face=v[ids];normal=np.cross(face[1]-face[0],face[2]-face[1])
        if np.dot(normal,face.mean(axis=0)-center)<0:normal=-normal
        if normal@camera>0:
            if args.editor_default:
                shade=1 if ids==[1,3,7,5] else (.96 if _%2 else .91)
                surfaces.append((face.mean(axis=0)@camera,xy[ids],np.array(to_rgb(color))*shade))
            else:
                ax.add_patch(Polygon(xy[ids],closed=True,facecolor=color,edgecolor='none',alpha=.13,zorder=z))
    if not args.editor_default:
        ax.add_collection(LineCollection([[xy[i],xy[j]] for i,j in edges],
            colors=[color],linewidths=.9,alpha=.87,zorder=z+.01))
if args.editor_default:
    for index,(_,points,color) in enumerate(sorted(surfaces,key=lambda v:v[0])):
        ax.add_patch(Polygon(points,closed=True,facecolor=color,edgecolor='#7E8D99',
            lw=.55,alpha=1,zorder=5+index*.001,joinstyle='round'))
for suffix in ('png','pdf','svg'):
    fig.savefig(OUT/f'{stem}.{suffix}',dpi=350,facecolor='white')
plt.close(fig)

legend=plt.figure(figsize=(12,1.1),facecolor='white')
for i,label in enumerate(categories):
    col,row=i%8,i//8;x=.02+col*.123;y=.68-row*.46
    legend.text(x,y,'■',color=colors[label],fontsize=14,va='center')
    legend.text(x+.019,y,label,fontsize=8.5,color='#34414E',va='center')
legend.savefig(OUT/('SAM31_box_category_colors'+variant+'.png'),dpi=250,facecolor='white');plt.close(legend)
metadata=dict(floor=2,source=str(BOX_SOURCE),source_sha256=hashlib.sha256(BOX_SOURCE.read_bytes()).hexdigest(),
    detector='SAM3.1 image detector, expanded vocabulary; Boxer 3D estimation; fixed multi-view fusion',
    number_of_boxes=len(objects),category_counts=dict(Counter(o['label'] for o in objects)),category_colors=colors,
    projection=dict(azimuth_degrees=-58,elevation_degrees=26,ground_z=GROUND,height_scale=1),
    box_geometry='Exact source corners_xyz, original position, orientation and dimensions; selected by recorded floor=2.',
    floor_source=str(floor.base.BASE/'wall_structural_rooms/L2_constrained.npz'),
    view_bounds=[lo.tolist(),hi.tolist()],all_boxes_rendered=[o['id'] for o in ordered],
    display=dict(box_face_alpha=.13,box_edge_alpha=.87,room_colors='Original palette mixed with 37% white'))
if args.editor_default:
    metadata['display'].update(box_face_alpha=1,box_edge_alpha=1,
        edge_color='#7E8D99',palette_source=str(editor_path),
        raw_category_colors=raw_colors,saturation=1.25,pastel_mix=.10,
        aliases_for_expanded_vocabulary=aliases)
if args.neutral_floor:
    metadata['display']['room_colors']='Uniform pale blue-gray #E0E8EE; original room boundaries retained with white contours.'
(OUT/('SAM31_overlay_provenance'+variant+'.json')).write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
print(OUT/f'{stem}.png')
