#!/usr/bin/env python3
"""Render existing automatic room regions and their multi-view semantic decisions."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager, patches
from scipy.ndimage import distance_transform_edt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/stage1_pipeline_flow_20261019/automated_room_semantics'
RESULTS = json.loads((OUT / 'semantic_results.json').read_text(encoding='utf-8'))
DECISIONS = {d['room_id']: d for d in RESULTS['decisions']}
COLORS = {
    'bedroom':'#7A63F5', 'bathroom':'#32B9CC', 'kitchen':'#FF9264',
    'living_room':'#FF5F91', 'dining_room':'#F5B548', 'corridor':'#58B88B',
    'entry_hall':'#77A9F5', 'laundry_utility':'#A7B95E', 'unknown':'#C3C8D2',
}
NAMES = {
    'bedroom':'Bedroom', 'bathroom':'Bathroom', 'kitchen':'Kitchen',
    'living_room':'Living room', 'dining_room':'Dining room', 'corridor':'Corridor',
    'entry_hall':'Entry hall', 'laundry_utility':'Utility', 'unknown':'Unknown',
}
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False})
fig, axes = plt.subplots(1, 3, figsize=(21, 8.8), facecolor='#F8F7F3')
fig.subplots_adjust(left=.035, right=.985, top=.86, bottom=.15, wspace=.065)
fig.text(.035,.945,'30 Rooms  |  RGB + Boxer Semantic Classification',fontsize=24,color='#172237',fontweight='bold')
accepted=sum(d['accepted'] for d in DECISIONS.values())
fig.text(.036,.902,f'700 s measured map  ·  1–2 real RGB views per room  ·  Qwen3-VL-Plus  ·  {accepted}/30 accepted',
         fontsize=11.5,color='#59667B')
for floor, ax in enumerate(axes, 1):
    data=np.load(ROOT/f'outputs/coverage700_wall_room_preview/wall_structural_rooms/L{floor}_constrained.npz')
    labels=data['labels']; origin=data['origin_xy']; res=float(data['resolution'])
    footprint=np.load(OUT/f'semantic_footprints/L{floor}_semantic_footprint.npz')
    display_labels=footprint['semantic_footprint_labels']
    assert np.array_equal(footprint['original_room_labels'],labels)
    canvas=np.ones((*labels.shape,4),dtype=float)
    canvas[:,:,:3]=[.95,.95,.96]; canvas[:,:,3]=0
    for rid in np.unique(labels[labels>0]):
        rid=int(rid); decision=DECISIONS[floor*100+rid]
        rgb=matplotlib.colors.to_rgb(COLORS[decision['room_type']])
        canvas[display_labels==rid,:3]=rgb; canvas[display_labels==rid,3]=.77 if decision['accepted'] else .48
    ax.imshow(canvas[:,::-1],origin='lower',extent=[-origin[0]-labels.shape[1]*res,-origin[0],
        origin[1],origin[1]+labels.shape[0]*res],interpolation='nearest')
    wall=data['measured_wall']; y,x=np.where(wall & (display_labels==0))
    ax.scatter(-(origin[0]+(x+.5)*res),origin[1]+(y+.5)*res,s=.14,c='#304057',alpha=.48,lw=0,rasterized=True)
    for rid in np.unique(labels[labels>0]):
        rid=int(rid); mask=labels==rid; dt=distance_transform_edt(mask)
        yy,xx=np.unravel_index(dt.argmax(),dt.shape)
        decision=DECISIONS[floor*100+rid]
        label=f'{floor}-{rid:02d}\n{NAMES[decision["room_type"]]}'
        ax.text(-(origin[0]+(xx+.5)*res),origin[1]+(yy+.5)*res,label,
                ha='center',va='center',fontsize=8.3,color='#172237',fontweight='bold',
                bbox=dict(boxstyle='round,pad=.22',fc='white',ec='none',alpha=.82))
    ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),title=f'Floor {floor}')
    ax.set_aspect('equal'); ax.tick_params(labelsize=8)
    ax.set_facecolor('#F8F7F3')
handles=[patches.Patch(facecolor=c,label=NAMES[k]) for k,c in COLORS.items()]
fig.legend(handles=handles,ncol=9,loc='lower center',bbox_to_anchor=(.5,.084),frameon=False,fontsize=10)
fig.text(.036,.035,'Model predictions, not ground truth. Semantic footprints fill narrow seams; wall barriers and navigable room masks remain unchanged. −X/Y view.',
         color='#59667B',fontsize=9.5)
for suffix in ('png','pdf'):
    fig.savefig(OUT/f'all_rooms_multiview_semantics.{suffix}',dpi=160,facecolor='#F8F7F3',bbox_inches='tight',pad_inches=.16)
plt.close(fig)
print(OUT/'all_rooms_multiview_semantics.png')
