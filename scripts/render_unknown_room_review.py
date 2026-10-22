#!/usr/bin/env python3
"""Display the actual selected RGB evidence behind every unknown room decision."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/stage1_pipeline_flow_20261019/automated_room_semantics'
decisions=json.loads((OUT/'semantic_results.json').read_text(encoding='utf-8'))['decisions']
unknown=[d for d in decisions if d['room_type']=='unknown']
frames=json.loads((OUT/'selected_frames.json').read_text(encoding='utf-8'))
graph={r['id']:r for r in json.loads((OUT/'room_graph_geometry.json').read_text(encoding='utf-8'))['rooms']}
summaries={
    103:'Doorway and tiled area; bathroom edge or passage is visible, but no decisive in-room fixture.',
    106:'Sparse room or passage with doors and ceiling light; no functional furniture or Boxer object.',
    207:'Tiny 2.84 m² region; photos mostly show adjacent kitchen/dining or doorframe; Boxer has doors only.',
    304:'Dining/living cues in one view, wardrobe/doorway in another; no Boxer objects to resolve the mix.',
    305:'One usable view of a doorframe and stair structure; no functional object evidence.',
    309:'Chair and ambiguous bench/cabinet; second view is mostly a door; no distinctive room fixture.',
}
fig=plt.figure(figsize=(16,2.45*len(unknown)+1.35),facecolor='#F8F7F3')
fig.text(.025,.968,'Why Six Rooms Remain Unknown',fontsize=24,fontweight='bold',color='#172237')
fig.text(.026,.943,'Measured views used by the model; reasons summarize its recorded response. No manual relabeling.',
         fontsize=10.5,color='#59667B')
for index,d in enumerate(unknown):
    room_id=d['room_id'];key=f'{room_id//100}-{room_id%100:02d}'
    top=.905-index*.145
    fig.text(.026,top,f'{key}  |  Unknown',fontsize=14.2,fontweight='bold',color='#172237')
    fig.text(.026,top-.021,f'Boxer objects: {len(graph[room_id]["objects"])}  ·  Model confidence: {d["confidence"]:.2f}',
             fontsize=9.5,color='#667186')
    fig.text(.026,top-.051,summaries[room_id],fontsize=9.4,color='#33435B',wrap=True)
    for j,record in enumerate(frames[key]):
        ax=fig.add_axes([.51+j*.237,top-.107,.22,.105])
        ax.imshow(Image.open(record['image']));ax.axis('off')
        ax.set_title(f'View {j+1} · {record["frame"].removesuffix(".npz")}',fontsize=8.8,pad=2)
    if len(frames[key])==1:
        fig.text(.76,top-.055,'No second in-room view',fontsize=10,color='#7C8798')
    fig.add_artist(plt.Line2D([.025,.975],[top-.126,top-.126],transform=fig.transFigure,
                              color='#E0E3E7',linewidth=.7))
fig.text(.026,.024,'Unknown means the current images and object list did not support a reliable room type; it is not a verified geometry error.',
         fontsize=9,color='#59667B')
for extension in ('png','pdf'):
    fig.savefig(OUT/f'unknown_rooms_evidence.{extension}',dpi=155,facecolor='#F8F7F3',bbox_inches='tight',pad_inches=.12)
plt.close(fig)
print(OUT/'unknown_rooms_evidence.png')
