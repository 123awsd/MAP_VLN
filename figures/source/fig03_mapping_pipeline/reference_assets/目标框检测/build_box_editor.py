from pathlib import Path
import sys,json,csv
ROOT=Path(__file__).resolve().parents[3];OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'PRE_MAP_VLN/scripts'))
from render_l2_room_box_standalone import footprint_contours,category_color,ROOM_COLORS,room_type_map
import numpy as np
meta=json.loads((OUT/'场景00337_L2_Boxer目标框_单层.json').read_text());scene=json.loads(Path(meta['source_scene_graph']).read_text());gridmeta=json.loads(Path(meta['source_wall_meta']).read_text());wall=np.load(meta['source_wall_grid'])==100
with open(meta['source_box_csv']) as f:rows=list(csv.DictReader(f))
boxes=[]
for i,r in enumerate(rows):boxes.append(dict(id=i,label=r['name'],center=[float(r[k]) for k in ['tx_world_object','ty_world_object','tz_world_object']],size=[float(r[k]) for k in ['scale_x','scale_y','scale_z']],yaw=2*np.arctan2(float(r['qz_world_object']),float(r['qw_world_object'])),color=category_color(r['name'])))
data={'boxes':boxes,'floor':gridmeta['floor_z_m'],'contours':[x.tolist() for x in footprint_contours(scene,gridmeta,wall)],'source':meta['source_box_csv']}
# Use the room image's source geometry in the same world-coordinate projection.
room_meta=json.loads((OUT.parent/'房间分割/场景00337_L2_房间分割_单层.json').read_text())
room_scene=json.loads(Path(room_meta['source_scene_graph']).read_text())
room_grid=json.loads(Path(room_meta['source_wall_meta']).read_text())
room_wall=np.load(room_meta['source_wall_grid'])==100
types=room_type_map(room_scene)
data['rooms']=[{'polygon':r['polygon_xy_m'],'color':ROOM_COLORS.get(types.get(int(r['id']),'unknown'),ROOM_COLORS['unknown'])} for r in room_scene['rooms'] if len(r.get('polygon_xy_m',[]))>=3]
ox,oy=room_grid['origin_xy_m'];res=room_grid['resolution_m']
data['wallRects']=[]
for y,row in enumerate(room_wall):
    transitions=np.diff(np.r_[False,row,False].astype(np.int8))
    for start,end in zip(np.flatnonzero(transitions==1),np.flatnonzero(transitions==-1)):
        data['wallRects'].append([ox+int(start)*res,oy+y*res,ox+int(end)*res,oy+(y+1)*res])
(OUT/'3D物体框调节.html').write_text((OUT/'box_editor_template.html').read_text().replace('__DATA__',json.dumps(data,separators=(',',':'))))
print('built',len(boxes),'boxes')
