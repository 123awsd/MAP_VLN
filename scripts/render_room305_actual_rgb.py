from pathlib import Path
import json,numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[1];r=R/'outputs/coverage_lifecycle_700_r1_20261016';o=R/'outputs/coverage700_wall_room_preview/room305_official';o.mkdir(exist_ok=True)
a=np.load(r/'video_cache/timeline.npz');t=a['snapshot_times'];frames=[json.loads(s)for s in (r/'capture_trace.jsonl').open()];ft=np.array([f['capture_wall_s']-frames[0]['capture_wall_s']for f in frames]);font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',23);canvas=Image.new('RGB',(1280,1040),'white');d=ImageDraw.Draw(canvas);rows=[]
for j,sec in enumerate([686,688,692.9,693.7]):
 i=int(abs(t-sec).argmin());fi=int(abs(ft-t[i]).argmin());im=Image.open(r/f'video_cache/rgb_{i:03d}.jpg');x=j%2*640;y=j//2*520;canvas.paste(im,(x,y+40));d.text((x+8,y+7),f'实测RGB {float(t[i]):.2f}s',font=font,fill='black');rows.append({'rgb_index':i,'time':float(t[i]),'nearest_capture_sequence':frames[fi]['sequence'],'time_error_s':float(ft[fi]-t[i]),'position':frames[fi]['position'],'yaw':frames[fi]['executed_command_yaw']})
canvas.save(o/'actual_rgb_selected.png');(o/'actual_rgb_manifest.json').write_text(json.dumps(rows,indent=2))
