from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
from repair_depth_validity import invalidate_safety_shell_depth
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from PIL import Image
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'room305_root_cause';D=B/'room305_provenance';a=np.load(D/'depth_witnesses.npz');j=json.loads((D/'audit.json').read_text());frames={round(f['capture_wall_s'],6):f for f in map(json.loads,(R/'outputs/coverage_lifecycle_700_r1_20261016/capture_trace.jsonl').read_text().splitlines())};cfg=json.loads((R/'scene_repairs/00337_three_floor_shell/repair_collision_only.json').read_text());shells=[(v['falcon_min'],v['falcon_max'])for v in cfg['objects']]
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(2,3,figsize=(15,9));before=[];after=[]
for b in j['best']:
 i=b['index'];f=frames[round(b['stamp'],6)];rot=Rotation.from_quat(f['orientation_xyzw']).as_matrix();d=a[f'depth{i}'];new,invalid=invalidate_safety_shell_depth(d,f['position'],rot,shells);y,x=np.indices(d.shape);p=np.stack([(x-320)*d/320,(y-240)*d/320,d],axis=-1)@rot.T+f['position'];roi=(d>.15)&(d<5)&(p[:,:,0]>-6.2)&(p[:,:,0]<-1.7)&(p[:,:,1]>3.7)&(p[:,:,1]<6.4)&(p[:,:,2]>5.4)&(p[:,:,2]<8.3);before.append(p[roi]);after.append(p[roi&~invalid])
 if i==0:
  axs[0,0].imshow(Image.open(D/'rgb0.png'));axs[0,0].set_title('实际RGB（保持原样）')
  axs[0,1].imshow(d,cmap='viridis',vmin=0,vmax=5);axs[0,1].set_title('修复前深度')
  axs[0,2].imshow(new,cmap='viridis',vmin=0,vmax=5);axs[0,2].set_title('修复后深度：黑紫色为无效测量')
for ax in axs[0]:ax.axis('off')
p=np.concatenate(before);q=np.concatenate(after)
for ax,pts,title in [(axs[1,0],p,'8帧局部端点：修复前'),(axs[1,1],q,'同8帧局部端点：修复后')]:
 ax.scatter(-pts[:,0],pts[:,1],c=pts[:,2]-5.595,cmap='viridis',s=.2,vmin=0,vmax=2.5,lw=0);ax.set(xlim=(1.7,6.2),ylim=(3.7,6.4),aspect='equal',title=title,xlabel='−X / m',ylabel='Y / m')
ax=axs[1,2];ax.scatter(-p[:,0],p[:,2]-5.595,s=.2,c='#c5c5c5',label='原测量',lw=0);ax.scatter(-q[:,0],q[:,2]-5.595,s=.2,c='#168e8a',label='保留测量',lw=0);ax.set(xlim=(1.7,6.2),ylim=(-.2,2.7),title='高度对比：灰色原测量，绿色保留',xlabel='−X / m',ylabel='相对楼面高度 / m')
fig.suptitle('安全壳深度修复效果｜真实8帧重处理，非全700秒重建/最终房间分割',fontsize=15);fig.tight_layout();fig.savefig(O/'depth_effect.png',dpi=180)
