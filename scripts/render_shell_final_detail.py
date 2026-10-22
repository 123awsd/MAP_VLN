from pathlib import Path
import numpy as np
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'shell_filtered_final'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(1,2,figsize=(13,6))
for ax,folder,title in zip(axes,['wall_structural_rooms','shell_filtered_final'],['原最终分割','过滤后全记录离线分割']):
 a=np.load(B/folder/'L3_constrained.npz');l=a['labels'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest');o=a['origin_xy'];r=float(a['resolution']);y,x=np.where(a['measured_wall']);ax.scatter(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,c='#333333',s=.7,lw=0)
 for k in np.unique(l[l>0]):
  d=nd.distance_transform_edt(l==k);y,x=np.unravel_index(d.argmax(),d.shape);t=ax.text(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,f'3-{k:02d}',ha='center',fontsize=11,clip_on=True);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 ax.set(xlim=(1.5,6.4),ylim=(3.5,6.7),aspect='equal',title=title,xlabel='−X / m',ylabel='Y / m')
fig.suptitle('305附近最终房间分割对比｜全记录采样重建，非在线TSDF等价重放');fig.tight_layout();fig.savefig(O/'room305_final_comparison.png',dpi=190)
