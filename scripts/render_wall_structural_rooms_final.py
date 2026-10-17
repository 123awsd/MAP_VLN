from pathlib import Path
import numpy as np
from scipy import ndimage as nd
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
b=Path('outputs/coverage700_wall_room_preview/wall_structural_rooms');fig,axs=plt.subplots(1,3,figsize=(18,8))
for f,ax in enumerate(axs,1):
 a=np.load(b/f'L{f}_constrained.npz');l=a['labels'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);o=a['origin_xy'];r=float(a['resolution']);y,x=np.where(a['measured_wall']);ax.scatter(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,s=.5,c='#333333',lw=0)
 for k in np.unique(l[l>0]):
  d=nd.distance_transform_edt(l==k);y,x=np.unravel_index(d.argmax(),d.shape);t=ax.text(-(o[0]+(x+.5)*r),o[1]+(y+.5)*r,f'{f}-{k:02d}',ha='center',fontsize=9);t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 ax.set(title=f'{f}楼',aspect='equal',xlabel='−X / m',ylabel='Y / m',xlim=(-10.5,8.5),ylim=(-2.5,18.5))
fig.suptitle('新墙提取 → 独立观测支持 → 房间分割｜实测700秒数据',fontsize=16);fig.tight_layout(rect=[0,.02,1,.95]);fig.savefig(b/'final_three_floors.png',dpi=180)
