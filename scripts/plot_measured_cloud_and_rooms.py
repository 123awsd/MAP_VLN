"""Measured cloud and reviewed room partitions, identical -X/Y axes."""
from pathlib import Path
import numpy as np
from scipy import ndimage as nd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,patheffects
R=Path(__file__).resolve().parents[1];B=R/'outputs/coverage700_wall_room_preview';O=B/'wall_guided/measured_review';G=np.load(R/'outputs/coverage_lifecycle_700_r1_20261016/original_style_video/geometry.npz')
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
fig,axs=plt.subplots(3,2,figsize=(13,19))
for f,g in enumerate([-.155,2.595,5.595]):
 a=np.load(O/f'L{f+1}_rooms.npz');p=G[f'p{f}'];p=p[(p[:,2]>g+.15)&(p[:,2]<g+2.2)]
 ax=axs[f,0];ax.scatter(-p[:,0],p[:,1],c=p[:,2]-g,cmap='Blues',vmin=-.3,vmax=2.5,s=.5,lw=0,rasterized=True)
 l=a['labels'];rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;ax=axs[f,1];ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5],interpolation='nearest');org=a['origin_xy'];res=float(a['resolution']);y,x=np.where(a['wall_grid']);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=.35,c='#a51b32',lw=0)
 for rid in np.unique(l):
  if not rid:continue
  dist=nd.distance_transform_edt(l==rid);y,x=np.unravel_index(dist.argmax(),dist.shape);t=ax.text(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,f'{f+1}-{rid:02d}',ha='center',va='center',fontsize=9,weight='bold');t.set_path_effects([patheffects.withStroke(linewidth=2,foreground='white')])
 for j in range(2):axs[f,j].set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',xlabel='−X / m',ylabel='Y / m',title=f'{f+1}楼｜'+('实测点云俯视（结构高度切片）'if j==0 else'当前墙体引导分区'))
fig.suptitle('同一700秒实测数据：点云与房间分区对照',fontsize=17);fig.tight_layout(rect=[0,.045,1,.965]);fig.text(.5,.018,'左：原始深度生成的4cm去重点云，显示各层地面以上0.15–2.20m，去掉地板/顶面遮挡。\n右：当前实测分区，保留编号；疑似外围小碎片按既有规则隐藏。全部统一−X/Y，无GT补全。',ha='center',fontsize=10);fig.savefig(O/'cloud_and_room_partitions.png',dpi=180)
