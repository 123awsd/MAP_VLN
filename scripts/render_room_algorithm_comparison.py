from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/algorithm_comparison';font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']='Noto Sans CJK JP';plt.rcParams['axes.unicode_minus']=False
methods=[('raw_distance','原始障碍投影＋距离分割'),('raw_morphological','原始障碍投影＋形态学分割'),('author_rose2','作者ROSE²非Voronoi模式'),('extracted_wall_distance','提取墙体＋同一距离分割')]
fig,axs=plt.subplots(3,4,figsize=(20,15))
for f in range(1,4):
 for j,(method,title)in enumerate(methods):
  d=np.load(O/f'L{f}_{method}.npz');l=d['labels'];bar=d['barrier'];org=d['origin_xy'];res=float(d['resolution']);rgba=plt.get_cmap('tab20')((l%20)/19);rgba[:,:,3]=(l>0)*.7;ax=axs[f-1,j];ax.imshow(rgba[:,::-1],origin='lower',extent=[-10.5,8.5,-2.5,18.5]);y,x=np.where(bar);ax.scatter(-(org[0]+(x+.5)*res),org[1]+(y+.5)*res,s=.3,c='#333333',lw=0);ax.set(xlim=(-10.5,8.5),ylim=(-2.5,18.5),aspect='equal',title=f'{f}楼｜{title}',xlabel='−X / m',ylabel='Y / m')
fig.suptitle('同一700秒实测点云、统一范围与分辨率：常规基线初步对比',fontsize=16);fig.tight_layout(rect=[0,.045,1,.96]);fig.text(.5,.015,'黑色为各方法使用的障碍/墙体；全部不使用GT，不做此前外侧碎片过滤。\nROSE²为作者实现默认适配，未专项调参；未比较Hydra。分区数量不等于准确率，耗时不包含点云及墙体预处理。',ha='center',fontsize=10);fig.savefig(O/'four_method_comparison.png',dpi=170)
