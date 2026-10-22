"""Render the first two measured point-cloud floors with the same projection."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args()
data=np.load(a.trial/'evaluation_geometry.npz');fig,axes=plt.subplots(1,2,figsize=(19,10.5),dpi=170);fig.patch.set_facecolor('#12191d');meta=[]
for ax,index,ground,lo,hi in [(axes[0],0,-.155,.05,2.10),(axes[1],1,2.595,2.8,4.85)]:
 q=data[f'p{index}'];q=q[(q[:,2]>lo)&(q[:,2]<hi)];ax.set_facecolor('#12191d');ax.scatter(-q[:,0],q[:,1],c=q[:,2]-ground,cmap='summer',s=.65,vmin=.2,vmax=2.25,linewidths=0,rasterized=True);ax.set_aspect('equal');ax.set_xlim(-10,8);ax.set_ylim(-2,18);ax.set_title(f'Floor {index+1} | final 504 s',color='white',fontsize=17,pad=12);ax.set_xlabel('-X (m)',color='white');ax.set_ylabel('Y (m)',color='white');ax.tick_params(colors='#b2c3c9')
 for spine in ax.spines.values():spine.set_color('#627077')
 meta.append({'floor':index+1,'z_crop_m':[lo,hi],'points':len(q)})
fig.suptitle('Combined fixes | measured point cloud',color='white',fontsize=20);fig.tight_layout(rect=[0,0,1,.96]);dest=a.trial/'floors_1_2_topdown_wide.png';fig.savefig(dest,facecolor=fig.get_facecolor());plt.close(fig)
(a.trial/'floors_1_2_topdown_wide.json').write_text(json.dumps({'source':'evaluation_geometry.npz','projection':'-X/Y','color':'height, not RGB','no_inpainting':True,'floors':meta},indent=2));print(dest)
