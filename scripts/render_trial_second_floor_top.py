#!/usr/bin/env python3
"""Top-down measured point cloud, height color; no filled or inferred geometry."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);a=p.parse_args();q=np.load(a.trial/'evaluation_geometry.npz')['p1'];q=q[(q[:,2]>2.8)&(q[:,2]<4.85)]
fig,ax=plt.subplots(figsize=(11,12),dpi=180);ax.set_facecolor('#12191d');fig.patch.set_facecolor('#12191d');ax.scatter(-q[:,0],q[:,1],c=q[:,2]-2.595,cmap='summer',s=.55,vmin=.2,vmax=2.25,linewidths=0,rasterized=True);ax.set_aspect('equal');ax.set_xlim(-10,8);ax.set_ylim(-2,18);ax.set_title('Second floor | observed point cloud | final 504 s',color='white',pad=14);ax.set_xlabel('-X (m)',color='white');ax.set_ylabel('Y (m)',color='white');ax.tick_params(colors='#b2c3c9');
for spine in ax.spines.values():spine.set_color('#627077')
fig.tight_layout();dest=a.trial/'second_floor_topdown.png';fig.savefig(dest,facecolor=fig.get_facecolor());plt.close(fig)
(a.trial/'second_floor_topdown.json').write_text(json.dumps({'source':'evaluation_geometry.npz:p1','z_crop_m':[2.8,4.85],'projection':'horizontal -X/Y, no perspective','color':'measured height, not RGB','points':len(q),'no_inpainting':True},indent=2));print(dest)
