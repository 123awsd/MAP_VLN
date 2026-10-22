#!/usr/bin/env python3
"""Render existing room labels over actual depth points and newly extracted walls.

Visualization does not alter labels or infer unseen geometry.
"""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--trial-dir',required=True,type=Path)
p.add_argument('--maps',required=True,nargs=3,type=Path)
p.add_argument('--output',required=True,type=Path)
a=p.parse_args()
plt.rcParams['font.family']='Noto Sans CJK JP'
fig,axes=plt.subplots(1,3,figsize=(18,9),facecolor='white')
with np.load(a.trial_dir/'geometry.npz') as geometry:
 for f,(ax,path) in enumerate(zip(axes,a.maps)):
  with np.load(path) as m:
   labels=m['labels'];origin=m['origin'];res=float(m['resolution'])
  points=geometry[f'p{f}'];walls=np.load(a.trial_dir/f'L{f+1}_new_wall_points.npz')['wall_points']
  y,x=np.where(labels>0);colors=plt.get_cmap('Pastel1')((labels[y,x]-1)%9)
  ax.scatter(-(origin[0]+(x+.5)*res),-(origin[1]+(y+.5)*res),c=colors,s=3,alpha=.45,rasterized=True)
  sampled=points[::max(1,len(points)//60000)]
  ax.scatter(-sampled[:,0],-sampled[:,1],c='#aaaaaa',s=.5,alpha=.8,rasterized=True)
  ax.scatter(-walls[:,0],-walls[:,1],c='#9b141e',s=2,rasterized=True)
  for rid in np.unique(labels[labels>0]):
   yy,xx=np.where(labels==rid);anchor=np.argmin((xx-np.median(xx))**2+(yy-np.median(yy))**2)
   ax.text(-(origin[0]+(xx[anchor]+.5)*res),-(origin[1]+(yy[anchor]+.5)*res),str(rid),fontsize=10,bbox=dict(facecolor='white',edgecolor='none',alpha=.85))
  ax.set_aspect('equal');ax.set_facecolor('white');ax.set_xlim(-11,9);ax.set_ylim(-19,3)
  ax.set_title(f'{f+1}层 · 区域编号仅供复核');ax.set_xlabel('−FALCON X (m)');ax.set_ylabel('−FALCON Y (m)')
fig.suptitle('实际深度点（灰）/ 新提取墙面（加粗红）/ 房间候选（浅色）\n区域编号不代表房间分割已验证',fontsize=15)
fig.tight_layout();a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=150);plt.close(fig)
