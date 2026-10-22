#!/usr/bin/env python3
"""Previous dense depth-surface scatter style, latest run only."""
import json
import argparse
p=argparse.ArgumentParser();p.add_argument("--mirror-x",action="store_true");args=p.parse_args()
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
root=Path(__file__).resolve().parents[1];out=root/'outputs/consistency_validation_20261015/latest_dense_floor_topdown';grid=np.load(out/'dense_observed_surfaces.npz');font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font);plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
extent=(-8.5,10.5,-2.5,18.5)
suffix='_minus_x' if args.mirror_x else ''
def draw(ax,i,clean=False):
 q=grid[f'floor{i+1}'];order=np.argsort(q[:,2]);q=q[order]
 ax.scatter(-q[:,0] if args.mirror_x else q[:,0],q[:,1],c='#123f5a',s=.55,marker='.',linewidths=0,alpha=1.,rasterized=True)
 ax.set_xlim((-extent[1],-extent[0]) if args.mirror_x else extent[:2]);ax.set_ylim(extent[2:]);ax.set_aspect('equal');ax.grid(False)
 if clean:ax.axis('off')
 else:ax.set_title(f'{i+1}楼｜稠密深度点云俯视图',fontsize=13,pad=12);ax.set_xlabel('-X / m' if args.mirror_x else 'X / m');ax.set_ylabel('Y / m')
for clean,filename in [(False,'latest_three_floors_dense_topdown.png'),(True,'latest_three_floors_dense_topdown_clean.png')]:
 fig,axes=plt.subplots(1,3,figsize=(16,6.8),facecolor='white')
 for i,ax in enumerate(axes):draw(ax,i,clean)
 if not clean:
  fig.suptitle('最新504秒运行 · 分层稠密点云',fontsize=18,y=.98);fig.text(.5,.025,'真实深度帧重投影与2.5 cm体素融合｜去除地板、天花板｜未观测区域不补点',ha='center',fontsize=10,color='#555555');fig.subplots_adjust(left=.04,right=.99,bottom=.11,top=.88,wspace=.15)
 else:fig.subplots_adjust(left=0,right=1,bottom=0,top=1,wspace=.035)
 fig.savefig(out/(Path(filename).stem+suffix+'.png'),dpi=220,facecolor='white',bbox_inches='tight',pad_inches=.02);plt.close(fig)
for i in range(3):
 fig,ax=plt.subplots(figsize=(8,9),facecolor='white');draw(ax,i,True);fig.subplots_adjust(left=0,right=1,bottom=0,top=1);fig.savefig(out/f'floor{i+1}_dense_topdown{suffix}.png',dpi=220,bbox_inches='tight',pad_inches=0,facecolor='white');plt.close(fig)
(out/('projection'+suffix+'.json')).write_text(json.dumps({'projection':'-X/Y' if args.mirror_x else 'X/Y','display_only':True,'source_cloud_unchanged':True},indent=2))
print(out/('latest_three_floors_dense_topdown'+suffix+'.png'))
