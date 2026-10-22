"""Latest 700s RGB-D vs official HM3D, fixed raw coordinates; -X display only."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
root=Path(__file__).resolve().parents[1];base=root/'outputs/consistency_validation_20261015';out=base/'coverage700_remaining';ref=np.load(base/'global_observability/surface_witnesses.npz');p=ref['points'];f=ref['floor'];d=np.load(out/'reference_distance.npz')['distance_m'];a=np.load(out/'opportunity_witnesses.npz')['verified'];official=np.load(base/'latest_official_comparison/official_sampled_surfaces.npz')['points'];parts=[]
for path in sorted((base/'coverage_surface_exports/coverage_lifecycle_700_r1_20261016').glob('chunk*.npz')):
 with np.load(path)as c:parts.append(c['points'][::8])
obs=np.concatenate(parts);grounds=[-.155,2.595,5.595];font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';font_manager.fontManager.addfont(font);plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name();plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,3,figsize=(15,16));rows=[]
for i,g in enumerate(grounds):
 keep=lambda q:(q[:,2]>=g+.15)&(q[:,2]<g+2.2)&(q[:,0]>=-8.5)&(q[:,0]<10.5)&(q[:,1]>=-2.5)&(q[:,1]<18.5)
 q=official[keep(official)];r=obs[keep(obs)];m=f==i+1
 axes[i,0].scatter(-q[:,0],q[:,1],s=.3,c='#4b5865',marker='.',lw=0,rasterized=True);axes[i,1].scatter(-r[:,0],r[:,1],s=.35,c='#124766',marker='.',lw=0,rasterized=True)
 for mask,color,size in [(m&(d<=.08),'#078da7',.7),(m&(d>.08)&~a,'#aeb4ba',1.2),(m&(d>.08)&a,'#d55e00',2.)]:axes[i,2].scatter(-p[mask,0],p[mask,1],s=size,c=color,marker='.',lw=0,rasterized=True)
 for j,ax in enumerate(axes[i]):
  ax.set_xlim(-10.5,8.5);ax.set_ylim(-2.5,18.5);ax.set_aspect('equal');ax.set_xlabel('−X / m');ax.set_ylabel('Y / m');ax.set_title(f'{i+1}楼｜'+['官方HM3D网格表面','700秒实际深度点云','8cm匹配与剩余机会'][j],fontsize=12)
 rows.append({'floor':i+1,'reference_area_est_m2':round(float(ref['weight_m2'][m].sum()),2),'matched_area_est_m2':round(float(ref['weight_m2'][m&(d<=.08)].sum()),2),'remaining_with_historical_view_m2':round(float(ref['weight_m2'][m&(d>.08)&a].sum()),2)})
fig.suptitle('最新700秒 vs 官方HM3D参考｜统一−X/Y俯视',fontsize=17);fig.text(.5,.025,'青：已匹配；橙：未匹配但有历史视点几何复核证据；灰：其余未匹配、原因待定。\n仅室内高度切片，非整栋可观测完整度；点云为保存深度的实测端点，未插值。',ha='center',fontsize=10);fig.subplots_adjust(top=.95,bottom=.08,wspace=.15,hspace=.23);fig.savefig(out/'official_vs_700_three_floors.png',dpi=170,facecolor='white');plt.close(fig);(out/'figure_metadata.json').write_text(json.dumps({'rows':rows,'raw_coordinate_frame':'FALCON','display':'-X/Y only','reference':'official scene mesh previously sampled at uniform area density','observed':'saved 700s RGB-D stride4/.4s exports, plot subsampled every8 endpoints','repair_overlay_present_during_runtime_and_opportunity_checks':True,'truth_for_offline_evaluation_only':True},indent=2));print(rows)
