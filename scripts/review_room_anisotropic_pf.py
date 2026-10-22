"""Visual review and topology diagnostics; no ground-truth accuracy claim."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.ndimage import label
from PIL import Image,ImageDraw
p=argparse.ArgumentParser();p.add_argument('--variant',default='corrected_fullvis');a=p.parse_args();root=Path('outputs/room_apf_latest_trial');palette=np.array([[130,190,225],[244,174,139],[179,213,153],[200,171,219],[238,216,125],[144,211,193],[225,159,186],[179,190,230],[212,187,155],[170,216,215],[229,197,208],[180,222,170],[232,161,123],[143,171,216],[180,171,133],[210,171,171],[133,200,215],[190,200,140]])
panels=[];metrics=[]
for f in [1,2,3]:
 folder=root/f'L{f}_{a.variant}';d=np.load(folder/'segmentation.npz');points=d['points'];labs=d['point_labels'];cells=d['cells'];cl=d['cell_labels'];pf=d['potential_field'];origin=d['grid_origin'];v=float(d['voxel']);nx,ny=pf.shape
 grid=np.full((nx,ny),-1,int);grid[tuple(cells.T)]=cl
 colored=np.full((ny,nx,3),250,np.uint8);idx=cl>=0;colored[ny-1-cells[idx,1],cells[idx,0]]=palette[cl[idx]%len(palette)]
 records=[]
 for lab in np.unique(cl[cl>=0]):
  mask=grid==lab;cc,n=label(mask);sizes=np.bincount(cc.ravel())[1:];xy=cells[cl==lab].mean(0);records.append({'label':int(lab),'area_m2':float(mask.sum()*v*v),'connected_components':int(n),'components_ge_half_m2':int((sizes*v*v>=.5).sum()),'point_count':int((labs==lab).sum())})
 image=Image.fromarray(colored).resize((nx*5,ny*5),Image.Resampling.NEAREST);draw=ImageDraw.Draw(image)
 for record in records:
  xy=cells[cl==record['label']].mean(0);draw.text((int(xy[0]*5),int((ny-1-xy[1])*5)),str(record['label']),(20,20,20))
 image.save(folder/'room_regions_numbered.png')
 # Observed points colored by room; small slices help wall gaps remain visible.
 ground=[-.155,2.595,5.595][f-1];keep=(points[:,2]>ground+.3)&(points[:,2]<ground+2.2);q=points[keep];pl=labs[keep];pixels=np.floor((q[:,:2]-origin[:2])/v).astype(int);canvas=np.full((ny,nx,3),245,np.uint8);good=(pl>=0)&(pixels[:,0]>=0)&(pixels[:,0]<nx)&(pixels[:,1]>=0)&(pixels[:,1]<ny);canvas[ny-1-pixels[good,1],pixels[good,0]]=palette[pl[good]%len(palette)];Image.fromarray(canvas).resize((nx*5,ny*5),Image.Resampling.NEAREST).save(folder/'labeled_observed_points_top.png')
 panel=Image.new('RGB',(950,700),'white');draw=ImageDraw.Draw(panel);draw.text((20,10),f'L{f}: room regions | anisotropic potential field',(20,20,20));im=image.copy();im.thumbnail((460,640));panel.paste(im,(10,45));im=Image.open(folder/'potential_field.png');im.thumbnail((460,640));panel.paste(im,(480,45));panels.append(panel)
 metric={'floor':f,'free_space_cluster_count':len(records),'surface_label_count':int(len(np.unique(labs[labs>=0]))),'room_area_records':records,'tiny_regions_under_half_m2':sum(r['area_m2']<.5 for r in records),'labels_with_multiple_major_components':sum(r['components_ge_half_m2']>1 for r in records),'no_ground_truth':True};metrics.append(metric);(folder/'topology_audit.json').write_text(json.dumps(metric,indent=2))
combined=Image.new('RGB',(950,2100),'white')
for i,panel in enumerate(panels):combined.paste(panel,(0,i*700))
combined.save(root/f'{a.variant}_three_floor_review.png');(root/f'{a.variant}_topology_summary.json').write_text(json.dumps(metrics,indent=2));print([(m['floor'],m['free_space_cluster_count'],m['tiny_regions_under_half_m2'],m['labels_with_multiple_major_components']) for m in metrics])
