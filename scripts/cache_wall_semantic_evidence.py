"""Attach existing predicted RGB semantics to observed 3D points, per-frame votes.

Predictions are evidence, never ground truth. Avoid flattened XY semantic votes
that transfer table/ceiling labels onto the wall at the same horizontal cell.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'outputs/semantic_structure_trial'
OUT=ROOT/'outputs/wall_refinement'

def main():
    geo=np.load(ROOT/'outputs/room_quality_ceiling/geometry.npz')
    # ADE20K tangible clutter. Paintings, doors and windows are separate and do
    # not veto the building wall behind them.
    negative_ids=[7,10,15,19,23,24,30,31,32,33,35,37,38,39,41,44,45,47,49,50,53,55,56,57,59,62,64,65,69,70,71,73,75,89,95,97,99,107,110,118,124,129]
    data={};trees=[];votes=[]
    for f in range(3):
        p=geo[f'p{f}'];trees.append(cKDTree(p));votes.append(np.zeros((len(p),4),np.uint16))
    frames=sorted((SRC/'rgbd').glob('*.npz'))
    for frame_no,path in enumerate(frames):
        d=np.load(path);s=np.load(SRC/'semantics'/path.name)
        dep=d['depth'];h,w=dep.shape;v,u=np.mgrid[0:h:3,0:w:3];v,u=v.ravel(),u.ravel()
        z=dep[v,u];label=s['labels'][v,u];conf=s['confidence'][v,u]
        good=(z>.1)&(z<4.95)&(conf>.6)
        v,u,z,label=v[good],u[good],z[good],label[good]
        xyz=np.c_[(u-w/2)*z/(w/2),(v-h/2)*z/(w/2),z]@d['rotation'].T+d['position']
        classes=np.full(len(label),3,int);classes[label==0]=0;classes[np.isin(label,negative_ids)]=1;classes[label==5]=2
        for f,ground in enumerate([-.155,2.595,5.595]):
            good=(xyz[:,2]>ground-.05)&(xyz[:,2]<[2.495,5.495,8.05][f])
            if not good.any():continue
            distance,idx=trees[f].query(xyz[good],k=4,workers=2)
            for category in range(4):
                near=(distance<.075)&(classes[good,None]==category)
                # At most one vote per recorded view and class for a point.
                ii=np.unique(idx[near]);votes[f][ii,category]+=1
        if frame_no%20==0: print('semantic frames',frame_no+1,'/',len(frames),flush=True)
    for f in range(3):
        data[f'votes{f}']=votes[f]
        print('floor',f+1,'voted points',int((votes[f].sum(1)>0).sum()),'negative',int((votes[f][:,1]>=2).sum()),flush=True)
    np.savez_compressed(OUT/'semantic_point_votes.npz',**data)

if __name__=='__main__':main()
