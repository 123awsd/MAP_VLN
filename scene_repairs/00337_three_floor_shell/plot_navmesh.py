from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
p=Path(__file__).parent;d=np.load(p/'navmesh.npz');v=d['vertices_habitat'];i=d['triangles'];S=np.array([[0,0,-1],[-1,0,0],[0,1,0]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);vf=(v-origin)@S.T+[0,0,1];t=vf[i];h=t[:,:,2].mean(1);spread=np.ptp(t[:,:,2],axis=1)
a=np.load(p.parents[1]/'outputs/exploration_method_v4/trajectory_assessed.npy')
fig,ax=plt.subplots(1,2,figsize=(12,8))
for k,(low,high,title)in enumerate([(2.5,2.95,'Floor 2 original walkable surface'),(5.5,5.85,'Floor 3 original walkable surface')]):
 mask=(h>low)&(h<high)&(spread<.18);ax[k].add_collection(PolyCollection(t[mask,:,:2],facecolors='#d0d6df',edgecolors='none'))
 stair=(h>2.95)&(h<5.5)&(spread>.12);ax[k].add_collection(PolyCollection(t[stair,:,:2],facecolors='#e9aa4a',edgecolors='#c08020',alpha=.8))
 q=a[(a[:,0]>310)&(a[:,0]<350)];ax[k].plot(q[:,1],q[:,2],'r-',lw=1);ax[k].set(xlim=(-7,8),ylim=(-3,18),title=title,xlabel='FALCON x',ylabel='FALCON y');ax[k].set_aspect('equal');ax[k].grid(alpha=.2)
fig.tight_layout();fig.savefig(p/'navmesh_stair_reference.png',dpi=150)
np.savez(p/'navmesh_falcon.npz',vertices=vf,triangles=i)
print('stairs bounds',t[stair].reshape(-1,3).min(0),t[stair].reshape(-1,3).max(0))
