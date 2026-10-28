from pathlib import Path
import numpy as np
r=Path(__file__).resolve().parent;zs=[dict(np.load(r/'stage_corrected_results'/n/'poses.npz')) for n in ['01_laptop','02_toolbox','03_microwave','04_cup']];ts=np.concatenate([z['times'] for z in zs]);_,ix=np.unique(ts,return_index=True);ix=ix[np.argsort(ts[ix])];arrays={k:np.concatenate([z[k] for z in zs])[ix][::4] for k in zs[0]};out=r/'odom_corrected_results/full_trajectory';out.mkdir(parents=True,exist_ok=True);np.savez_compressed(out/'poses.npz',**arrays)
s=(r/'odom_correction/map_support.py').read_text().replace("R=Path(__file__).resolve().parent;root=R.parent","R=Path(__file__).resolve().parent/'odom_correction';root=R.parent").replace("['01_laptop','02_toolbox','03_microwave','04_cup']","['full_trajectory']")
(r/'support_full_trajectory.py').write_text(s)
print('reference camera samples',len(arrays['times']),arrays['times'][[0,-1]])
