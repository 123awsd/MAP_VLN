#!/usr/bin/python3
"""Register existing explanatory geometry to the official HM3D floor plate.

The temporary Blender render is solely a calibration image. The published
preview uses the downloaded, unchanged official PNG. Run with --render to
regenerate /tmp/fig8_calibration_nadir.png; then run compose_official.py.
"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import cv2
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
ASSETS=OUT/'assets/hm3d_official'
parser=argparse.ArgumentParser()
parser.add_argument('--render',action='store_true')
args=parser.parse_args()
calibration=Path('/tmp/fig8_calibration_nadir.png')
if args.render:
    script=Path('/tmp/fig8_official_calibration.py')
    scene=OUT.parent/'multiroom_redraw/fig8_textured_scene.blend'
    script.write_text(f'''import bpy
bpy.ops.wm.open_mainfile(filepath={str(scene)!r})
s=bpy.context.scene
s.camera.location=(-4.6,2.05,24)
s.camera.rotation_euler=(0,0,0)
s.camera.data.ortho_scale=13.4
s.render.resolution_x=s.render.resolution_y=1400
s.render.resolution_percentage=100
s.cycles.samples=8
s.render.threads_mode='FIXED';s.render.threads=8
s.render.filepath={str(calibration)!r}
bpy.ops.render.render(write_still=True)
''')
    subprocess.run(['blender','-b','-t','8','--python',str(script)],check=True)

a=cv2.rotate(cv2.imread(str(calibration),-1),cv2.ROTATE_180)
b=cv2.imread(str(ASSETS/'topdown_floors.png'),-1)
sift=cv2.SIFT_create(nfeatures=16000)
ka,da=sift.detectAndCompute(cv2.cvtColor(a,cv2.COLOR_BGRA2GRAY),a[:,:,3])
kb,db=sift.detectAndCompute(cv2.cvtColor(b,cv2.COLOR_BGRA2GRAY),b[:,:,3])
good=[m for m,n in cv2.BFMatcher().knnMatch(da,db,k=2) if m.distance<.7*n.distance]
src=np.float32([ka[m.queryIdx].pt for m in good])
dst=np.float32([kb[m.trainIdx].pt for m in good])
M,mask=cv2.estimateAffinePartial2D(src,dst,method=cv2.RANSAC,ransacReprojThreshold=2)
res=np.linalg.norm(cv2.transform(src[None],M)[0]-dst,axis=1)[mask.ravel()>0]
B=np.array([[0,-1400/13.4,1399-1400*(.5+4.6/13.4)],
            [-1400/13.4,0,1399-1400*(.5+2.05/13.4)],[0,0,1]])
T=M@B
old=json.loads((OUT.parent/'multiroom_redraw/assets/projection.json').read_text())
geo=json.loads((OUT/'assets/sequence_geometry.json').read_text())
execution=ROOT/'PRE_MAP_VLN/outputs/stage2_3d/tasks/00166_cushion_recovery_20261024/execution/habitat_execution.json'
observations=json.loads(execution.read_text())['observations']
poses=old['poses_falcon'];poses['qpre']=[-2.65,-5.25,1]
original={k:list(v) for k,v in poses.items()}
for name,index in [('q1',0),('q2',1),('q3',3)]:
    poses[name]=observations[index]['pose'][:3]
routes=geo['route_falcon']
for route in routes.values():
    for i,p in enumerate(route):
        for name in ('q1','q2','q3'):
            if np.linalg.norm(np.array(p)-original[name])<1e-5:
                route[i]=poses[name]

def project(p):return (T@np.array([p[0],p[1],1])).tolist()
result=dict(
    source_url='https://habitatwebsite.s3.amazonaws.com/website-visualization/00166-RaYrxWt5pR1/topdown_floors.png',
    source_page='https://aihabitat.org/datasets/hm3d/00166-RaYrxWt5pR1/',
    sha256=hashlib.sha256((ASSETS/'topdown_floors.png').read_bytes()).hexdigest(),
    image_size=[1280,1269],falcon_xy_to_official_pixel=T.tolist(),
    registration=dict(method='SIFT + similarity RANSAC; nadir render used only for calibration; official pixels retained',
                      matches=len(good),inliers=int(mask.sum()),median_px=float(np.median(res)),
                      p95_px=float(np.percentile(res,95))),
    poses={k:project(v) for k,v in poses.items()},
    route={k:[project(p) for p in r] for k,r in routes.items()},
    anchors={k:project(v) for k,v in dict(armchair=[-4.70,-7.55,.65],
                  cushion=[-4.03,-3.43,.75],sofa=[-4.62,-3.43,.58]).items()},
    route_note='Authored explanatory route, not algorithm output. Observation endpoints use native Habitat camera poses.',
    observation_pose_source=str(execution),
    observation_indices={'q1':0,'q2':1,'q3':3})
(ASSETS/'geometry.json').write_text(json.dumps(result,indent=2)+'\n')
print(f'Registration: {mask.sum()} inliers, median {np.median(res):.3f} px, P95 {np.percentile(res,95):.3f} px')
