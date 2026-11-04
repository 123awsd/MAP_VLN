"""Render actual occupied-voxel geometry and export calibrated projections."""
from pathlib import Path
import json
import numpy as np
import bpy
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
ASSETS=HERE/'assets';ASSETS.mkdir(exist_ok=True)
blend=ROOT/'画图/主图/体素地图视觉试验_20261030/measured_voxel_study.blend'
bpy.ops.wm.open_mainfile(filepath=str(blend))
for name in ['Recorded body positions','Recorded recovery positions']:
    bpy.data.objects[name].hide_render=True
scene=bpy.context.scene
scene.cycles.use_denoising=False
scene.cycles.samples=48
scene.render.film_transparent=True
scene.render.image_settings.color_mode='RGBA'
camera=scene.camera
odom=np.load(ROOT/'画图/主图/local_vggt/odom_correction/odom.npy')
events=json.loads((ROOT/'画图/主图/local_vggt/odom_correction/events.json').read_text())
trace=odom[(odom[:,0]>=events[0]['bag_sec'])&(odom[:,0]<=events[7]['bag_sec'])][::8]
cal=ROOT/'真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/episode/calibration/camera_extrinsic.json'
tbc=np.array(json.loads(cal.read_text())['body_to_color_optical'])


def rotation(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def render(name,target,offset,ortho,size):
    camera.location=Vector(target)+Vector(offset)
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.type='ORTHO';camera.data.ortho_scale=ortho
    scene.render.resolution_x,scene.render.resolution_y=size
    scene.render.resolution_percentage=100
    bpy.context.view_layer.update()
    def project(point):
        uv=world_to_camera_view(scene,camera,Vector(point))
        return [float(uv.x*size[0]),float((1-uv.y)*size[1])]
    markers=[]
    for label,index in [('A',2),('1',4),('2',5),('3',6),('4',7)]:
        row=odom[np.argmin(abs(odom[:,0]-events[index]['bag_sec']))]
        rot=rotation(row[4:8]);cam=row[1:4]+rot@tbc[:3,3]
        corners=np.array([[-.505,-.405,1],[.688,-.405,1],[.688,.388,1],[-.505,.388,1]])*1.25
        world=corners@(rot@tbc[:3,:3]).T+cam
        markers.append({'id':label,'body':project(row[1:4]),'camera':project(cam),
                        'corners':[project(p) for p in world],'bag_seconds':float(row[0])})
    metadata={'size':size,'route':[project(p) for p in trace[:,1:4]],'times':trace[:,0].tolist(),
              'markers':markers,'recovery_seconds':[events[2]['bag_sec'],events[4]['bag_sec']],
              'scale_pixels_per_m':float(np.linalg.norm(np.array(project([-10,3,0]))-project([-5,3,0]))/5),
              'camera_location':list(camera.location),'camera_target':target,'ortho_scale':ortho,
              'source_blend':str(blend.relative_to(ROOT)),
              'source_geometry':'Archived 0.2 m occupied volume; exact voxel boundaries, height cut [-0.3,1.9] m.'}
    (ASSETS/f'{name}.json').write_text(json.dumps(metadata,indent=2))
    scene.render.filepath=str(ASSETS/f'{name}.png')
    bpy.ops.render.render(write_still=True)
    print('Finished',name,flush=True)


render('global',[-11,3,.3],[0,-68,45],71,[2600,900])
render('recovery',[11.5,2.5,.3],[0,-16,32],16,[1200,680])
