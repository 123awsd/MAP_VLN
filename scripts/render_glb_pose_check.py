import argparse, bpy, mathutils, sys
from pathlib import Path
import numpy as np

S=np.asarray([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])
C=np.diag([1.,-1.,-1.])
def qmat(q):
 x,y,z,w=q; return np.asarray([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def mquat(m):
 t=np.trace(m); s=np.sqrt(t+1)*2; w=.25*s; x=(m[2,1]-m[1,2])/s; y=(m[0,2]-m[2,0])/s; z=(m[1,0]-m[0,1])/s; return mathutils.Quaternion((w,x,y,z))

def main():
    p=argparse.ArgumentParser(); p.add_argument('--scene',required=True); p.add_argument('--pose',required=True); p.add_argument('--out',required=True)
    argv=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else sys.argv[1:]; a=p.parse_args(argv)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=str(Path(a.scene).resolve()))
    with np.load(a.pose) as d:
        pos=np.asarray(d['position'],dtype=float); q=np.asarray(d['orientation_xyzw'],dtype=float)
    rot=qmat(q) @ C
    # GLB/scene graph and recorded Falcon poses use the same z-up world frame.
    cam_data=bpy.data.cameras.new('CheckCamera'); cam=bpy.data.objects.new('CheckCamera',cam_data); bpy.context.collection.objects.link(cam)
    cam.location=pos; cam.rotation_mode='QUATERNION'; cam.rotation_quaternion=mquat(rot)
    cam_data.lens=24; cam_data.sensor_width=32; bpy.context.scene.camera=cam
    sc=bpy.context.scene; sc.render.engine='BLENDER_EEVEE'; sc.render.resolution_x=640; sc.render.resolution_y=480; sc.render.resolution_percentage=100
    sc.render.image_settings.file_format='PNG'; sc.render.filepath=str(Path(a.out).resolve())
    world=bpy.data.worlds.new('CheckWorld') if not bpy.data.worlds else bpy.data.worlds[0]
    sc.world=world; world.use_nodes=True; world.node_tree.nodes['Background'].inputs['Color'].default_value=(0.12,0.12,0.12,1); world.node_tree.nodes['Background'].inputs['Strength'].default_value=0.8
    ld=bpy.data.lights.new('CheckSun','SUN'); lo=bpy.data.objects.new('CheckSun',ld); bpy.context.collection.objects.link(lo); lo.rotation_euler=(0.4,-0.5,0.3); ld.energy=4.0
    bpy.ops.render.render(write_still=True)
if __name__=='__main__': main()
