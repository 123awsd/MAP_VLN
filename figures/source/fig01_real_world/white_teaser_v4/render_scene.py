"""Render measured RGB-D points for the white-background figure."""
from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
UPSTREAM = ROOT / '画图/主图/local_vggt'
OUT = HERE / 'assets'
OUT.mkdir(exist_ok=True)
source = UPSTREAM / 'mapping_rgbd/mapping_rgbd.ply'
with source.open('rb') as f:
    header = []
    while True:
        row = f.readline(); header.append(row)
        if row.strip() == b'end_header': break
    offset = f.tell()
count = int(next(x for x in b''.join(header).decode().splitlines() if x.startswith('element vertex')).split()[-1])
dtype = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('r','u1'),('g','u1'),('b','u1'),('a','u1')])
raw = np.memmap(source,mode='r',offset=offset,dtype=dtype,shape=(count,))
points = np.column_stack([raw['x'],raw['y'],raw['z']])
colors = np.column_stack([raw['r'],raw['g'],raw['b']])
valid = ((points[:,0]>-43)&(points[:,0]<21)&(points[:,1]>-.25)&(points[:,1]<2.65)&(points[:,2]>-18)&(points[:,2]<10))
points,colors = points[valid],colors[valid]
odom = np.load(UPSTREAM / 'odom_correction/odom.npy')
events = json.loads((UPSTREAM / 'odom_correction/events.json').read_text())
route = odom[(odom[:,0]>=events[0]['bag_sec'])&(odom[:,0]<=events[7]['bag_sec'])][::8]
to_view = lambda p: np.asarray(p)[...,[0,2,1]] * [1,1,-1]
centers = to_view(route[:,1:4])
cal = ROOT / '真机实验素材/成功场次/实验室4任务加主动搜索/02_地图与场景/episode/calibration/camera_extrinsic.json'
tbc = np.array(json.loads(cal.read_text())['body_to_color_optical'])


def quaternion_matrix(q):
    x,y,z,w = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


floor = np.array([[-43,-.125,10],[21,-.125,10],[21,-.125,-18],[-43,-.125,-18]])
for azimuth in [-10, 10]:
    width,height = 3200,1250
    az,el = np.deg2rad([azimuth,50])
    right = np.array([np.cos(az),0,np.sin(az)])
    up = np.array([-np.sin(az)*np.sin(el),np.cos(el),np.cos(az)*np.sin(el)])
    basis = np.stack([right,-up],axis=1)
    bounds = np.vstack([floor@basis,centers@basis])
    lo,hi = bounds.min(0),bounds.max(0)
    scale = min((width-140)/(hi[0]-lo[0]),(height-110)/(hi[1]-lo[1]))
    midpoint=(lo+hi)/2
    def project(p):return (np.asarray(p)@basis-midpoint)*scale+[width/2,height/2]
    uv=np.rint(project(points)).astype(int)
    keep=(uv[:,0]>=2)&(uv[:,0]<width-3)&(uv[:,1]>=2)&(uv[:,1]<height-3)
    uv,cc=uv[keep],colors[keep]
    order=np.argsort(points[keep]@np.cross(right,up));uv,cc=uv[order],cc[order]
    rgba=np.zeros((height,width,4),dtype='uint8')
    for dx,dy in [(0,0),(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,-1)]:
        rgba[uv[:,1]+dy,uv[:,0]+dx,:3]=cc
        rgba[uv[:,1]+dy,uv[:,0]+dx,3]=255
    image=Image.fromarray(rgba)
    stem=f'scene_az{azimuth}'
    image.save(OUT/f'{stem}.png')
    markers=[]
    for label,index in [('A',2),('1',4),('2',5),('3',6),('4',7)]:
        row=odom[np.argmin(abs(odom[:,0]-events[index]['bag_sec']))]
        rot=quaternion_matrix(row[4:8]);camera_center=row[1:4]+rot@tbc[:3,3]
        optical_rotation=rot@tbc[:3,:3]
        corners=np.array([[-.505,-.405,1],[.688,-.405,1],[.688,.388,1],[-.505,.388,1]])*1.45
        world_corners=corners@optical_rotation.T+camera_center
        markers.append({'id':label,'point':project(to_view(row[1:4])).tolist(),
                        'camera':project(to_view(camera_center)).tolist(),
                        'frustum_corners':project(to_view(world_corners)).tolist(),
                        'bag_seconds':float(row[0])})
    grid=[]
    for x in range(-40,21,5):grid.append(project([[x,-.125,10],[x,-.125,-18]]).tolist())
    for y in range(-10,19,5):grid.append(project([[-43,-.125,-y],[21,-.125,-y]]).tolist())
    meta={'size':[width,height],'azimuth_degrees':azimuth,'elevation_degrees':50,
          'scale_pixels_per_meter':float(scale),'floor_polygon':project(floor).tolist(),'floor_grid':grid,
          'route':project(centers).tolist(),'route_bag_seconds':route[:,0].tolist(),'markers':markers,
          'recovery_bag_seconds':[events[2]['bag_sec'],events[4]['bag_sec']],
          'source_cloud':str(source.relative_to(ROOT)),
          'display_crop_world_xyz':{'x':[-43,21],'y':[-10,18],'z':[-.25,2.65]},
          'floor_grid_note':'Coordinate reference plane at archived estimated floor z=-0.125 m, not an additional measured surface.',
          'frusta_note':'Calibrated optical-camera poses at recorded body pose; uniform optical depth 1.45 m for display.'}
    (OUT/f'{stem}.json').write_text(json.dumps(meta,indent=2))
    preview=Image.new('RGB',image.size,'white');preview.paste(image,(0,0),image)
    draw=ImageDraw.Draw(preview);draw.line([tuple(p) for p in meta['route']],fill='#236D7D',width=8)
    preview.thumbnail((1600,625));preview.save(HERE/f'view_{azimuth}.jpg',quality=95)
print('Rendered two 3D scene views with calibrated observation poses.')
