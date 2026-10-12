from pathlib import Path
import hashlib,json,itertools
import numpy as np
p=Path(__file__).parent
S=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092])
boxes=[
 ('second_floor_ceiling_seal',[.32,3.30,5.13],[4.95,10.35,5.25],'按附近原天花板约5.15m加固封闭；真实楼梯投影在x<=0.13，保留开口'),
 ('third_east_middle',[1.42,5.70,5.49],[1.54,11.27,8.00],'封闭此前向东侧空域出界的中段'),
 ('third_east_south_step',[1.42,5.70,5.49],[3.40,5.82,8.00],'连接第三层东侧阶梯形外边界'),
 ('third_east_south',[3.28,-1.55,5.49],[3.40,5.82,8.00],'第三层南段外侧封闭'),
 ('third_east_north_step',[1.42,11.15,5.49],[3.40,11.27,8.00],'北段外侧连接，保留西侧室内走廊'),
 ('third_east_north',[3.28,11.15,5.49],[3.40,16.90,8.00],'第三层北段外侧封闭'),
 ('third_north',[-5.92,16.78,5.49],[3.40,16.90,8.00],'第三层北侧虚拟外边界'),
 ('third_south',[-5.92,-1.55,5.49],[3.40,-1.43,8.00],'第三层南侧虚拟外边界'),
 ('third_west',[-5.92,-1.55,5.49],[-5.80,16.90,8.00],'第三层西侧虚拟外边界'),
 ('third_ceiling',[-5.92,-1.55,7.96],[3.40,16.90,8.08],'按原天花板约7.96m补封顶，三层任务不开放第四层'),
]
boxes += [
 ('third_floor_south',[-5.92,-1.55,5.49],[3.40,7.85,5.58],'在原三层地面下方补封闭底面，保留原地面显示'),
 ('third_floor_north',[-5.92,9.10,5.49],[3.40,16.90,5.58],'补封闭底面北段'),
 ('third_floor_stair_west',[-5.92,7.85,5.49],[-4.95,9.10,5.58],'楼梯开口西侧底面'),
 ('third_floor_stair_east',[.35,7.85,5.49],[3.40,9.10,5.58],'楼梯开口东侧底面；楼梯矩形x[-4.95,.35],y[7.85,9.10]不封闭'),
]
faces=[(0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)]
objects=[]
for name,lo,hi,reason in boxes:
 v=np.array(list(itertools.product(*zip(lo,hi))),float)
 h=(v-[0,0,1])@S+origin
 lines=['mtllib virtual_wall.mtl','usemtl wall']+[f'v {x:.7f} {y:.7f} {z:.7f}'for x,y,z in h]
 for a,b,c,d in faces:lines +=[f'f {a+1} {c+1} {b+1}',f'f {a+1} {d+1} {c+1}']
 (p/f'{name}.obj').write_text('\n'.join(lines)+'\n');objects.append({'name':name,'mesh':f'{name}.obj','falcon_min':lo,'falcon_max':hi,'purpose':reason})
(p/'virtual_wall.mtl').write_text('newmtl wall\nKa 0.78 0.75 0.67\nKd 0.78 0.75 0.67\nKs 0 0 0\nd 1\nillum 1\n')
scene=p.parents[1]/'data/versioned_data/hm3d-0.2/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb'
config={'format':'habitat_scene_overlay.v1','scene_filename':scene.name,'source_glb_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'coordinate_frame':'mesh vertices: Habitat world, metres','reference_initial_sensor_habitat_xyz':origin.tolist(),'objects':objects,'note':'明确的虚拟场景封闭面；并非声称恢复原建筑精确外墙。原GLB与规划算法不变。'}
(p/'repair.json').write_text(json.dumps(config,ensure_ascii=False,indent=2));print('objects',len(objects))
