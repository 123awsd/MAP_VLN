"""CPU textured surface sampling and point z-buffer: official GLB, offline only."""
from pathlib import Path
import struct,json
import habitat_sim,magnum.trade as trade
import numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parents[1];O=R/'outputs/coverage700_wall_room_preview/room305_official';O.mkdir(exist_ok=True);path=R/'data/scene_datasets/hm3d/example/00337-CFVBbU9Rsyb/CFVBbU9Rsyb.basis.glb';raw=path.read_bytes();j=json.loads(raw[20:20+struct.unpack_from('<I',raw,12)[0]]);mgr=trade.ImporterManager();imp=mgr.load_and_instantiate('AnySceneImporter');imp.open_file(str(path));rng=np.random.default_rng(305)
S=np.array([[0,0,-1],[-1,0,0],[0,1,0]]);origin=np.array([9.714411735534668,-1.6005380153656006,-2.374816417694092]);points=[];colors=[]
for k in range(imp.mesh_count):
 m=imp.mesh(k);v=np.asarray(m.attribute(trade.MeshAttribute.POSITION));v=(np.column_stack([v[:,0],v[:,2],-v[:,1]])-origin)@S.T+[0,0,1];idx=np.asarray(m.indices).reshape(-1,3);tri=v[idx];lo=tri.min(1);hi=tri.max(1);keep=(hi[:,0]>-6.5)&(lo[:,0]<-1)&(hi[:,1]>3)&(lo[:,1]<7)&(hi[:,2]>5.4)&(lo[:,2]<8.5)
 if not keep.any():continue
 idx=idx[keep];tri=tri[keep];area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)*.5;counts=np.ceil(area*10000).astype(int);ii=np.repeat(np.arange(len(tri)),counts);u=np.sqrt(rng.random(len(ii)));v1=rng.random(len(ii));w=np.column_stack([1-u,u*(1-v1),u*v1]);p=np.einsum('ni,nij->nj',w,tri[ii]);mask=(p[:,0]>-6.5)&(p[:,0]<-1)&(p[:,1]>3)&(p[:,1]<7)&(p[:,2]>5.4)&(p[:,2]<8.5);p=p[mask];w=w[mask];ii=ii[mask]
 if not len(p):continue
 uv=np.asarray(m.attribute(trade.MeshAttribute.TEXTURE_COORDINATES));uv=np.einsum('ni,nij->nj',w,uv[idx[ii]])
 material=j['materials'][j['meshes'][k]['primitives'][0]['material']];tex=j['textures'][material['pbrMetallicRoughness']['baseColorTexture']['index']];source=tex.get('source',tex.get('extensions',{}).get('GOOGLE_texture_basis',{}).get('source'));im=np.array(imp.image2d(source).pixels);xy=np.floor((uv%1)*[im.shape[1],im.shape[0]]).astype(int);c=im[xy[:,1],xy[:,0],:3];points.append(p);colors.append(c)
p=np.concatenate(points);c=np.concatenate(colors);np.savez_compressed(O/'official_colored_local.npz',points=p,colors=c)
views=[('主体朝右',[-3.3,5.2,6.9],[-5.2,4.2,7.1]),('沿凸出方向',[-3.6,4.25,7.1],[-5.6,4.2,7.1]),('凸出位置回看',[-5.2,4.2,7.1],[-3.5,4.5,6.8]),('主体斜顶',[-2.4,5.4,6.8],[-4.4,5.3,6.7])]
font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',22);canvas=Image.new('RGB',(1600,1280),'white');draw=ImageDraw.Draw(canvas)
for k,(name,cp,t) in enumerate(views):
 cp=np.array(cp);forward=np.array(t)-cp;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);up=np.cross(right,forward);q=p-cp;z=q@forward;valid=z>.04;ids=np.flatnonzero(valid);xx=np.rint(400+400*(q[ids]@right)/z[ids]).astype(int);yy=np.rint(300-400*(q[ids]@up)/z[ids]).astype(int);ok=(xx>=0)&(xx<800)&(yy>=0)&(yy<600);ids=ids[ok];xx=xx[ok];yy=yy[ok];order=np.argsort(-z[ids]);img=np.full((600,800,3),225,np.uint8);img[yy[order],xx[order]]=c[ids[order]];Image.fromarray(img).save(O/f'cpu_view_{k}.png');canvas.paste(Image.fromarray(img),(k%2*800,k//2*640+40));draw.text((k%2*800+10,k//2*640+5),f'{name}｜官方纹理表面CPU投影',font=font,fill='black')
canvas.save(O/'official_cpu_views.png');(O/'cpu_manifest.json').write_text(json.dumps({'scene':str(path),'kind':'novel view CPU point z-buffer of official textured mesh, not actual capture','seed':305,'sample_density_per_m2':10000,'views':views,'repair_geometry_included':False,'limitations':['point raster may have sampling holes','no global clipping geometry outside ROI','camera positions are diagnostic, not safety verified']},ensure_ascii=False,indent=2))
