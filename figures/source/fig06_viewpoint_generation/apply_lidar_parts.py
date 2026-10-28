from pathlib import Path
exec((Path(__file__).resolve().parent/'inspect_lidar_parts.py').read_text().split('result=[]')[0])
silver=next(i for i,m in enumerate(o.data.materials) if m and m.name.startswith('Photo brushed aluminium'));blue=next(i for i,m in enumerate(o.data.materials) if m and m.name.startswith('Lidar continuous blue'))
labels={}
for key,ids in groups.items():
 vs=[(o.matrix_world@o.data.vertices[i].co-center)/1.8 for i in ids];lo=[min(v[k] for v in vs) for k in range(3)];hi=[max(v[k] for v in vs) for k in range(3)]
 if hi[2]>.055 and max(abs(lo[0]),abs(hi[0]),abs(lo[1]),abs(hi[1]))<.08:
  # The large outer connected cage and top fasteners stay aluminium.
  labels[key]=silver if hi[1]-lo[1]>.060 or lo[2]>.076 else blue
for f in o.data.polygons:
 label=labels.get(root(f.vertices[0]))
 if label is not None:f.material_index=label
bpy.ops.wm.save_as_mainfile(filepath=str(p/'collision_explanation.blend'));bpy.context.scene.render.filepath=str(p/'collision_explanation.png');bpy.ops.render.render(write_still=True)
