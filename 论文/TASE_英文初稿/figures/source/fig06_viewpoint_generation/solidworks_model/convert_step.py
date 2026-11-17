import sys,json
from pathlib import Path
sys.path.insert(0,'/tmp/step_conversion_deps')
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
out=Path(__file__).resolve().parent
reader=STEPControl_Reader();status=reader.ReadFile('/home/uav/桌面/装配体1.STEP');assert status==IFSelect_RetDone
reader.TransferRoots();shape=reader.OneShape();box=Bnd_Box();BRepBndLib.Add_s(shape,box)
count=0;it=TopExp_Explorer(shape,TopAbs_SOLID)
while it.More():count+=1;it.Next()
print('SOLIDS',count,'BOUNDS',(*box.CornerMin().Coord(),*box.CornerMax().Coord()),flush=True)
mesh=BRepMesh_IncrementalMesh(shape,.18,False,.22,True);mesh.Perform()
writer=StlAPI_Writer();writer.ASCIIMode=False;assert writer.Write(shape,str(out/'assembly_complete.stl'))
(out/'step_report.json').write_text(json.dumps({'source':'/home/uav/桌面/装配体1.STEP','solids':count,'bounds_mm':(*box.CornerMin().Coord(),*box.CornerMax().Coord()),'linear_deflection_mm':.18,'angular_deflection_rad':.22},indent=2))
print('EXPORTED',out/'assembly_complete.stl',flush=True)
