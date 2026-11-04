from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent/'collision.pcd'
b=P.read_bytes(); marker=b'DATA binary\n'; header,data=b.split(marker,1); points=np.frombuffer(data,dtype='<f4').reshape(-1,3)
x=np.arange(-10.35,.65,.05); y=np.arange(-11.35,-.75,.05); xx,yy=np.meshgrid(x,y); add=np.vstack([np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,z))) for z in (.505,1.895)]).astype('<f4'); pts=np.vstack((points,add)); h=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n'; P.write_bytes(h.encode()+pts.tobytes()); print('added virtual altitude limit points:',len(add))
