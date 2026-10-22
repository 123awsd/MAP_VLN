"""Invalidate rays blocked by explicitly non-observable axis-aligned safety shells.
Depth is optical z. Zero means no measurement, never a max-range free ray.
"""
import numpy as np

def invalidate_safety_shell_depth(depth, position, rotation_optical, shells, fx=320., fy=320., cx=320., cy=240.):
    depth=np.asarray(depth,dtype=np.float32)
    valid=np.isfinite(depth)&(depth>0)
    yy,xx=np.indices(depth.shape)
    directions=np.stack(((xx-cx)/fx,(yy-cy)/fy,np.ones(depth.shape)),axis=-1)@np.asarray(rotation_optical).T
    blocked=np.zeros(depth.shape,bool);position=np.asarray(position)
    for lo,hi in shells:
        lo=np.asarray(lo);hi=np.asarray(hi)
        near=np.full(depth.shape,-np.inf);far=np.full(depth.shape,np.inf);possible=np.ones(depth.shape,bool)
        for axis in range(3):
            d=directions[:,:,axis];parallel=np.abs(d)<1e-12
            possible&=~parallel|((position[axis]>=lo[axis])&(position[axis]<=hi[axis]))
            safe=np.where(parallel,1.,d)
            t1=(lo[axis]-position[axis])/safe;t2=(hi[axis]-position[axis])/safe
            near=np.maximum(near,np.where(parallel,-np.inf,np.minimum(t1,t2)))
            far=np.minimum(far,np.where(parallel,np.inf,np.maximum(t1,t2)))
        # One millimetre covers depth_u16 quantization; does not enlarge spatial bounds.
        blocked|=valid&possible&(far>=np.maximum(near,0))&(np.maximum(near,0)<=depth+.001)
    return np.where(valid&~blocked,depth,0).astype(np.float32),blocked
