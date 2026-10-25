"""Fixed-time MINCO S4NU interpolation for this explanatory figure.

Dense linear-algebra transcription of the boundary/continuity equations in
PRE_MAP_VLN/real_fly/vendor/dls_ws_src/super_planner/src/utils/minco.cpp.
This does not run the production corridor or time-allocation optimizer.
"""
import math
import numpy as np
from scipy.linalg import solve

def basis(t,derivative=0):
    return np.array([0 if k<derivative else math.factorial(k)/math.factorial(k-derivative)*t**(k-derivative) for k in range(8)])

def interpolate(points,speed=.65):
    points=np.asarray(points,float);N=len(points)-1
    duration=np.maximum(np.linalg.norm(np.diff(points,axis=0),axis=1)/speed,.7)
    A=np.zeros((8*N,8*N));b=np.zeros((8*N,points.shape[1]));row=0
    for d in range(4):
        A[row,:8]=basis(0,d);b[row]=points[0] if d==0 else 0;row+=1
    for i in range(N-1):
        for d in (4,5,6):
            A[row,8*i:8*i+8]=basis(duration[i],d);A[row,8*i+8:8*i+16]=-basis(0,d);row+=1
        A[row,8*i:8*i+8]=basis(duration[i]);b[row]=points[i+1];row+=1
        for d in range(4):
            A[row,8*i:8*i+8]=basis(duration[i],d);A[row,8*i+8:8*i+16]=-basis(0,d);row+=1
    for d in range(4):
        A[row,-8:]=basis(duration[-1],d);b[row]=points[-1] if d==0 else 0;row+=1
    coeff=solve(A,b).reshape(N,8,-1)
    samples=[]
    for i,t in enumerate(duration):
        tt=np.linspace(0,t,max(16,math.ceil(t*60)))
        samples.extend([basis(v)@coeff[i] for v in tt[:-1]])
    samples.append(points[-1]);samples=np.array(samples)
    continuity=max((float(np.linalg.norm(basis(duration[i],d)@coeff[i]-basis(0,d)@coeff[i+1]))
                    for i in range(N-1) for d in range(4)),default=0)
    return samples,dict(durations_s=duration.tolist(),coefficients_ascending=coeff.tolist(),
                        max_pvaj_joint_error=continuity,linear_residual=float(np.max(np.abs(A@coeff.reshape(-1,points.shape[1])-b))))
