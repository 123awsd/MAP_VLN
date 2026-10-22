"""Controlled topology checks for the new cell partition, without semantic claims."""
import numpy as np
from scipy import ndimage as nd
from segment_rooms_wall_structure import merge_unsupported_cells
r=.05;origin=np.zeros(2)
wall=np.zeros((80,100),bool);wall[10,10:91]=1;wall[70,10:91]=1;wall[10:71,10]=1;wall[10:71,90]=1
wall[10:71,50]=1;wall[35:46,50]=0
# A door connects air but an explicit room separator distinguishes the units.
assert nd.label(~wall)[0][30,30]==nd.label(~wall)[0][30,70]
sep=wall.copy();sep[35:46,50]=1
cells,_=merge_unsupported_cells(sep,wall,[],origin,r,1.2)
assert cells[30,30]!=cells[30,70]
# A long unsupported line through one room should not become a real partition.
blank=wall.copy();blank[10:71,50]=0;bar=blank.copy();bar[10:71,50]=1
ops=[{'kind':'unobserved_plane_span','length_m':3.,'a':[2.525,.525],'b':[2.525,3.525]}]
cells,links=merge_unsupported_cells(bar,blank,ops,origin,r,1.2)
assert cells[30,30]==cells[30,70] and links
assert not np.any((cells>0)&blank)
print('PASS: door connectivity separated from room identity; unsupported interior span merged; observed walls preserved')
