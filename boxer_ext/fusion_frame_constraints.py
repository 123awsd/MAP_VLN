"""Optional association constraints from exact same-frame proposal provenance."""
from collections import defaultdict
import itertools
import numpy as np


def proposal_cannot_links(rows, threshold=.1):
    groups=defaultdict(list);blocked=set()
    for i,row in enumerate(rows):groups[int(row['time_ns'])].append(i)
    for ids in groups.values():
        for i,j in itertools.combinations(ids,2):
            a=[float(rows[i][k]) for k in ('x1','y1','x2','y2')]
            b=[float(rows[j][k]) for k in ('x1','y1','x2','y2')]
            inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
            union=max(0,a[2]-a[0])*max(0,a[3]-a[1])+max(0,b[2]-b[0])*max(0,b[3]-b[1])-inter
            if union>0 and inter/union<threshold:blocked.add((i,j))
    return blocked


def constrained_components(matrix, threshold, cannot_links):
    n=matrix.shape[0];parent=list(range(n));members=[{i} for i in range(n)]
    if matrix.is_sparse:
        m=matrix.coalesce();indices=m.indices().numpy();values=m.values().numpy()
        edges=[(float(v),int(i),int(j)) for (i,j),v in zip(indices.T,values) if i<j and v>=threshold]
    else:
        m=matrix.detach().cpu().numpy();i,j=np.where(np.triu(m>=threshold,1));edges=[(float(m[a,b]),int(a),int(b)) for a,b in zip(i,j)]
    def find(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    rejected=0
    for _,i,j in sorted(edges,key=lambda e:(-e[0],e[1],e[2])):
        a,b=find(i),find(j)
        if a==b:continue
        if any((min(x,y),max(x,y)) in cannot_links for x in members[a] for y in members[b]):rejected+=1;continue
        parent[b]=a;members[a]|=members[b];members[b]=set()
    groups=defaultdict(list)
    for i in range(n):groups[find(i)].append(i)
    return list(groups.values()),rejected
