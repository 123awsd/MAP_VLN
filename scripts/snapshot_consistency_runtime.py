#!/usr/bin/env python3
"""Hash the full shared ROS source tree, including unsnapshotted dependencies."""
import hashlib,json,sys
from pathlib import Path
workspace=Path('/workspace/falcon_ws');files={}
for root in [workspace/'src/FALCON',workspace/'src/pre_map_bridge']:
 for p in sorted(root.rglob('*')):
  if p.is_file() and (p.suffix in ['.cpp','.h','.hpp','.yaml','.xml','.launch'] or p.name=='CMakeLists.txt'):
   files[str(p.relative_to(workspace))]=hashlib.sha256(p.read_bytes()).hexdigest()
binaries={}
for p in sorted((workspace/'devel/lib').rglob('*')):
 if p.is_file() and (p.suffix=='.so' or p.name in ['exploration_node','traj_server']):binaries[str(p.relative_to(workspace))]=hashlib.sha256(p.read_bytes()).hexdigest()
Path(sys.argv[1]).write_text(json.dumps({'source_sha256':files,'binary_sha256':binaries},indent=2))
