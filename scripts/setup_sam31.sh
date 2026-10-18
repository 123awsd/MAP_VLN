#!/usr/bin/env bash
# Reproduce the SAM3.1 front end without changing the Boxer environment.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"
[[ -x .envs/boxer/bin/python ]] || { echo 'Run setup_host_envs.sh first'; exit 2; }
if [[ ! -x .envs/sam31/bin/python ]]; then
  .envs/boxer/bin/python -m venv --system-site-packages .envs/sam31
fi
.envs/sam31/bin/python -m pip install --no-deps -r requirements/sam31-extra-pip.txt
.envs/sam31/bin/python - <<'PY'
import json,subprocess,hashlib,os
from pathlib import Path
from huggingface_hub import hf_hub_download
root=Path.cwd();cfg=json.loads((root/'config/sam31_backend_manifest.json').read_text());src=cfg['sam_source'];target=root/src['local_directory']
if not target.exists():
 subprocess.run(['git','clone','--filter=blob:none','--no-checkout',src['url'],str(target)],check=True)
 subprocess.run(['git','-C',str(target),'checkout','--detach',src['commit']],check=True)
elif (target/'source_revision.json').exists():
 assert json.loads((target/'source_revision.json').read_text())['revision']==src['commit']
else:
 assert subprocess.check_output(['git','-C',str(target),'rev-parse','HEAD'],text=True).strip()==src['commit']
weights=cfg['weights'];folder=root/'models/sam31';folder.mkdir(exist_ok=True,parents=True);dest=folder/weights['filename']
if not dest.exists():
 cached=Path(hf_hub_download(weights['repository'],weights['filename'],revision=weights['revision']))
 import shutil
 shutil.copyfile(cached,dest)
h=hashlib.sha256()
with dest.open('rb')as fp:
 for block in iter(lambda:fp.read(8*1024*1024),b''):h.update(block)
assert h.hexdigest()==weights['sha256'],'checkpoint hash mismatch'
(folder/'manifest.json').write_text(json.dumps(weights,indent=2))
# Patch only a clean pinned checkout, or recognize an already-applied patch.
patch=root/cfg['boxer_patch']['path'];boxer=root/'third_party/boxer'
if subprocess.run(['git','-C',str(boxer),'apply','--reverse','--check',str(patch)],capture_output=True).returncode!=0:
 assert subprocess.check_output(['git','-C',str(boxer),'rev-parse','HEAD'],text=True).strip()==cfg['boxer_patch']['base_commit'],'Use the pinned clean Boxer checkout, not another revision'
 subprocess.run(['git','-C',str(boxer),'apply','--check',str(patch)],check=True)
 subprocess.run(['git','-C',str(boxer),'apply',str(patch)],check=True)
print('SAM3.1 front end ready; image detector only, no video tracking')
PY
