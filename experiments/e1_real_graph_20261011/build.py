#!/usr/bin/env python3
import json
import subprocess
from pathlib import Path
root=Path(__file__).resolve().parent
assert not (root/'private_build/libglim_ros.so').exists()
for i,cmd in enumerate(json.loads((root/'build_commands.json').read_text())):
    print('E1_REAL_BUILD',i,flush=True)
    with (root/('build_%d.log'%i)).open('x') as f:
        subprocess.run(cmd,cwd='/workspace/cbs_gtsam4.3/build/glim_ros',stdout=f,stderr=subprocess.STDOUT,check=True)
print('PRIVATE_BUILD_COMPLETE',flush=True)
