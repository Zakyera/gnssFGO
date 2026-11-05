"""Load pinned bridge/provider DSOs only; never construct or run an estimator."""
from pathlib import Path
import ctypes
import hashlib
import json
import os
import subprocess
import sys

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/native_covariance_cache_cpi3_20261019'
BASE=WS/'runs/shared_linearization_s4_provider_20261012/release_2'
BRIDGE=WS/'runs/local_revision_covariance_cpi1_20261019/libglim_ros.so'

def main():
    target=ROOT/'abi_load.json';assert not target.exists()
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(map(str,[ROOT/'release_v2',BRIDGE.parent,BASE,
        WS/'runs/e1b4a_local_update_batching_20261012/release_2',
        WS/'runs/e1b_local_profile_20261012/release_2',WS/'devel/lib',WS/'devel',
        '/opt/ros/noetic/lib','/usr/local/cuda/lib64']))
    code='''import ctypes,json
from pathlib import Path
ctypes.CDLL(%r,mode=ctypes.RTLD_GLOBAL)
paths=sorted(set(l.split()[-1] for l in Path('/proc/self/maps').read_text().splitlines() if 'libglim_ros.so' in l or 'libgtsam_points.so' in l))
print(json.dumps(paths))
''' % str(BRIDGE)
    p=subprocess.run([sys.executable,'-c',code],env=env,text=True,capture_output=True)
    result=dict(command=[sys.executable,'-c',code],returncode=p.returncode,stdout=p.stdout,stderr=p.stderr)
    target.write_text(json.dumps(result,indent=2)+'\n')
    assert p.returncode==0
    paths=json.loads(p.stdout)
    assert paths==sorted([str(BRIDGE),str(ROOT/'release_v2/libgtsam_points.so.1.2.1')]),paths
    print('unchanged CP-I1 bridge loads with new isolated provider')

if __name__=='__main__':main()
