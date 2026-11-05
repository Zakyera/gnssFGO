"""Dynamic-load compatibility only. Does not instantiate GLIM or a solver."""
from pathlib import Path
import json
import os
import subprocess

W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
env=os.environ.copy()
env["LD_LIBRARY_PATH"]=":".join(map(str,[
 R/"release_1",W/"runs/shared_linearization_s4_provider_20261012/release_2",
 W/"runs/e1b4a_local_update_batching_20261012/release_2",
 W/"runs/e1b_local_profile_20261012/release_2",W/"devel/lib",
 Path("/opt/ros/noetic/lib"),Path("/usr/local/cuda/lib64")]))
code="""
import ctypes,json
from pathlib import Path
ctypes.CDLL('/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019/libglim_ros.so',mode=ctypes.RTLD_GLOBAL)
paths=sorted(set(l.split()[-1] for l in Path('/proc/self/maps').read_text().splitlines()
                 if 'libglim_ros.so' in l or 'libgtsam_points.so' in l or 'libgtsam_points_cuda.so' in l))
print(json.dumps(paths))
"""
command=["/usr/bin/python3","-c",code]
p=subprocess.run(command,env=env,capture_output=True,text=True)
result=dict(command=command,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr)
if p.returncode==0:
    paths=json.loads(p.stdout)
    result["passed"]=str(R/"release_1/libgtsam_points.so.1.2.1") in paths
else:result["passed"]=False
with (R/"abi_load.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result),flush=True)
if not result["passed"]:raise SystemExit(1)
