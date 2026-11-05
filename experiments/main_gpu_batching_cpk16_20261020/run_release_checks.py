"""Run the already-built candidate after resolving, not guessing, DSO symlinks."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import xml.etree.ElementTree as ET

W=Path("/workspace/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020/release_1"
O=R/"checks_1"
O.mkdir(exist_ok=False)
env=os.environ.copy()
env["LD_LIBRARY_PATH"]=":".join(map(str,[R,
 W/"runs/native_covariance_cache_cpi3_20261019/release_v2",
 W/"runs/shared_linearization_s4_provider_20261012/release_2",
 W/"runs/e1b4a_local_update_batching_20261012/release_2",
 W/"runs/e1b_local_profile_20261012/release_2",W/"devel/lib",
 Path("/opt/ros/noetic/lib"),Path("/usr/local/cuda/lib64")]))
env["OMP_NUM_THREADS"]="1"
for k in ["CPK16_MAIN_GPU_BATCHING","GLIM_SHARED_NATIVE_PROVIDER","GLIM_SHARED_NATIVE_SHADOW"]:
    env.pop(k,None)
loaded=subprocess.check_output(["ldd",str(R/"test_main_batch")],env=env,text=True)
(O/"loader.txt").write_text(loaded)
identities={}
for line in loaded.splitlines():
    if "=>" in line:
        path=Path(line.split("=>",1)[1].strip().split()[0])
        if path.exists():
            with path.open("rb") as f:h=hashlib.sha256(f.read()).hexdigest()
            identities[str(path)]=dict(resolved=str(path.resolve()),sha256=h)
cuda=next(Path(p) for p in identities if Path(p).name=="libgtsam_points_cuda.so.1")
assert cuda.resolve()==(W/"runs/e1b_local_profile_20261012/release_2/libgtsam_points_cuda.so.1.2.1").resolve()
assert str(R/"libgtsam_points.so.1") in identities
(O/"loaded_hashes.json").write_text(json.dumps(identities,indent=2)+"\n")
summary={}
for name,filt in [("main","MainBatch*"),("original","LocalLinearizer*"),("repeat","MainBatch*")]:
    cmd=[str(R/"test_main_batch"),"--gtest_filter="+filt,"--gtest_output=xml:"+str(O/(name+".xml"))]
    with (O/(name+".log")).open("x") as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
    summary[name]=dict(returncode=r.returncode,command=cmd)
    if (O/(name+".xml")).exists():summary[name]["gtest"]=ET.parse(O/(name+".xml")).getroot().attrib
    (O/"result.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(name,summary[name],flush=True)
    if r.returncode:
        print((O/(name+".log")).read_text()[-12000:],flush=True)
        raise SystemExit(r.returncode)
(O/"passed.json").write_text(json.dumps(dict(passed=True,tests=summary,scope="K16a only"),indent=2)+"\n")
