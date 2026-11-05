"""Standalone sampler check; no estimator, ROS master or bag playback."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import numpy as np
import rosbag
import rospy
from sensor_msgs import point_cloud2
P=Path(__file__).resolve().parent
W=Path("/workspace/cbs_gtsam4.3")
R=P.parent
out=P/"sampler_build_2"
out.mkdir(exist_ok=False)
bag=W/"src/datasets/GrandTour/2024-10-01-11-29-55_hesai.bag"
stamp=1727775008775065899
with rosbag.Bag(str(bag)) as source:
    messages=list(source.read_messages(topics=["/boxi/hesai/points"],
        start_time=rospy.Time(stamp//10**9,stamp%10**9),
        end_time=rospy.Time(stamp//10**9,stamp%10**9+1)))
assert len(messages)==1 and messages[0][1].header.stamp.to_nsec()==stamp
msg=messages[0][1]
pts=np.array(list(point_cloud2.read_points(msg,field_names=("x","y","z"),skip_nans=False)),dtype="<f4")
pts.tofile(out/"raw_xyz.bin")
args=json.loads((R/"k16b_build_5/commands.json").read_text())[0]["command"]
args=[str(P/"sampler_k16e_2.cpp") if x.endswith("/test_main_lifecycle.cpp") else x
      for x in args if x not in ("-lgtest","-lgtest_main") and not x.endswith("/ExactGpuSnapshot.cpp")]
args[args.index("-o")+1]=str(out/"sampler")
with (out/"build.log").open("x") as log:
    rc=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT).returncode
(out/"command.json").write_text(json.dumps(dict(command=args,returncode=rc),indent=2))
if rc:
    print((out/"build.log").read_text()[-8000:]);raise SystemExit(rc)
env=os.environ.copy()
env.pop("LD_PRELOAD",None)
env["LD_LIBRARY_PATH"]=":".join(map(str,[
    W/"runs/native_covariance_cache_cpi3_20261019/release_v2",
    W/"runs/shared_linearization_s4_provider_20261012/release_2",
    W/"devel/lib",Path("/opt/ros/noetic/lib"),Path("/usr/local/cuda/lib64")]))
with (out/"run.log").open("x") as log:
    subprocess.run([str(out/"sampler"),str(out/"raw_xyz.bin"),str(out/"selected")],
                   env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
res=dict(raw_stamp_ns=stamp,raw_points=len(pts),raw_xyz_sha256=hashlib.sha256(pts.tobytes()).hexdigest(),
         note="same raw XYZ and default RNG each call; original sampler DSO; not a source replay",threads={})
for n in (2,1):
    paths=sorted((out/"selected").glob(str(n)+"_*.txt"))
    sets=[set(np.loadtxt(p,dtype=np.int64).tolist()) for p in paths]
    hashes=[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]
    res["threads"][n]=dict(repeats=len(paths),unique_hashes=len(set(hashes)),sha256=hashes,
        counts=[len(s) for s in sets],
        intersection_with_first=[len(sets[0]&s) for s in sets])
with (out/"result.json").open("x") as f: json.dump(res,f,indent=2)
print(json.dumps(res,indent=2))
