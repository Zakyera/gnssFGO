"""Validate diagnostic identities and preserved inputs; no experiments."""
from pathlib import Path
import csv
import hashlib
import json
import subprocess
import numpy as np
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
P=R/"k16e_source_2"
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
result={"protected":{},"source_repositories":{},"captures":{}}
for name in ("protected","input_hashes"):
    hashes=json.loads((R/(name+".json")).read_text())
    changed=[p for p,h in hashes.items() if sha(p)!=h]
    result["protected"][name]={"count":len(hashes),"changed":changed}
    assert not changed,changed
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
             status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
             diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    result["source_repositories"][name]={"unchanged":new==old}
    assert new==old,name
assert sha(R/"release_1/libgtsam_points.so.1.2.1")=="6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca"
result["probe_sha256"]=sha(P/"build/probe.so")
inputs=json.loads((P/"input_manifest.json").read_text())
original_inputs=json.loads((R/"k16d_source_1/input_manifest.json").read_text())
for topic,spec in inputs["topics"].items():
    assert spec["messages"]==original_inputs["topics"][topic]["messages"][:len(spec["messages"])]
for mode in ("reference","batched"):
    d=P/mode
    capture=json.loads((d/"capture_complete.json").read_text())
    cleanup=json.loads((d/"cleanup.json").read_text())
    assert capture["passed"] and cleanup["all_exited"]
    assert all(rc==0 for rc in cleanup["returncodes"].values()),cleanup
    assert not (d/"failed.txt").exists()
    maps=(d/"maps.txt").read_text()
    assert "/k16e_source_2/build/probe.so" in maps
    assert "/e1b_local_profile_20261012/release_2/libgtsam_points_cuda.so.1.2.1" in maps
    data=list(csv.DictReader((d/"receipts.csv").open()))
    for kind,topic in (("lidar","/boxi/hesai/points"),("imu","/boxi/cpt7/imu")):
        assert [(int(r["seq"]),int(r["stamp_ns"])) for r in data if r["kind"]==kind]==[
            tuple(r[:2]) for r in inputs["topics"][topic]["messages"]]
    params=json.loads((d/"effective_params.json").read_text())
    baseline=json.loads((R/"k16d_source_1/reference/effective_params.json").read_text())
    changed=[k for k in set(params)|set(baseline) if params.get(k)!=baseline.get(k)]
    expected={"config_path","glim_dump_path"}
    if mode=="batched": expected.add("glim_factor_graph_inspector_recording_id")
    assert set(changed)==expected, changed
    result["captures"][mode]=dict(capture=capture,cleanup=cleanup,parameter_differences=changed)
    log=(d/"node.log").read_text()
    assert "exception was caught" not in log
analysis=json.loads((P/"first_update_analysis.json").read_text())
assert analysis["exact_input_equations_passed"]
assert analysis["native_count"]==11
assert all(v==0 for c in analysis["calls"] for v in c["poses"].values())
result["first_update_equations"]=analysis["equations"]
a=np.fromfile(P/"reference/probe/call_0/actual_before/source_points.bin",dtype="<f4").reshape(-1,3)
b=np.fromfile(P/"batched/probe/call_0/actual_before/source_points.bin",dtype="<f4").reshape(-1,3)
dist,idx=cKDTree(b).query(a)
result["real_geometry"]=dict(points=len(a),exact_xyz_matches=int(sum(dist==0)),
    exact_match_fraction=float(np.mean(dist==0)),
    nearest_neighbor_median_p95_max_m=np.quantile(dist,[.5,.95,1]).tolist(),
    caution="array-index/lexicographic max distances in raw analysis are NOT physical errors")
sampler=json.loads((P/"sampler_build_2/result.json").read_text())
assert sampler["threads"]["2"]["unique_hashes"]>1 and sampler["threads"]["1"]["unique_hashes"]==1
result["sampler"]=sampler
result["scope"]="first real GPU equations and upstream input variability; no full-source preservation promotion"
result["passed"]=True
with (H/"RESULT_K16E.json").open("x") as f: json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
