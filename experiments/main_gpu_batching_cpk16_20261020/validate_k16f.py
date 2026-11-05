"""Validate completed same-prepared-input gate; no estimator execution."""
from pathlib import Path
import csv
import hashlib
import json
import re
import sys
import numpy as np
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
P=R/"k16f_source_2"
D=P/"paired"
K=W/"runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared"
sys.path.insert(0,str(K/"transport_tools"))
from live_receiver_protocol import parse_belief

capture=json.loads((D/"capture_complete.json").read_text())
cleanup=json.loads((D/"cleanup.json").read_text())
assert capture["passed"] and capture["requested_window_s"]==8
assert cleanup["all_exited"] and all(x==0 for x in cleanup["returncodes"].values())
assert not (D/"failed.txt").exists() and not (D/"probe/failure.txt").exists()
rows=[json.loads(s) for s in (D/"probe/updates.jsonl").read_text().splitlines()]
assert [r["update"] for r in rows]==list(range(1,len(rows)+1))
assert all(r["passed"] for r in rows)
last=rows[-1]
assert last["retirement_updates"]>0 and last["initial_gpu_checks"]>0
assert last["prepared_hits"]==last["sync_calls"] and last["prepared_hits"]>0
assert last["pair_queries"]>500 and last["states"]<=51
assert last["stamp"]-rows[0]["stamp"]>6.5
assert last["p_max"]<=1e-9 and last["v_max"]<=1e-9 and last["b_max"]<=1e-9
assert last["r_max"]<=1e-10
# The observer enforces scale-dependent matrix bounds at every comparison.
# Preserve actual maxima rather than replace its gates with aggregate claims.
inputs=json.loads((P/"input_manifest.json").read_text())
receipts=list(csv.DictReader((D/"receipts.csv").open()))
for kind,topic in (("lidar","/boxi/hesai/points"),("imu","/boxi/cpt7/imu")):
    assert [(int(r["seq"]),int(r["stamp_ns"])) for r in receipts if r["kind"]==kind]==[
        tuple(r[:2]) for r in inputs["topics"][topic]["messages"]],kind
native=[r for r in receipts if r["kind"]=="odom"]
times=np.array([int(r["stamp_ns"]) for r in native],dtype=np.int64)
poses=np.array([[float(r[k]) for k in ("px","py","pz","qx","qy","qz","qw")] for r in native])
assert len(native)>=70 and np.isfinite(poses).all() and np.all(np.diff(times)>0)
assert (times[-1]-times[0])/1e9>6.8
assert (inputs["window_end_ns"]-times[-1])/1e9<.3
assert np.max(np.abs(np.linalg.norm(poses[:,3:],axis=1)-1))<1e-6
assert [int(r["seq"]) for r in native]==list(range(len(native)))
provider=None
pending=None
first=revisions=0
seen=set()
beliefs={}
first_error=0.
minimum_eigenvalue=float("inf")
max_symmetry=0.
for raw in (D/"node.log").open():
    line=re.sub(r"\x1b\[[0-9;]*m","",raw).strip()
    for bad in ("SHARED_NATIVE_PROVIDER_FAILURE","E1B_PROFILE_SINK_ERROR",
                "exception was caught","falling back!!","terminate called",
                "Segmentation fault","IndeterminantLinearSystemException"):
        assert bad not in line,line
    if "SHARED_NATIVE_PROVIDER_QUERY " in line:
        f=dict(x.split("=",1) for x in line.split("SHARED_NATIVE_PROVIDER_QUERY ",1)[1].split())
        assert int(f["resets"])==0
        assert int(f["admitted"])==sum(int(f[k]) for k in ("active","absorbed","removed"))
        provider=(int(f["from"])&((1<<56)-1),int(f["to"])&((1<<56)-1),
            np.array([[float(v) for v in row.split(":")] for row in f["matrix"].split(";")]).reshape(-1))
    if "GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW," in line:
        f=line.split("GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW,",1)[1].split(",")
        assert len(f)==5 and f[-1]=="native_local"
        pending=tuple(map(int,f[:4]));assert pending not in seen;seen.add(pending)
    b=parse_belief("glim",line)
    if b is None:continue
    key=(b["i"],b["j"],b["source_revision"])
    assert key not in beliefs;beliefs[key]=b
    assert b["j"]-b["i"]==2,"unexpected horizon pair outside paired queries"
    cov=np.asarray(b["covariance"]).reshape(6,6)
    assert np.isfinite(cov).all()
    minimum_eigenvalue=min(minimum_eigenvalue,float(np.linalg.eigvalsh((cov+cov.T)/2)[0]))
    max_symmetry=max(max_symmetry,float(np.max(np.abs(cov-cov.T))))
    if b["source_revision"]:
        assert pending==(*key,int(b["source_update"]));pending=None;revisions+=1
    else:
        assert provider is not None and provider[:2]==key[:2]
        error=float(np.max(np.abs(provider[2]-cov.reshape(-1))))
        assert error==0
        first_error=max(first_error,error);first+=1
assert pending is None and revisions>0 and revisions==len(seen)
assert first>=len(native)-3 and minimum_eigenvalue>0 and max_symmetry<1e-10
params=json.loads((D/"effective_params.json").read_text())
baseline=json.loads((R/"k16d_source_1/reference/effective_params.json").read_text())
changed=[k for k in set(params)|set(baseline) if params.get(k)!=baseline.get(k)]
assert set(changed)=={"config_path","glim_dump_path","glim_factor_graph_inspector_recording_id"}
maps=(D/"maps.txt").read_text()
for part in ("/main_gpu_batching_cpk16_20261020/release_1/libgtsam_points.so.1.2.1",
             "/local_revision_covariance_cpi1_20261019/libglim_ros.so",
             "/e1b_local_profile_20261012/release_2/libgtsam_points_cuda.so.1.2.1",
             "/k16f_source_2/build_1/probe.so"):
    assert part in maps,part
assert json.loads((P/"integrity_before.json").read_text())["passed"]
result=dict(passed=True,capture=capture,cleanup=cleanup,paired_summary=last,
    update_physical_span_s=last["stamp"]-rows[0]["stamp"],
    first_retirement_update=next(r for r in rows if r["retirement_updates"]>0),
    sensor_counts={t:len(s["messages"]) for t,s in inputs["topics"].items()},
    native_count=len(native),native_span_s=float((times[-1]-times[0])/1e9),
    native_first_stamp_ns=int(times[0]),native_last_stamp_ns=int(times[-1]),
    startup_gap_s=float((times[0]-inputs["window_start_ns"])/1e9),
    tail_gap_s=float((inputs["window_end_ns"]-times[-1])/1e9),
    first_beliefs=first,revised_beliefs=revisions,
    first_provider_wire_covariance_max=first_error,
    minimum_exported_covariance_eigenvalue=minimum_eigenvalue,
    maximum_exported_covariance_symmetry_error=max_symmetry,
    effective_parameter_changes=changed,
    candidate_route="unchanged K16a DSO: disabled source vs enabled independently owned companion",
    scope="one actual prepared-input OFF source history through retirement; not timing, ATE or whole-system certification")
with (H/"RESULT_K16F.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
