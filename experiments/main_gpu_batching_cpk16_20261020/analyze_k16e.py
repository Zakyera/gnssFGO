"""Read-only comparison; writes a new result, never changes acceptance gates."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
P=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/k16e_source_2")
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def mat(p): return np.loadtxt(p,skiprows=1,ndmin=2)
def rows(mode,kind):
    with (P/mode/"receipts.csv").open() as f:
        return [r for r in csv.DictReader(f) if r["kind"]==kind]
out={"scope":"K16e first-update diagnosis; not K16d preservation acceptance",
     "equations":{},"calls":[],"native":[]}
for mode in ("reference","batched"):
    assert json.loads((P/mode/"capture_complete.json").read_text())["passed"]
    assert json.loads((P/mode/"cleanup.json").read_text())["all_exited"]
    calls=sorted((P/mode/"probe").glob("call_*"))
    assert len(calls)==2, (mode,len(calls))
    out["equations"][mode]=[json.loads((c/"result.json").read_text()) for c in calls]
for i in range(2):
    a=P/"reference/probe"/f"call_{i}"
    b=P/"batched/probe"/f"call_{i}"
    comparison=dict(index=i,poses={},arrays={},actual_equation_max=float(
        np.max(np.abs(mat(a/"actual.txt")-mat(b/"actual.txt")))))
    for p in a.glob("pose_*.txt"):
        comparison["poses"][p.name]=float(np.max(np.abs(mat(p)-mat(b/p.name))))
    for p in (a/"actual_before").glob("*.bin"):
        q=b/"actual_before"/p.name
        row={"reference_bytes":p.stat().st_size,"candidate_bytes":q.stat().st_size if q.exists() else None,
             "reference_sha256":sha(p),"candidate_sha256":sha(q) if q.exists() else None}
        row["exact"]=q.exists() and row["reference_sha256"]==row["candidate_sha256"]
        if q.exists() and p.stat().st_size==q.stat().st_size and p.name in (
                "source_points.bin","source_covs.bin","source_normals.bin",
                "source_intensities.bin","voxel_means.bin","voxel_covs.bin"):
            x=np.fromfile(p,dtype="<f4");y=np.fromfile(q,dtype="<f4")
            row.update(changed_entries=int(np.count_nonzero(x!=y)),
                       entries=len(x),maxabs=float(np.max(np.abs(x-y))),
                       rms=float(np.sqrt(np.mean((x.astype(float)-y)**2))))
        comparison["arrays"][p.name]=row
    # Distinguish point ordering from changed geometry.
    x=np.fromfile(a/"actual_before/source_points.bin",dtype="<f4").reshape(-1,3)
    y=np.fromfile(b/"actual_before/source_points.bin",dtype="<f4").reshape(-1,3)
    xs=x[np.lexsort(x.T)];ys=y[np.lexsort(y.T)]
    comparison["source_point_set_exact"]=bool(np.array_equal(xs,ys))
    comparison["source_sorted_max"]=float(np.max(np.abs(xs-ys)))
    out["calls"].append(comparison)
for kind in ("imu","lidar"):
    a=rows("reference",kind);b=rows("batched",kind)
    assert [(r["seq"],r["stamp_ns"]) for r in a]==[(r["seq"],r["stamp_ns"]) for r in b]
    out[kind+"_count"]=len(a)
a=rows("reference","odom");b=rows("batched","odom")
assert [r["stamp_ns"] for r in a]==[r["stamp_ns"] for r in b]
for x,y in zip(a,b):
    pos=lambda r:np.array([float(r[k]) for k in ("px","py","pz")])
    rot=lambda r:Rotation.from_quat([float(r[k]) for k in ("qx","qy","qz","qw")])
    out["native"].append(dict(stamp_ns=int(x["stamp_ns"]),
        position_difference_m=float(np.linalg.norm(pos(x)-pos(y))),
        angle_difference_rad=float((rot(x).inv()*rot(y)).magnitude())))
out["native_count"]=len(a)
out["exact_input_equations_passed"]=all(r["passed"] for v in out["equations"].values() for r in v)
with (P/"first_update_analysis.json").open("x") as f: json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
