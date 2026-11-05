"""Preserve the failed observer attempt and corrected plumbing evidence."""
from pathlib import Path
import hashlib
import json
import subprocess
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
A=R/"k16f_source_1"
B=R/"k16f_source_2"
def sha(p):
    with Path(p).open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()
result={"status":"OBSERVER_CORRECTED_REAL_HISTORY_GATE_NOT_COMPLETED"}
for name in ("protected","input_hashes"):
    h=json.loads((R/(name+".json")).read_text())
    changed=[p for p,s in h.items() if sha(p)!=s]
    assert not changed,changed
    result[name]={"count":len(h),"changed":changed}
result["source_repositories"]={}
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
        status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
        diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    assert old==new,name
    result["source_repositories"][name]={"unchanged":True}
result["failed_attempt"]={
    "path":str(A/"paired"),
    "reason":(A/"paired/probe/failure.txt").read_text(),
    "cleanup":json.loads((A/"paired/cleanup.json").read_text())}
assert result["failed_attempt"]["cleanup"]["all_exited"]
assert not (A/"paired/capture_complete.json").exists()
result["cpu_equality_proof"]=json.loads((A/"cpu_copy_check/result.json").read_text())
result["corrected_plumbing"]=json.loads((B/"smoke/result.json").read_text())
assert result["corrected_plumbing"]["passed"]
assert not (B/"paired").exists(),"fresh source attempt already exists; do not duplicate"
result["observer_sha256"]=sha(B/"build_1/probe.so")
result["candidate_sha256"]=sha(R/"release_1/libgtsam_points.so.1.2.1")
result["real_history_passed"]=False
result["production_changed"]=False
with (H/"RESULT_K16F_PENDING.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
