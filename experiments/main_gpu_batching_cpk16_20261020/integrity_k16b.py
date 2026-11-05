"""Verify untouched production/protected inputs; create a new receipt."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
def sha(p):
    with Path(p).open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()
protected=json.loads((R/"protected.json").read_text())
inputs=json.loads((R/"input_hashes.json").read_text())
changed=[p for p,h in protected.items() if sha(p)!=h]
input_changed=[p for p,h in inputs.items() if sha(p)!=h]
git_changed=[]
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
      status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
      diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    if new!=old:git_changed.append(name)
candidate=sha(R/"release_1/libgtsam_points.so.1.2.1")
result=dict(passed=not(changed or input_changed or git_changed) and
      candidate=="6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca",
  protected_files=len(protected),protected_changed=changed,input_changed=input_changed,
  source_repositories_changed=git_changed,candidate_sha256=candidate)
name=sys.argv[1] if len(sys.argv)>1 else "INTEGRITY_K16B.json"
assert name in ("INTEGRITY_K16B.json","INTEGRITY_K16B_FINAL.json")
with (H/name).open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
if not result["passed"]:raise SystemExit(1)
