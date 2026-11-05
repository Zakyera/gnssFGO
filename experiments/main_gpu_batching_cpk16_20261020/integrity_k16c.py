"""Same protected-input checks as K16b, distinct immutable K16c receipts."""
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
changed={}
for name in ["protected","input_hashes"]:
    hashes=json.loads((R/(name+".json")).read_text())
    changed[name]=[p for p,h in hashes.items() if sha(p)!=h]
source_changed=[]
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
      status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
      diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    if new!=old:source_changed.append(name)
candidate=sha(R/"release_1/libgtsam_points.so.1.2.1")
result=dict(passed=not any(changed.values()) and not source_changed and
  candidate=="6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca",
  changed=changed,source_changed=source_changed,candidate_sha256=candidate,
  protected_count=len(json.loads((R/"protected.json").read_text())))
name=sys.argv[1];assert name in ("INTEGRITY_K16C_BEFORE.json","INTEGRITY_K16C_FINAL.json")
with (H/name).open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
assert result["passed"]
