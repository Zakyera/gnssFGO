"""Read-only correctness/integrity synthesis; writes new report evidence only."""
from pathlib import Path
import hashlib
import json
import re
import subprocess

H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
def sha(p):
    with Path(p).open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()
checks=R/"release_1/checks_1"
receipts=[checks/"passed.json",R/"ubsan_1/passed.json",R/"asan_1/passed.json",R/"abi_load.json"]
assert all(json.loads(p.read_text())["passed"] for p in receipts)
manifest=json.loads((R/"protected.json").read_text())
changed=[p for p,h in manifest.items() if sha(p)!=h]
inputs=json.loads((R/"input_hashes.json").read_text())
input_changes=[p for p,h in inputs.items() if sha(p)!=h]
git_changes=[]
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
             status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
             diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    if new!=old:git_changes.append(name)
metrics={}
for p in [checks/"main.log",checks/"repeat.log",R/"ubsan_1/main.log"]:
    for kind,value in re.findall(r"CPK16_ERROR (\w+) max_abs=([0-9eE+.-]+)",p.read_text()):
        x=metrics.setdefault(kind,dict(comparisons=0,max_abs=0.))
        x["comparisons"]+=1;x["max_abs"]=max(x["max_abs"],float(value))
result=dict(passed=not changed and not input_changes and not git_changes,
            protected_files=len(manifest),changed=changed,input_changes=input_changes,
            source_repository_changes=git_changes,factor_metrics=metrics,
            library_sha256=sha(R/"release_1/libgtsam_points.so.1.2.1"),
            receipts={str(p):json.loads(p.read_text()) for p in receipts},
            scope="K16a factor/cache/dispatch only; K16b actual MAIN lifecycle NOT RUN; no timing/dataset")
with (H/"RESULT_K16A.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!="receipts"},indent=2))
if not result["passed"]:raise SystemExit(1)
