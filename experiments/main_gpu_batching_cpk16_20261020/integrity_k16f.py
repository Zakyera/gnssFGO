"""Nonmutating estimator checks with distinct write-once audit receipts."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
P=R/"k16f_source_2"
phase=sys.argv[1]
assert phase in ("before","after")
def sha(p):
    with Path(p).open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()
result={"phase":phase,"files":{},"repositories":{}}
for name in ("protected","input_hashes"):
    hashes=json.loads((R/(name+".json")).read_text())
    changed=[p for p,s in hashes.items() if sha(p)!=s]
    result["files"][name]={"count":len(hashes),"changed":changed}
    assert not changed,changed
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
        status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
        diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    assert new==old,name
    result["repositories"][name]={"unchanged":True}
manifest=json.loads((P/"prepared.json").read_text())
for path,digest in manifest["files"].items():assert sha(P/path)==digest,path
assert sha(P/"build_1/probe.so")=="f641eb76d7b653d02d7222a1a0f179bcb3c91459bfe325dcc88ea1300e072267"
assert sha(R/"release_1/libgtsam_points.so.1.2.1")=="6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca"
assert json.loads((P/"smoke/result.json").read_text())["passed"]
if phase=="before":assert not (P/"paired").exists(),"refuse duplicate source launch"
result.update(prepared_files=len(manifest["files"]),observer_sha256=sha(P/"build_1/probe.so"),passed=True)
with (P/("integrity_"+phase+".json")).open("x") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
