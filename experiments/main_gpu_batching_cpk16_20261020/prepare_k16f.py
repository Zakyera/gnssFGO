"""Prepare one bounded real-input diagnostic, never change earlier artifacts."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
assert len(sys.argv)==2 and sys.argv[1] in ("1","2")
P=R/("k16f_source_"+sys.argv[1])
def sha(p):
    with Path(p).open("rb") as f: return hashlib.file_digest(f,"sha256").hexdigest()
receipt={}
for name in ("protected","input_hashes"):
    hashes=json.loads((R/(name+".json")).read_text())
    changed=[p for p,h in hashes.items() if sha(p)!=h]
    assert not changed,changed
    receipt[name]=dict(count=len(hashes),changed=changed)
for name,old in json.loads((R/"source_git.json").read_text()).items():
    repo=W/"src"/name
    new=dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
        status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
        diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    assert new==old,name
P.mkdir(exist_ok=False)
for n in ("PROTOCOL_K16F.txt","probe_k16f.cpp","build_k16f.py","smoke_k16f.cpp","run_smoke_k16f.py",
          "test_main_lifecycle.cpp","observe_k16d.py"):
    shutil.copy2(H/n,P/n)
for n in ("ExactGpuSnapshot.h","ExactGpuSnapshot.cpp"):
    shutil.copy2(R/"source"/n,P/n)
# Mechanical declaration extension only in private diagnostic copy.
p=P/"ExactGpuSnapshot.h"
p.write_text(p.read_text().replace(" public:\n"," public:\n"
    "  static std::vector<unsigned char> runtimeState(const IntegratedVGICPFactorGPU&);\n",1))
shutil.copytree(R/"k16d_source_1/glim_config",P/"glim_config")
shutil.copy2(R/"k16d_source_1/frozen_glim_params.yaml",P/"frozen_glim_params.yaml")
(P/"input_k16f.py").write_text((H/"input_k16d.py").read_text().replace(
    "60_000_000_000","8_000_000_000"))
s=(H/"run_source_k16d.py").read_text()
s=s.replace('mode = sys.argv[1]\nassert mode in ("reference", "batched")\n'
    'if mode == "batched":\n    assert json.loads((P / "reference/validation.json").read_text())["passed"]',
    'mode = "paired"\nassert len(sys.argv)==1\n'
    'assert json.loads((P/"smoke/result.json").read_text())["passed"]')
s=s.replace("port = 11463","port = 11465")
s=s.replace('if mode == "batched":\n    libs.insert(0, R / "release_1")',
    'libs.insert(0, R / "release_1")')
s=s.replace('if mode == "batched":\n    node_env["CPK16_MAIN_GPU_BATCHING"] = "1"',
    'node_env["LD_PRELOAD"] = str(P/"build_1/probe.so")\n'
    'node_env["CPK16F_PROBE_ROOT"] = str(out/"probe")')
s=s.replace('if mode == "batched" else "3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7"',
    'if mode == "paired" else "3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7"')
s=s.replace('out.mkdir()\n','out.mkdir()\n(out/"probe").mkdir()\n')
s=s.replace('deadline = time.monotonic() + 400','deadline = time.monotonic() + 90')
s=s.replace('requested_window_s=60','requested_window_s=8')
s=s.replace('assert "/CPK16" not in maps','assert str(P/"build_1/probe.so") in maps')
s=s.replace('"cpk16d_" + mode','"cpk16f_" + mode')
(P/"run_source_k16f.py").write_text(s)
files={str(p.relative_to(P)):sha(p) for p in P.rglob("*") if p.is_file()}
(P/"prepared.json").write_text(json.dumps(dict(files=files,integrity=receipt),indent=2))
print(P)
