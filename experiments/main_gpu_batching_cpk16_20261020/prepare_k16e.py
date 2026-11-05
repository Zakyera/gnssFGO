"""New bounded diagnostic artifacts; immutable previous inputs are copied."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
H=Path(__file__).resolve().parent
W=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R=W/"runs/main_gpu_batching_cpk16_20261020"
P=R/"k16e_source_2"
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
# Check all protected estimator/profile inputs before making diagnostics.
changed={}
for n in ("protected","input_hashes"):
    changed[n]=[p for p,h in json.loads((R/(n+".json")).read_text()).items() if sha(p)!=h]
assert not any(changed.values()), changed
P.mkdir(exist_ok=False)
for n in ("PROTOCOL_K16E.txt","probe_k16e.cpp","build_k16e.py"):
    shutil.copy2(H/n,P/n)
for n in ("ExactGpuSnapshot.h","ExactGpuSnapshot.cpp"):
    shutil.copy2(R/"source"/n,P/n)
p=P/"ExactGpuSnapshot.h"
s=p.read_text().replace(" public:\n"," public:\n"
    "  static void discardDiagnosticPending(IntegratedVGICPFactorGPU&);\n"
    "  static void dumpDiagnostic(const IntegratedVGICPFactorGPU&,const std::string&);\n",1)
p.write_text(s)
shutil.copytree(R/"k16d_source_1/glim_config",P/"glim_config")
shutil.copy2(R/"k16d_source_1/frozen_glim_params.yaml",P/"frozen_glim_params.yaml")
shutil.copy2(H/"observe_k16d.py",P/"observe_k16d.py")
s=(H/"input_k16d.py").read_text().replace("60_000_000_000","2_000_000_000")
(P/"input_k16d.py").write_text(s)
s=(H/"run_source_k16d.py").read_text()
s=s.replace('if mode == "batched":\n    assert json.loads((P / "reference/validation.json").read_text())["passed"]',
    'if mode == "batched":\n    assert json.loads((P / "reference/capture_complete.json").read_text())["passed"]')
s=s.replace("port = 11463","port = 11464")
s=s.replace('node_env["CPK16_MAIN_GPU_BATCHING"] = "1"',
    'node_env["CPK16_MAIN_GPU_BATCHING"] = "1"')
s=s.replace('expected_dso = (','node_env["LD_PRELOAD"] = str(P/"build/probe.so")\n'
    'node_env["CPK16E_PROBE_ROOT"] = str(out/"probe")\n'
    'expected_dso = (')
s=s.replace('out.mkdir()\n','out.mkdir()\n(out/"probe").mkdir()\n')
s=s.replace('deadline = time.monotonic() + 400','deadline = time.monotonic() + 40')
s=s.replace('requested_window_s=60','requested_window_s=2')
s=s.replace('assert "/CPK16" not in maps','assert str(P/"build/probe.so") in maps')
s=s.replace('libs[0] / "libgtsam_points.so.1.2.1") in maps',
    'libs[0] / "libgtsam_points.so.1.2.1") in maps')
(P/"run_source_k16e.py").write_text(s)
files={str(p.relative_to(P)):sha(p) for p in P.rglob("*") if p.is_file()}
(P/"prepared.json").write_text(json.dumps(dict(files=files,protected_unchanged=True),indent=2))
print(P)
