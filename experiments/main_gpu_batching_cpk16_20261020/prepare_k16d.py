"""Host preparation: immutable source-only harness, no estimator modifications."""
import hashlib
import json
import shutil
from pathlib import Path
H = Path(__file__).resolve().parent
W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
K = W / "runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared"
P = W / "runs/main_gpu_batching_cpk16_20261020/k16d_source_1"
assert json.loads((H / "INTEGRITY_K16D_BEFORE.json").read_text())["passed"]
P.mkdir(exist_ok=False)
for name in ("run_source_k16d.py", "observe_k16d.py", "input_k16d.py", "PROTOCOL_K16D.txt"):
    shutil.copy2(H / name, P / name)
shutil.copytree(K / "input_profiles/glim_config", P / "glim_config")
shutil.copy2(K / "full_off_source/resolved_glim_params.yaml", P / "frozen_glim_params.yaml")
def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()
manifest = {str(p.relative_to(P)): sha(p) for p in sorted(P.rglob("*")) if p.is_file()}
with (P / "prepared.json").open("x") as f:
    json.dump(dict(files=manifest, original_container_state="stopped",
                   original_params=str(K / "full_off_source/resolved_glim_params.yaml")), f, indent=2)
print(P)
