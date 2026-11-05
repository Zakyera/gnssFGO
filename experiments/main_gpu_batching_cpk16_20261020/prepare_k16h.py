"""Copy immutable prerequisites into a new diagnostic root; no estimator edits."""
from pathlib import Path
import hashlib,json,shutil
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/main_gpu_batching_cpk16_20261020'
G=R/'k16g_source_1'; P=R/'k16h_source_1'
assert not P.exists(), 'refuse duplicate preparation'
assert hashlib.file_digest((G/'build_1/probe.so').open('rb'),'sha256').hexdigest()=='ffededfc5dd8322c83310d724309992cc53982c48ef5266d809213f8accd89b0'
P.mkdir()
for name in ['glim_config','frozen_glim_params.yaml','input_manifest.json','observe_k16d.py']:
    src=G/name
    if src.is_dir():shutil.copytree(src,P/name)
    else:shutil.copy2(src,P/name)
for name in ['paced_k16h.cpp','PROTOCOL_K16H.txt','build_k16h.py','run_k16h.py','validate_k16h.py']:
    shutil.copy2(H/name,P/name)
print(P)
