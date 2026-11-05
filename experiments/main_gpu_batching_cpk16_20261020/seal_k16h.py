from pathlib import Path
import hashlib,json
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');R=W/'runs/main_gpu_batching_cpk16_20261020';P=R/'k16h_source_1'
assert not (P/'prepared.json').exists() and not (P/'paired').exists()
assert json.loads((P/'coverage_3/coverage_complete.json').read_text())['passed']
assert json.loads((P/'driver_build_4/complete.json').read_text())['passed']
files={str(p.relative_to(P)):hashlib.sha256(p.read_bytes()).hexdigest()
       for p in P.rglob('*') if p.is_file() and '__pycache__' not in str(p)}
with (P/'prepared.json').open('x') as f:json.dump(dict(files=files,observer='unchanged k16g_source_1/build_1/probe.so'),f,indent=2)
print('sealed',len(files),'files')
