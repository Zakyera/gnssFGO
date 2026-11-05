"""New standalone retained-factor test; never edits/launches an estimator."""
import hashlib,json,shutil,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
old=W/'runs/kimera_failed_factor_cpk1_20261019'
R=W/'runs/kimera_consistent_landmark_cpk2_20261019'
R.mkdir(exist_ok=False)
manifest=json.loads((old/'protected_before.json').read_text())
manifest.update(json.loads((old/'pre_capture_receipt.json').read_text())['diagnostic_manifest'])
for base in (old, H.parent/'kimera_failed_factor_cpk1_20261019'):
    for p in base.rglob('*'):
        if p.is_file():
            with p.open('rb') as f:manifest[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
(R/'protected_before.json').write_text(json.dumps(manifest,indent=2))
for n in ('ConditionalStereo.h','test.cpp','build.py'):
    shutil.copy2(H/n,R/n)
shutil.copy2(old/'analysis/smart_snapshot.cpp',R/'smart_snapshot.cpp')
shutil.copy2(old/'actual_test_commands.json',R/'original_commands.json')
print(R)
