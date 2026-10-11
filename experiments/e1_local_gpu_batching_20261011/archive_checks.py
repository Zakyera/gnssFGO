#!/usr/bin/env python3
"""Record E1 provenance and archive small validation logs, not binaries/data."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN = WS / 'runs/glim_local_gpu_batching_e1_20261011/private'
MANIFEST = WS / 'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/protected_manifest.json'
EXPECTED_SOURCE_EDITS = {
    str(WS / 'src/cbs/include/cbs/bpsam/bpsam.h'),
    str(WS / 'src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'),
    str(WS / 'src/cbs/src/bpsam/bpsam.cpp'),
    str(WS / 'src/glim_ros1/include/glim_ros/cbs_bridge.hpp'),
    str(WS / 'src/glim_ros1/src/glim_ros/cbs_bridge.cpp'),
    str(WS / 'src/glim_ros1/CMakeLists.txt'),
}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


protected = json.loads(MANIFEST.read_text())
matched, changes, missing = [], {}, []
for name, expected in protected.items():
    if not Path(name).is_file():
        missing.append(name)
        continue
    actual = sha(name)
    if actual == expected:
        matched.append(name)
    else:
        changes[name] = {'baseline': expected, 'candidate': actual}
assert not missing, missing
assert set(changes) <= EXPECTED_SOURCE_EDITS, changes
result = {'protected_manifest': str(MANIFEST), 'protected_total': len(protected),
          'unchanged_count': len(matched), 'intentional_source_changes': changes,
          'unexpected_changes': [], 'missing': [], 'binaries': {}, 'repositories': {}}
for p in [RUN / 'libcbs.so', RUN / 'libglim_ros.so', RUN / 'test_attempt3',
          RUN / 'sanitize_attempt2',
          Path('/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build/replay'),
          WS / 'runs/glim_cbs_diagnostic_budget_20261010/prepared/private_build/libglim_ros.so']:
    result['binaries'][str(p)] = sha(p)
for repo in [WS / 'src/cbs', WS / 'src/glim_ros1', WS / 'src/gtsam_points']:
    result['repositories'][repo.name] = {
        'head': subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        'status': subprocess.check_output(['git','-C',str(repo),'status','--porcelain'],text=True),
    }
out = HERE / 'validation_logs'
out.mkdir(exist_ok=False)
for p in sorted(RUN.iterdir()):
    if p.suffix in ['.log', '.xml', '.json']:
        shutil.copy2(p, out / p.name)
result['archived_logs'] = {p.name: sha(p) for p in sorted(out.iterdir())}
with (HERE / 'validation_manifest.json').open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps({k: result[k] for k in ['protected_total','unchanged_count','intentional_source_changes']},indent=2))
