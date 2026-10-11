#!/usr/bin/env python3
"""Archive bounded diagnostic inputs/evidence; large datasets stay at run root."""
import hashlib
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/glim_e1_real_graph_20261011/prepared')
DEST = HERE / 'evidence'
assert json.loads((ROOT / 'validation.json').read_text())['passed']
DEST.mkdir(exist_ok=False)
names = [
    'prepared_manifest.json', 'preflight.json', 'validation.json',
    'build_commands.json', 'build_0.log', 'build_1.log',
    'gpu_preflight.log', 'ldd_preflight.log', 'port_preflight.log',
    'regression.log', 'regression.xml', 'control/complete.json', 'control/source.log',
    'bag_hesai_metadata.log', 'bag_cpt7_imu_metadata.log', 'bag_alphasense_metadata.log',
    'source.env', 'source_runner.sh', 'coverage_checks.py', 'verify_effective.py',
    'active_window.launch', 'grandtour_eth1_kimera_stereo.launch',
    'kimera_vio_ros.launch', 'kimera_vio_ros_newer_college_2021_stereo.launch',
    'private_build/cbs_bridge.cpp', 'private_build/RealGraphAudit.h',
    'e1_source/run_metadata.txt', 'e1_source/source_coverage.json',
    'e1_source/effective_mode_check.json', 'e1_source/resolved_glim_params.yaml',
    'e1_source/resolved_kimera_params.yaml',
]
for n in [10, 30, 60, 100, 150, 180]:
    for f in ['summary.json', 'structure.txt', 'means_unchanged.marker']:
        names.append('audit/sample_%d/%s' % (n, f))
for tree in ['glim_config', 'kimera_profile']:
    names.extend(str(p.relative_to(ROOT)) for p in (ROOT/'e1_source'/tree).rglob('*') if p.is_file())
manifest = {}
for name in names:
    src, dst = ROOT/name, DEST/name
    assert src.is_file() and src.stat().st_size < 2_000_000, name
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    digest = hashlib.sha256(src.read_bytes()).hexdigest()
    assert hashlib.sha256(dst.read_bytes()).hexdigest() == digest
    manifest[name] = {'original': str(src), 'sha256': digest, 'bytes': src.stat().st_size}
with (HERE/'ARCHIVE_MANIFEST.json').open('x') as f:
    json.dump(manifest, f, indent=2)
print('Archived', len(manifest), 'byte-identical files;', sum(v['bytes'] for v in manifest.values()), 'bytes')
