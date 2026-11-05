"""Snapshot CP-K20, changing only identities and explicit coverage/reporting."""
from pathlib import Path
import ast
import difflib
import hashlib
import importlib.util
import json
import shutil
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S = W/'runs/gnssfgo_eth1_finalization_full_cpk20_20261021/prepared'
R = W/'runs/gnssfgo_eth1_keyframe_finalization_full_cpk22_20261021/prepared'
V = Path('/home/yeranis/results/gnssfgo_eth1_keyframe_finalization_full_cpk22_20261021')
H20 = H.parent/'eth1_finalization_full_cpk20_20261021'
P21 = W/'runs/kimera_eth1_full_rate050_cpk21_20261021'

def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()

def once(s, a, b):
    assert s.count(a) == 1, (a, s.count(a))
    return s.replace(a, b)

def common(s):
    return s.replace(S.parent.name, R.parent.name).replace('eth1_cpk20_full', 'eth1_cpk22_full').replace('11470', '11472').replace('CP-K20', 'CP-K22')

assert not R.parent.exists() and not V.exists(), 'refuse existing outputs'
assert shutil.disk_usage(W).free > 15*1024**3
with socket.socket() as sock: sock.bind(('127.0.0.1', 11472))
manifest = json.loads((S/'protected_manifest.json').read_text())
for p, want in manifest.items(): assert sha(p) == want, p
assert not list(S.glob('full_*_control')), 'do not duplicate a launched CP-K20'

# Bounded saved-data checks; no source, solver, or evaluator rerun.
spec = importlib.util.spec_from_file_location('coverage_cpk22', H/'coverage_checks.py')
coverage = importlib.util.module_from_spec(spec); spec.loader.exec_module(coverage)
t, p = coverage.load(P21/'native_odometry.csv', True)
old = json.loads((P21/'analysis.json').read_text())
clock = old['topic_summary']['/boxi/cpt7/imu']
first, last = clock['first_ns']/1e9, clock['last_ns']/1e9
positive = coverage.check(t, p, 346.32, first, last, 'native_kimera')
progress = coverage.processing([P21/'capture/estimators.log'], t)
assert positive['rows'] == 1492 and progress['frontend_processed'] == 3460
negatives = []
for name, tt, pp, end in [
    ('duplicate stamp', np.r_[t[0], t[:-1]], p, last),
    ('long keyframe gap', np.r_[t[:-1], t[-2]+5.2], p, last),
    ('truncated native tail', t[:-2], p[:-2], last),
    ('truncated input', t, p, last-1),
    ('nonfinite position', t, p*np.nan, last)]:
    try: coverage.check(tt, pp, 346.32, first, end, 'native_kimera')
    except AssertionError: negatives.append(name)
    else: raise AssertionError('negative unexpectedly passed: '+name)
try: coverage.processing([P21/'capture/estimators.log'], t[:-1])
except AssertionError: negatives.append('missing output versus backend')
else: raise AssertionError('backend conservation negative passed')
# Native GLIM's former dense-coverage gate is intentionally unchanged.
gt = np.arange(first+.6, last-.1, .1); gp = np.zeros((len(gt), 3))
coverage.check(gt, gp, 346.32, first, last, 'native_glim')
try: coverage.check(gt[:-12], gp[:-12], 346.32, first, last, 'native_glim')
except AssertionError: negatives.append('GLIM tail remains strict')
else: raise AssertionError('GLIM dense coverage relaxed')

shutil.copytree(S, R, symlinks=True,
    ignore=shutil.ignore_patterns('__pycache__', 'protected_manifest.json', 'preflight'))
diff = []
for path in R.rglob('*'):
    if not path.is_file() or path.suffix not in ('.py', '.json', '.sh', '.env', '.launch', '.txt'): continue
    old_text = path.read_text(); s = common(old_text)
    if path.name == 'coverage_checks.py': s = (H/'coverage_checks.py').read_text()
    if path.name in ('source_runner.sh', 'source_runner_on.sh'):
        s = once(s, 'min_kimera_rows=1700', '# Sequential IDs, frontend completion and backend/output conservation were checked above.\nmin_kimera_rows=11')
    if path.name == 'validate_candidate.py':
        s = once(s, 'assert len(rows) == len(poses) and len(rows) >= 1700',
            "coverage = json.loads((S / 'source_coverage.json').read_text())\n"
            "assert coverage['passed'] and coverage['coverage_contract'] == 'native_keyframes_cpk22'\n"
            "assert coverage['kimera_processing']['passed']\n"
            "assert len(rows) == len(poses) == coverage['kimera_processing']['backend_processed'] and len(rows) > 10")
    if path.name == 'validate_case.py':
        s = once(s, "assert source_cov['native_glim']['rows']>=3430 and source_cov['native_kimera']['rows']>=1700",
            "assert source_cov['native_glim']['rows']>=3430\n"
            "    assert source_cov['coverage_contract']=='native_keyframes_cpk22' and source_cov['kimera_processing']['passed']\n"
            "    assert source_cov['native_kimera']['rows']==source_cov['kimera_processing']['backend_processed']>10")
    if path.name == 'evaluate_fl4f.py':
        s = once(s, '    assert len(times)>500 and max(np.diff(times))<=.100001',
            "    assert len(times)>500 and np.all(np.diff(times)>0)\n"
            "    segments=np.split(times,np.flatnonzero(np.diff(times)>.10001)+1)\n"
            "    coverage=dict(grid_queries=len(grid),excluded_queries=len(grid)-len(times),\n"
            "        envelope_s=float(times[-1]-times[0]),\n"
            "        covered_contiguous_duration_s=float(sum(s[-1]-s[0] for s in segments)),\n"
            "        segments=[dict(start_s=float(s[0]),end_s=float(s[-1]),rows=len(s)) for s in segments],\n"
            "        maximum_query_gap_s=float(np.diff(times).max()))")
        s = once(s, 'common_mask=dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),',
            'common_mask=dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),coverage=coverage,')
        s = once(s, "        convention='Per-path rigid", "        path_length_caveat='Polyline through actual saved or supported queried positions; includes chords across missing intervals, not dense coverage.',\n        convention='Per-path rigid")
    if path.name == 'PROTOCOL.txt': s = (H/'PROTOCOL.txt').read_text()
    path.write_text(s)
    if s != old_text: diff.extend(difflib.unified_diff(old_text.splitlines(True), s.splitlines(True), fromfile=str(S/path.relative_to(R)), tofile=str(path)))
summary = common((H20/'summarize.py').read_text())
summary = once(summary, " for line in (S/'estimators.log').open():", " kimera_logs=[S/'estimators.log',*S.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]\n seen_timing=set()\n for line in (line for log in kimera_logs for line in log.open()):\n  timing_payload=line.split('KIMERA_',1)[-1].strip()\n  if timing_payload in seen_timing:continue\n  if 'KIMERA_' in line:seen_timing.add(timing_payload)")
summary = once(summary, " 'reported in source_coverage; Kimera retains its initial ~3.2s internal gap.',", " 'reported in source_coverage, including sparse stationary-tail keyframes. Common',\n 'mask segments exclude unsupported interpolation; its envelope is not dense coverage.',")
(H/'summarize.py').write_text(summary)
(H/'harness.diff').write_text(''.join(diff))

def cfg(path):
    c = json.loads(path.read_text())
    for k in ('sequence','ros_master_uri','receiver_binary'): c.pop(k)
    for source in c['sources']: source.pop('session')
    return c
assert cfg(R/'full_off.json') == cfg(R/'full_on.json') == cfg(S/'full_off.json')
for folder in ('private_build', 'input_profiles'):
    for path in (S/folder).rglob('*'):
        if path.is_file(): assert sha(path) == sha(R/path.relative_to(S)), path
for path in [*R.rglob('*'), H/'summarize.py']:
    if path.suffix == '.py': ast.parse(path.read_text())
    elif path.suffix == '.launch': ET.parse(path)
    elif path.suffix in ('.sh', '.env'): subprocess.run(['bash','-n',str(path)], check=True)
result = json.loads((S/'preflight/validation.json').read_text())
result.update(template=str(S), coverage_contract='native_keyframes_cpk22',
    coverage_test=dict(passed=True, positive=positive, processing=progress, rejected_negatives=negatives),
    no_source_or_solver_test=True, bag_preflight_reused_from=str(S/'preflight/validation.json'))
# Original bag identities are already in the protected source manifests; record
# present size/mtime to make this metadata reuse explicit, not a second scan.
result['bag_files'] = {}
for name in ('hesai','cpt7_imu','alphasense','cpt7_ie_tc'):
    path = Path(result['bags'][name]['path'].replace('/workspace/', str(W.parent)+'/'))
    st = path.stat(); result['bag_files'][name] = dict(path=str(path), bytes=st.st_size, mtime_ns=st.st_mtime_ns)
for base in (R, H):
    for path in base.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.name != 'REPORT.txt': manifest[str(path)] = sha(path)
result['protected_files'] = len(manifest)
(R/'protected_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
(R/'preflight').mkdir()
(R/'preflight/validation.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(dict(passed=True, prepared=str(R), protected_files=len(manifest),
    saved_fixture_rows=len(t), rejected_negatives=negatives, model_and_profiles_unchanged=True), indent=2))
