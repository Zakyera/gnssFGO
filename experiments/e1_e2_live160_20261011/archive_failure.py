"""Archive the completed capture's blocked validation, never relabel it as passed."""
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
RUN = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011')
ROOT = RUN / 'prepared'
DEST = HERE / 'evidence'
evidence = json.loads((RUN / 'capture_evidence.json').read_text())
assert evidence['capture_completed'] and not evidence['overall_validation_passed']
assert not (ROOT / 'live160_on_numerical_repeat').exists()
DEST.mkdir(exist_ok=False)
files = [p for p in RUN.iterdir() if p.is_file() and p.suffix in ['.py', '.json', '.jsonl', '.txt']]
files += [p for p in ROOT.iterdir() if p.is_file() and p.suffix in ['.py', '.json', '.sh', '.env', '.launch']]
files += list((ROOT / 'preflight').glob('*')) + list((RUN / 'validation_logs').glob('*'))
source = ROOT / 'live160_on_source'
files += [source / name for name in ['run_metadata.txt', 'source_coverage.json', 'effective_mode_check.json',
                                     'resolved_glim_params.yaml', 'resolved_kimera_params.yaml']]
for tree in ['glim_config', 'kimera_profile']:
    files.extend(p for p in (source / tree).rglob('*') if p.is_file())
files += [ROOT / 'live160_on_control' / name for name in ['complete.json', 'monitor.jsonl', 'source.log']]
live = ROOT / 'live160_on_receiver'
files += [live / name for name in ['manifest.json', 'resolved_config.json', 'transport_status.json']]
files += [live / 'receiver' / name for name in ['summary.json', 'source_accounting.csv', 'unsupported_beliefs.csv',
                                              'publication_convergence.csv', 'e2_memo.csv']]
manifest = {}
omitted = []
for path in sorted(set(files)):
    assert path.is_file(), path
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if path.stat().st_size > 2_000_000:
        omitted.append({'path': str(path), 'bytes': path.stat().st_size, 'sha256': digest})
        continue
    target = DEST / path.relative_to(RUN)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, target)
    assert digest == hashlib.sha256(target.read_bytes()).hexdigest()
    manifest[str(target.relative_to(HERE))] = {'original': str(path), 'bytes': path.stat().st_size, 'sha256': digest}
with (HERE / 'ARCHIVE_MANIFEST.json').open('x') as stream:
    json.dump({'status': 'CAPTURE_COMPLETE_VALIDATION_BLOCKED_HEADROOM_MISSED',
               'copied': manifest, 'omitted': omitted, 'run_root': str(RUN)}, stream, indent=2)
print('Archived', len(manifest), 'unchanged files;', sum(v['bytes'] for v in manifest.values()), 'bytes')
