"""Archive a stopped S5 capacity failure without running any estimator/validator."""
import json
from pathlib import Path
import shutil
import subprocess
from prepare import HERE, ROOT, RUN, BUILD, PRIOR, sha

control = ROOT / 'capacity160_on_control'
source = ROOT / 'capacity160_on_source'
live = ROOT / 'capacity160_on_receiver'
assert (control / 'failed.txt').read_text().strip() == 'live queue gate exceeded'
assert not (control / 'complete.json').exists()
assert not (source / 'complete.marker').exists()
assert not (live / 'complete.marker').exists()
assert json.loads((HERE / 'partial_summary.json').read_text())['status'] == 'CAPACITY_FAILED_PARTIAL_CAPTURE'

manifest = json.loads((ROOT / 'protected_manifest.json').read_text())
for p, h in manifest.items():
    assert sha(p) == h, p
repos = json.loads((PRIOR / 'before_integrity.json').read_text())['source_repositories']
for p, v in repos.items():
    assert subprocess.check_output(['git', '-C', p, 'rev-parse', 'HEAD'], text=True).strip() == v['head']
    assert subprocess.check_output(['git', '-C', p, 'status', '--porcelain'], text=True).strip() == v['status']

process_inventory = {}
for container, idle in [('cbsms_ws_gpu', 'tail'), ('ubuntu-jammy-humble-nvidia', 'zsh')]:
    output = subprocess.check_output(['docker', 'top', container, '-eo', 'pid,comm,args'], text=True)
    rows = output.strip().splitlines()[1:]
    assert len(rows) == 1 and rows[0].split()[1] == idle, output
    process_inventory[container] = output

evidence = HERE / 'evidence'
evidence.mkdir(exist_ok=False)
def copy(p, dest):
    dst = evidence / dest
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dst)

for p in ROOT.iterdir():
    if p.is_file() and p.suffix in ['.json', '.py', '.sh', '.env', '.launch']:
        copy(p, 'harness/' + p.name)
for sub in ['preflight', 'preflight_sandbox_denied', 'preflight_reference_path_hold', 'capacity160_on_control']:
    for p in (ROOT / sub).rglob('*'):
        if p.is_file() and p.stat().st_size < 5_000_000:
            copy(p, sub + '/' + str(p.relative_to(ROOT / sub)))
for name in ['effective_mode_check.json', 'resolved_glim_params.yaml', 'resolved_kimera_params.yaml', 'e1b_profile.jsonl']:
    copy(source / name, 'source/' + name)
for name in ['transport_status.json', 'transport_complete.json', 'resolved_config.json', 'manifest.json', 'STOP']:
    copy(live / name, 'receiver/' + name)
for name in ['summary.json', 'source_accounting.csv', 'unsupported_beliefs.csv', 'publication_convergence.csv', 'publication_steps.csv']:
    copy(live / 'receiver' / name, 'receiver/' + name)
copy(RUN / 'memory_observations.jsonl', 'memory_observations.jsonl')
for label, p in [('release_retained', BUILD / 'tests/result.json'),
                 ('sanitizer_retained', BUILD.parent / 'asan_nopie/tests/result.json')]:
    copy(p, 'retained_tests/' + label + '.json')

# Keep the large raw logs, native trajectories, bags and receiver products in
# their original run directory; record exact sizes and hashes, never rewrite.
raw = [p for p in source.iterdir() if p.is_file()]
raw += [p for p in (source / 'recordings').rglob('*') if p.is_file()]
raw += [p for p in (source / 'kimera_logs').rglob('*') if p.is_file()]
raw += [p for p in live.iterdir() if p.is_file()]
raw += [p for p in (live / 'receiver').iterdir() if p.is_file()]
raw_manifest = {str(p): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(set(raw))}
receiver = Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay')
result = dict(
    artifact_integrity_passed=True, experiment_passed=False,
    status='CAPACITY_FAILED_PARTIAL_CAPTURE',
    protected_files=len(manifest), source_repositories=repos,
    stopped_process_inventory=process_inventory,
    private_bridge_sha256=sha(BUILD / 'libglim_ros.so'),
    private_cpu_sha256=sha(BUILD / 'libgtsam_points.so.1.2.1'),
    receiver_sha256=sha(receiver),
    raw_artifacts=raw_manifest,
    evidence_files={str(p.relative_to(evidence)): sha(p) for p in sorted(evidence.rglob('*')) if p.is_file()},
)
with (HERE / 'integrity.json').open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps({k: v for k, v in result.items() if k not in ['raw_artifacts', 'evidence_files', 'source_repositories']}, indent=2))
print('ARCHIVED_FAILURE', len(raw_manifest), 'raw artifacts;', len(result['evidence_files']), 'small evidence files')
