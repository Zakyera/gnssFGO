"""Integrity and independent count checks only; no estimator execution."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
PRIOR = HERE.parent / 'shared_linearization_s5_live160_20261012'
ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared')
REC = ROOT / 'capacity160_on_receiver/receiver'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    output = HERE / 'integrity.json'
    assert not output.exists(), 'Refuse to overwrite integrity record'
    prior = json.loads((PRIOR / 'integrity.json').read_text())
    protected = json.loads((ROOT / 'protected_manifest.json').read_text())
    summary = json.loads((HERE / 'summary.json').read_text())
    for path, expected in protected.items():
        assert sha(path) == expected, path
    for path, expected in prior['raw_artifacts'].items():
        assert Path(path).stat().st_size == expected['bytes'], path
        assert sha(path) == expected['sha256'], path
    for path, expected in prior['evidence_files'].items():
        assert sha(PRIOR / 'evidence' / path) == expected, path
    for path, expected in summary['input_sha256'].items():
        assert sha(path) == expected, path
    repositories = {}
    for path, expected in prior['source_repositories'].items():
        def git(*args):
            return subprocess.check_output(['git', '-C', path, *args], text=True).strip()
        current = dict(head=git('rev-parse', 'HEAD'), status=git('status', '--porcelain'))
        assert current == expected, (path, current, expected)
        repositories[path] = current
    with (REC / 'publication_convergence.csv').open() as stream:
        convergence = list(csv.DictReader(stream))
    with (REC / 'publication_steps.csv').open() as stream:
        steps = list(csv.DictReader(stream))
    support = [r for r in convergence if r['phase'] == 'support']
    assert len(convergence) == 1481 and len(support) == 1480
    assert len(steps) == 2511
    assert sum(r['recovery'] == '0' for r in steps) == 2446
    assert sum(r['recovery'] == '1' for r in steps) == 65
    assert sum(int(r['attempted']) for r in support) == 2446
    assert abs(sum(float(r['seconds']) * 1000 for r in support) -
               summary['blocks']['0-148.580001']['refinement_ms']['total']) < 1e-6
    for path in (HERE / 'analyze.py', HERE / 'verify.py'):
        compile(path.read_text(), str(path), 'exec')
    result = dict(
        task='S5a', integrity_passed=True, estimator_executions=0,
        protected_files=len(protected), frozen_raw_artifacts=len(prior['raw_artifacts']),
        frozen_evidence_files=len(prior['evidence_files']),
        analysis_inputs=len(summary['input_sha256']), source_repositories=repositories,
        independent_counts=dict(publications=len(convergence), support_publications=len(support),
                                ordinary_steps=2446, recovery_rows=65),
        syntax_checks=['analyze.py', 'verify.py'],
        audit_files={p.name: sha(p) for p in sorted(HERE.iterdir()) if p.is_file()},
        limits='Integrity/count audit only. S5 remains capacity-failed and partially validated; no new estimator tests.')
    with output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items()
                      if k not in ['source_repositories', 'audit_files']}, indent=2))


if __name__ == '__main__':
    main()
