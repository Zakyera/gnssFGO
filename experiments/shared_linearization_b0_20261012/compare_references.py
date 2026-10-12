"""Record B0 against both retained references; recheck frozen S1 evidence."""
from pathlib import Path
import hashlib
import json
from summarize import extra

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = WS / 'runs/gnssfgo_e1b4a_timing40_20261012'
RUN = WS / 'runs/gnssfgo_shared_linearization_b0_20261012'

def read(p):
    return json.loads(p.read_text())

def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()

def main():
    assert read(RUN/'validation_logs/complete.json')['passed']
    assert not (RUN/'validation_logs/failed.txt').exists()
    previous = read(HERE.parent/'e1b4a_timing40_20261012/comparison.json')
    fresh = read(HERE/'comparison.json')
    protected = read(OLD/'prepared/protected_manifest.json')
    for path, expected in protected.items():
        assert sha(Path(path)) == expected, path
    s1 = read(HERE.parent/'shared_linearization_s1_20261012/INTEGRITY.json')
    for path, record in s1['inspected_evidence'].items():
        assert sha(WS/path) == record['sha256'], path
    refs = dict(unbatched=previous['baseline'], previous_batched=previous['candidate'],
                fresh_batched=fresh['candidate'])
    result = dict(
        inherited_protected_hashes=len(protected),
        s1_evidence_hashes=len(s1['inspected_evidence']),
        source_repositories=fresh['source_repositories'],
        runs=dict(unbatched=previous['baseline_run'], previous_batched=str(OLD), fresh_batched=str(RUN)),
        timing=refs,
        extra=dict(unbatched=previous['baseline_extra'], previous_batched=previous['candidate_extra'],
                   fresh_batched=extra(RUN)),
        controller_seconds=dict(unbatched=previous['baseline_controller']['seconds'],
                                previous_batched=previous['candidate_controller']['seconds'],
                                fresh_batched=fresh['candidate_controller']['seconds']),
        fresh_over_previous_batched={scope:{metric:refs['fresh_batched'][scope][metric]/refs['previous_batched'][scope][metric]
              for metric in ['median','p95','max','mean']} for scope in ['all_local_root_ms']},
        functional_pass=fresh['functional_pass'],
        retained_unbatched_local_gate_pass=fresh['local_timing_gate_pass'],
        complete_scan_headroom_pass=fresh['first40s_scan_headroom_pass'],
        limitation='Three independent live captures; timing and scheduling can differ. No shared-linearization implementation, no accuracy claim, no longer-run capacity proof.')
    result['fresh_over_previous_batched']['complete_scan']={metric:refs['fresh_batched']['legacy']['scan_total_ms'][metric]/refs['previous_batched']['legacy']['scan_total_ms'][metric]
                       for metric in ['median','p95','max','mean']}
    with (HERE/'reference_comparison.json').open('x') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['timing','extra','source_repositories']}, indent=2))

if __name__ == '__main__':
    main()
