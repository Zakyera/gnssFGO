"""Final bounded-checkpoint validation; no estimator execution."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
a=json.loads((ROOT/'provenance_pre_replay.json').read_text())
b=json.loads((ROOT/'provenance_after.json').read_text())
c=json.loads((ROOT/'comparison.json').read_text())
checks={field:a[field]==b[field] for field in ('protected','helpers','original_outputs','journal','repo_head','repo_status')}
checks['protected_hashes_valid']=not b['mismatches']
checks['private_preexisting_files_unchanged']=all(b['private_files'].get(k)==v for k,v in a['private_files'].items())
checks['only_expected_build_addition']=set(b['private_files'])-set(a['private_files'])=={'build/tests_sanitizer'}
checks['recorded_behavior_identical']=c['all_semantics_equal']
for label in ('unit_final','sanitizer','build'):
    commands=json.loads((ROOT/f'command_{label}.json').read_text())['commands']
    checks[label]=all(x['returncode']==0 for x in commands)
for case in ('baseline','cached'):
    d=c[case]
    checks[case+'_expected_stop']=(d['failed']['index']=='1272' and d['failed']['status']=='budget_exhausted'
         and d['failed']['attempted']=='20' and float(d['failed']['gradient_after'])>1e-5)
    checks[case+'_source_accounting']=(d['revision_outcomes']=={'insert':1880,'replace':73241} and d['published_rows']==1272)
    checks[case+'_no_final_trajectory']=(ROOT/case/'trajectory.tum').stat().st_size==0
result=dict(checks=checks,all_pass=all(checks.values()),
    classification='CACHE_ONLY_BEHAVIOR_PRESERVATION_VALIDATED',
    original_publication_failure_resolved=False,live_throughput_validated=False,
    original_artifacts_unchanged=True,production_changed=False,fixed_lag_added=False,
    cached_executable_sha256=b['private_files']['build/replay'],repo_head=b['repo_head'])
with (ROOT/'FINAL_VALIDATION.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
assert result['all_pass']
