"""Summarize completed isolated qualification; no source runs or rescoring."""
import hashlib
import json
from pathlib import Path
import re
H=Path(__file__).resolve().parent
K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
BASE=H.parent/'prior_conversion_offline_cpv100_20261103'
BK=K.parent/BASE.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert read(H/'qualification_v3/passed.json')['passed']
assert read(H/'qualification_v3/cleanup.json')['owned_jobs_exited']
rows=[json.loads(s) for s in (K/'results_v3/cases.jsonl').read_text().splitlines()]
by_name={r['case']:r for r in rows}
assert len(by_name)==len(rows)==1100
old=[]
for sub in ('results','nearby_results'):
    old += [json.loads(s) for s in (BK/sub/'cases.jsonl').read_text().splitlines()]
assert len(old)==1098
max_error_difference=0
for r in old:
    now=by_name[r['label']]
    assert now['baseline_rejected']==r['rejected']
    max_error_difference=max(max_error_difference,abs(now['baseline_error']-r['converted_cache_error']))
    assert now['candidate_error']==0
assert max_error_difference<1e-14
log=(H/'qualification_v3/tests.log').read_text()
summary=re.search(r'PASS cases=(\d+) baseline_rejections=(\d+) candidate_rejections=(\d+) moved_checks=(\d+) moved_max=([^ ]+) objective_max=([^ ]+) negative_controls=(\d+) lifecycle_pose_difference=([^\n]+)',log)
assert summary,log[-1500:]
n,old_reject,new_reject,moved,moved_error,objective_error,negative,pose_difference=summary.groups()
assert int(n)==1100 and int(old_reject)==126 and int(new_reject)==0
assert float(moved_error)==float(objective_error)==0 and int(negative)==15
lifecycle=re.findall(r'LIFECYCLE candidate=(\d) origin=([^ ]+) boundaries=(\d+) promotions=(\d+)',log)
assert len(lifecycle)==4 and all(int(r[2])==15 and int(r[3])>=15 for r in lifecycle)
example=by_name['synthetic_nearby_shape2_index20'];assert example['baseline_rejected']
message=(K/'results_v3/failure_message.txt').read_text()
assert 'slot=1 ' in message and 'original_cache_error=' in message and 'converted_cache_error=' in message
out=dict(passed=True,scope='Private header candidate; no deployed estimator or live capture',
         exact_anchor_cases=1100,prior_cp100_cases_rechecked=1098,baseline_rejections=126,candidate_rejections=0,
         moved_anchor_comparisons=int(moved),moved_linearization_relative_error=float(moved_error),
         moved_objective_difference=float(objective_error),negative_controls=15,
         example=example,baseline_error_reproduction_max_absolute_difference=max_error_difference,
         synthetic_lifecycle=[dict(candidate=bool(int(c)),origin=float(o),boundaries=int(b),promotions=int(p)) for c,o,b,p in lifecycle],
         synthetic_near_origin_baseline_pose_difference=float(pose_difference),
         harness_correction=dict(initial_failure_preserved=True,candidate_headers_unchanged=True,
             cause='Test demanded bitwise equality to unsymmetrized raw matrix instead of stored Gaussian.',
             raw_vs_stored_relative_error=float(re.search(r'HARNESS raw_vs_assembled=([^\n]+)',log).group(1))),
         lifecycle_harness_correction=dict(failed_stages_preserved=True,
             baseline_crashed_before_first_marginal_prior=True,
             cause='Absolute key-index ordering groups violated CCOLAMD range under incremental subsets; test now uses 0 oldest / 1 remaining.',
             corrected_fixture_checks_leaf_and_retained_key_counts=True,
             gdb_unavailable_no_installation=True,asan_attempt_inconclusive=True),
         diagnostics=dict(summary_has_separate_errors_slot_keys=True,typed_exception_retains_matrices_anchor=True,
             typed_fixture_export_verified=True,live_matrix_export_handler_installed=False),
         unchanged=dict(threshold=1e-8,original_cache_check=True,moved_anchor_code=True,estimator_binaries=True,settings=True),
         limits=['Actual CP99 failed theta/prior was not recorded; historical cause remains unproven.',
                 'Synthetic iSAM2 lifecycle is not the full Kimera/CBS backend integration or a live sequence.',
                 'The fast path uses exact value equality after validation, never a near-anchor epsilon.',
                 'Failure matrix/anchor fields are accessible to a typed handler; only summary appears automatically in standard exception logs.',
                 'No covariance calibration, GLIM ATE causality, timing or full-run success claim.'],
         next_step='Separate private Kimera/CBS integration build and regression validation; deploy or launch a live run only after explicit approval.')
with (H/'analysis.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))

