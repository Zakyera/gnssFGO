"""Summarize conversion-only tests, preserving prior inputs and output artifacts."""
import hashlib
import json
from collections import defaultdict
from pathlib import Path

H=Path(__file__).resolve().parent
K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def stats(rows):
    assert rows
    return dict(count=len(rows),rejected=sum(r['rejected'] for r in rows),
                min_error=min(r['converted_cache_error'] for r in rows),
                max_error=max(r['converted_cache_error'] for r in rows))
first=[json.loads(line) for line in (K/'results/cases.jsonl').read_text().splitlines()]
nearby=[json.loads(line) for line in (K/'nearby_results/cases.jsonl').read_text().splitlines()]
assert len(first)==196 and len(nearby)==902
for r in first+nearby:
    assert r['original_cache_error']==0 and r['exact_anchor_control_error']==0
    assert r['rejected']==(r['converted_cache_error']>1e-8)
assert read(H/'diagnostic_passed.json')['passed'] and read(H/'nearby_stage1/passed.json')['passed']
for d in (H,H/'nearby_stage1'):
    assert read(d/'cleanup.json')['owned_jobs_exited']
    assert read(d/'cleanup.json')['containers']=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
groups=defaultdict(list)
for r in first:groups[r['label'].rsplit('_',1)[1]].append(r)
near_groups=defaultdict(list)
for r in nearby:near_groups['synthetic' if r['label'].startswith('synthetic') else 'saved_translated'].append(r)
example=next(r for r in nearby if r['label']=='synthetic_nearby_shape2_index20')
assert example['rejected'] and example['self_local_norm']==0
out=dict(
    question='Can frozen CPK9 promotion reject solely because conversion perturbs an identical prior/cache?',
    answer='Yes: demonstrated numerical false rejection in isolated unchanged-anchor tests.',
    threshold=1e-8,first_test=stats(first),nearby_test=stats(nearby),
    first_groups={k:stats(v) for k,v in groups.items()},
    nearby_groups={k:stats(v) for k,v in near_groups.items()},
    representative_pose_synthetic_prior_example=example,
    representative_pose=dict(source='last published CP99 pose1475; NOT failed theta at update1476',
       position=[8.672789906320816,82.15889280701087,4.272652190999145],
       quaternion_xyzw=[-.09373492640277502,-.058385719301346756,.1014050752205833,.9886970624946703]),
    example_prior=dict(type='synthetic SPD unary Pose3 Hessian',H_diagonal=[1,1,1,1,1000000,1],eta=[0]*6,constant=1),
    controls=dict(original_cache_error_zero_all=True,exact_unchanged_anchor_clone_error_zero_all=True,
       deliberate_cache_corruption_rejected=True,failed_promotion_graph_and_locks_unchanged=True,
       frozen_header_implementation_used=True,no_guard_tolerance_change=True),
    scope=dict(source_runs=0,optimizer_runs=0,estimator_builds=0,standalone_diagnostic_builds=2,
       production_fix_applied=False,actual_cp99_failure_reproduced=False,settings_changes=0),
    limits=[
       'CP99 failure magnitude, failed branch and actual prior/anchor were not recorded.',
       'Synthetic unary prior is not reconstructed CP99 marginal prior.',
       'All35 original saved CP98 priors pass; they cover early8.5-12s, not the failed299.5s event.',
       'All738 nearby-translated saved-prior cases also pass.56/164 synthetic nearby cases reject.',
       'CP98 recorder stores aggregate cache, not per-slot cache; constructed identical per-prior cache isolates conversion only.',
       'World-coordinate stress translations and finite-difference step sweep are diagnostic controls, not live configuration changes.',
       'Exact-anchor clone is a control, not an implemented production fix or a proof for moved-anchor linearization.',
       'This does not establish the cause of GLIM ATE regression or validate covariance calibration.'],
    next_step='Private isolated unchanged-anchor exact-Gaussian implementation plus unchanged moved-anchor behavior, corruption/ownership/type/rollback regressions and separate failure-branch logging; require approval before any new live ON.')
out['diagnostic_artifact_sha256']={str(p):sha(p) for p in [K/'probe.cpp',K/'nearby.cpp',K/'probe',K/'nearby',K/'dependencies.d',K/'nearby_dependencies.d',K/'results/cases.jsonl',K/'nearby_results/cases.jsonl',H/'PROTOCOL.txt',H/'analyze.py',H/'probe.log',H/'nearby_stage1/probe.log']}
with (H/'analysis.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:out[k] for k in ('answer','first_test','nearby_test','nearby_groups','representative_pose_synthetic_prior_example')},indent=2))
