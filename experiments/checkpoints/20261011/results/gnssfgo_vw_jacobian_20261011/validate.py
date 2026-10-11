"""Final read-only conservation/preservation checks. Does not rerun receiver."""
import collections,csv,json,math
from pathlib import Path
root=Path(__file__).resolve().parent; out=root/'candidate'
journal=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_covariance160_validated_20261010/prepared/full160_on_receiver/events.csv')
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
incoming=collections.Counter();byordinal={};latest={};imu=0;ends=[]
with journal.open() as f:
    for row in csv.reader(f):
        if not row or row[0].startswith('#'):continue
        if row[0]=='I':imu+=1
        if row[0]=='E':ends.append(int(row[1]))
        if row[0]!='B':continue
        key=(row[2],row[3],row[6],row[7],row[5]);incoming[key]+=1
        byordinal[(row[2],row[3],row[4],row[5])]=key;latest[key]=row
dispatch=rows(out/'causal_dispatch.csv');unsupported=rows(out/'unsupported_beliefs.csv')
accounted=collections.Counter((r['source'],r['session'],r['from_index'],r['to_index'],r['revision']) for r in dispatch)
for r in unsupported:accounted[byordinal[(r['source'],r['session'],r['ordinal'],r['revision'])]]+=1
assert incoming==accounted
active={};insert=replace=0
for r in dispatch:
    key=(r['source'],r['session'],r['from_index'],r['to_index'])
    if r['outcome']=='insert':assert key not in active;insert+=1
    else:assert r['outcome']=='replace' and key in active and int(r['revision'])>int(active[key]);replace+=1
    active[key]=r['revision']
noise=rows(out/'source_model_covariance.csv');noise_ids=set();max_error=0.
for r in noise:
    key=tuple(r[k] for k in ('source','session','from_index','to_index'))
    assert r['revision']==active[key] and key not in noise_ids;noise_ids.add(key)
    original=latest[key+(r['revision'],)]
    for i in range(6):
        for j in range(6):
            expected=float(original[16+6*i+j])
            for prefix in ('reported','effective'):
                max_error=max(max_error,abs(float(r[f'{prefix}_{i}{j}'])-expected))
assert noise_ids==set(active) and max_error<=1e-15
summary=json.loads((out/'summary.json').read_text());convergence=rows(out/'publication_convergence.csv')
for r in convergence:assert r['status']=='stationary' and float(r['gradient_after'])<=1e-5
online=rows(out/'online_states.csv');final=rows(out/'states.csv')
for row in online+final:
    assert all(math.isfinite(float(v)) for v in row.values())
assert len(online)==len(final)==summary['support_states']==1298
assert summary['pending_at_end']==0 and summary['revision_duplicates']==0 and summary['revision_stale']==0
assert summary['relative_factor_count']==len(active)==insert
assert summary['active_factor_count']==summary['wnoa_factor_count']+len(active)+3
assert summary['imu_factor_count']==summary['bias_variable_count']==summary['acceleration_variable_count']==0
assert summary['maximum_factor_covariance_abs_difference']<=1e-15
progress=rows(out/'stream_progress.csv')
assert len(ends)==1 and int(progress[-1]['event_count'])==imu+sum(incoming.values())
pre=json.loads((root/'provenance_pre_replay.json').read_text());after=json.loads((root/'provenance_after.json').read_text())
for k in ('protected','helpers','original_outputs','journal','repo_head','repo_status','private_files'):
    assert pre[k]==after[k],k
result=dict(valid=True,input_belief_versions=sum(incoming.values()),dispatched=len(dispatch),unsupported=len(unsupported),
 input_imu_timestamp_events=imu,end_markers=len(ends),insertions=insert,replacements=replace,
 active_sources=dict(collections.Counter(k[0] for k in active)),
 max_final_logged_covariance_vs_input=max_error,max_actual_factor_covariance_error=summary['maximum_factor_covariance_abs_difference'],
 published_states=len(online),final_states=len(final),convergence_checks=len(convergence),
 maximum_publication_gradient=max(float(r['gradient_after']) for r in convergence),
 final_gradient=float(convergence[-1]['gradient_after']),pending_at_end=0,
 final_physical_ns=int(online[-1]['timestamp_ns']),journal_end_ns=ends[0],
 support_span_seconds=summary['duration_seconds'],window_end_elapsed_seconds=(int(online[-1]['timestamp_ns'])-1727775008003475000)/1e9,
 preserved_protected_artifacts=len(pre['protected']),preserved_helper_hashes=len(pre['helpers']),
 preserved_original_outputs=True,preserved_private_build=True,repo_head=after['repo_head'],repo_status=after['repo_status'],
 deterministic_replay_repeat='not run: protocol authorized one candidate replay; repeated Jacobian evaluations passed',
 full_160_second_validation=False,live_throughput_validation=False)
with (root/'FINAL_VALIDATION.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
