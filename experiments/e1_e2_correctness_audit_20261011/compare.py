"""Exact artifacts and independent causal/conservation audit, not GT scoring."""
import collections,csv,hashlib,json,sys
from pathlib import Path
ROOT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
SAVED=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_live60_20261011/prepared/live60_on_receiver')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def payload(p):
    excluded={'publication_convergence.csv':{'seconds'},'revision_events.csv':{'update_seconds'},
              'stream_progress.csv':{'wall_unix_ns'}}
    if p.name in excluded:return [{k:v for k,v in r.items() if k not in excluded[p.name]} for r in rows(p)]
    if p.name=='summary.json':return {k:v for k,v in json.loads(p.read_text()).items() if k not in ['wall_seconds','revision_update_seconds']}
    return sha(p)
def quantile(data,q):
    a=sorted(data);i=(len(a)-1)*q;lo=int(i);hi=min(lo+1,len(a)-1)
    return a[lo]+(a[hi]-a[lo])*(i-lo)
case=sys.argv[1];assert case in ['off1','on1','on2','off2']
directory=ROOT/case;command=json.loads((ROOT/(case+'.json')).read_text());assert command['passed']
assert json.loads((ROOT/'environment_control.json').read_text())['passed']
# The pinned, unchanged executable is the matched oracle. Comparing to the
# old unpinned oracle failed; check_environment.py retains that discrepancy.
exact={p.name:payload(ROOT/'reference_pinned'/p.name)==payload(directory/p.name) for p in (SAVED/'receiver').iterdir() if p.is_file()}
assert all(exact.values()),exact
summary=json.loads((directory/'summary.json').read_text())
convergence=rows(directory/'publication_convergence.csv')
assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in convergence)
assert summary['pending_at_end']==summary['revision_duplicates']==summary['revision_stale']==0
assert summary['imu_factor_count']==summary['bias_variable_count']==summary['acceleration_variable_count']==0
assert summary['active_factor_count']==summary['wnoa_factor_count']+summary['relative_factor_count']+3
assert summary['maximum_factor_covariance_abs_difference']<=1e-12
clock={r['state_index']:int(r['available_timestamp_ns']) for r in rows(directory/'support_clock.csv')}
arrivals=collections.Counter();ordinals={};imu=0;end=0
with (SAVED/'events.csv').open() as f:
    for r in csv.reader(f):
        if not r:continue
        if r[0]=='I':imu+=1
        if r[0]=='E':end+=1
        if r[0]!='B':continue
        key=(r[2],r[3],r[6],r[7],r[5]);arrivals[key]+=1
        ordinals[(r[2],r[3],r[4],r[5])]=key
accounted=collections.Counter();active={}
for r in rows(directory/'causal_dispatch.csv'):
    key=tuple(r[k] for k in ['source','session','from_index','to_index']);revision=int(r['revision'])
    assert int(r['arrival_ns'])<=int(r['insertion_ns']) and clock[r['required_support']]<=int(r['insertion_ns'])
    assert int(r['required_support'])<=int(r['created_support'])
    if r['outcome']=='insert':assert key not in active
    else:assert r['outcome']=='replace' and key in active and revision>active[key]
    active[key]=revision;accounted[key+(r['revision'],)]+=1
for r in rows(directory/'unsupported_beliefs.csv'):
    accounted[ordinals[(r['source'],r['session'],r['ordinal'],r['revision'])]]+=1
assert arrivals==accounted and end==1 and len(active)==summary['relative_factor_count']
timing={};start=int(convergence[0]['physical_ns'])
for label,lo,hi in [('all',-1,1000),('early',-1,20),('middle',20,40),('late',40,1000)]:
    values=[float(r['seconds']) for r in convergence if lo<=(int(r['physical_ns'])-start)/1e9<hi]
    timing[label]=dict(n=len(values),total=sum(values),median=quantile(values,.5),p95=quantile(values,.95),maximum=max(values))
last={r['kind']:r for r in rows(directory/'e2_memo.csv') if r['phase']=='final'}
result=dict(valid=True,case=case,exact=exact,supports=summary['support_states'],versions=sum(arrivals.values()),
    replacements=summary['revision_removals'],unsupported=summary['beliefs_skipped_no_support'],ids=len(active),
    gradient_max=max(float(r['gradient_after']) for r in convergence),
    covariance_max_difference=summary['maximum_factor_covariance_abs_difference'],
    matrix_reuse=last,timing=timing,process_seconds=command['seconds'],max_rss_kib=command['max_rss_kib'])
with (ROOT/(case+'_comparison.json')).open('x') as f:json.dump(result,f,indent=2)
print(case,'exact products',len(exact),'versions',result['versions'],'wall',result['process_seconds'])
