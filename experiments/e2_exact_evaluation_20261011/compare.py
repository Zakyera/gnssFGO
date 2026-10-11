"""Exact frozen replay comparison; timing fields alone are excluded."""
import collections,csv,hashlib,json,sys
from pathlib import Path
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
CASES={'live60':WS/'gnssfgo_e1_live60_20261011/prepared/live60_on_receiver',
       'partial143':WS/'gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/full160_on_receiver'}
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def quantile(data,p):
    a=sorted(data);i=(len(a)-1)*p;lo=int(i);hi=min(lo+1,len(a)-1)
    return a[lo]+(a[hi]-a[lo])*(i-lo)
def timing(data):
    v=[float(r['seconds']) for r in data]
    return dict(n=len(v),sum=sum(v),mean=sum(v)/len(v),median=quantile(v,.5),p95=quantile(v,.95),maximum=max(v))
case=sys.argv[1];assert case in CASES
repeat=len(sys.argv)>2 and sys.argv[2]=='repeat';suffix='_repeat' if repeat else ''
reference=ROOT/('reference_'+case+suffix);candidate=ROOT/('candidate_'+case+suffix)
assert reference.is_dir() and candidate.is_dir()
exact={};differences=[]
for p in sorted((CASES[case]/'receiver').iterdir()):
    if not p.is_file():continue
    name=p.name
    versions=[p,reference/name,candidate/name]
    if repeat:versions.extend([ROOT/('reference_'+case)/name,ROOT/('candidate_'+case)/name])
    if name in ['publication_convergence.csv','revision_events.csv','stream_progress.csv']:
        excluded={'publication_convergence.csv':{'seconds'},'revision_events.csv':{'update_seconds'},
                  'stream_progress.csv':{'wall_unix_ns'}}[name]
        data=[[{k:v for k,v in r.items() if k not in excluded} for r in rows(path)] for path in versions]
    elif name=='summary.json':
        data=[{k:v for k,v in json.loads(path.read_text()).items() if k not in ['wall_seconds','revision_update_seconds']} for path in versions]
    else:data=[sha(path) for path in versions]
    exact[name]=all(d==data[0] for d in data[1:])
    if not exact[name]:differences.append(name)
assert not differences,('EXACT VALIDATION FAILED',differences)
summary=json.loads((candidate/'summary.json').read_text())
convergence=rows(candidate/'publication_convergence.csv')
assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in convergence)
assert summary['pending_at_end']==summary['revision_duplicates']==summary['revision_stale']==0
assert summary['imu_factor_count']==summary['bias_variable_count']==summary['acceleration_variable_count']==0
assert summary['active_factor_count']==summary['wnoa_factor_count']+summary['relative_factor_count']+3
assert summary['maximum_factor_covariance_abs_difference']<=1e-12
clock={r['state_index']:int(r['available_timestamp_ns']) for r in rows(candidate/'support_clock.csv')}
arrivals=collections.Counter();ordinals={};imu=0;end=0
with (CASES[case]/'events.csv').open() as f:
    for r in csv.reader(f):
        if not r:continue
        if r[0]=='I':imu+=1
        if r[0]=='E':end+=1
        if r[0]!='B':continue
        key=(r[2],r[3],r[6],r[7],r[5]);arrivals[key]+=1
        ordinals[(r[2],r[3],r[4],r[5])]=key
accounted=collections.Counter();active={}
for r in rows(candidate/'causal_dispatch.csv'):
    key=tuple(r[k] for k in ['source','session','from_index','to_index']);revision=int(r['revision'])
    assert int(r['arrival_ns'])<=int(r['insertion_ns']) and clock[r['required_support']]<=int(r['insertion_ns'])
    assert int(r['required_support'])<=int(r['created_support'])
    if r['outcome']=='insert':assert key not in active
    else:assert r['outcome']=='replace' and key in active and revision>active[key]
    active[key]=revision;accounted[key+(r['revision'],)]+=1
for r in rows(candidate/'unsupported_beliefs.csv'):
    accounted[ordinals[(r['source'],r['session'],r['ordinal'],r['revision'])]]+=1
assert arrivals==accounted and end==1 and len(active)==summary['relative_factor_count']
stats={};start=int(convergence[0]['physical_ns'])
for mode,directory in [('reference',reference),('candidate',candidate)]:
    allrows=rows(directory/'publication_convergence.csv')
    stats[mode]={'all':timing(allrows)}
    for label,lo,hi in [('early',-1,20),('middle',20,40),('late',40,1000)]:
        selected=[r for r in allrows if lo<=(int(r['physical_ns'])-start)/1e9<hi]
        if selected:stats[mode][label]=timing(selected)
    stats[mode]['receiver_wall_seconds']=json.loads((directory/'summary.json').read_text())['wall_seconds']
    stats[mode]['process_wall_seconds']=json.loads((ROOT/('command_'+mode+'_'+case+suffix+'.json')).read_text())['seconds']
last={r['kind']:r for r in rows(candidate/'e2_memo.csv') if r['phase']=='final'}
result=dict(valid=True,case=case,repeat=repeat,exact=exact,maximum_publication_gradient=max(float(r['gradient_after']) for r in convergence),
    support_states=summary['support_states'],versions=sum(arrivals.values()),timestamp_events=imu,admitted_ids=len(active),
    unsupported=summary['beliefs_skipped_no_support'],replacements=summary['revision_removals'],
    max_factor_covariance_difference=summary['maximum_factor_covariance_abs_difference'],
    cache_totals=last,timing=stats)
with (ROOT/(case+suffix+'_comparison.json')).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
