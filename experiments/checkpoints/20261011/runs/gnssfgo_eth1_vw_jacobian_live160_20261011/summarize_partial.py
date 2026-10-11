"""Read-only assessment of the deliberately stopped capture, not a new run."""
import csv
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
P=ROOT/'prepared'; S=P/'full160_on_source'; L=P/'full160_on_receiver'; R=L/'receiver'
sys.path.insert(0,str(P))
from experiment import file_sha
from coverage_checks import load
sys.path.insert(0,'/home/yeranis/repos/gnssFGO/tools')
from live_receiver_protocol import parse_belief
spec=importlib.util.spec_from_file_location('retained_timing',ROOT.parent/'glim_cbs_diagnostic_budget_20261010/compare_timing.py')
timing=importlib.util.module_from_spec(spec); spec.loader.exec_module(timing)
read=lambda p:list(csv.DictReader(p.open()))
monitor=[json.loads(x) for x in (P/'full160_on_control/monitor.jsonl').read_text().splitlines()]
monitor=[x for x in monitor if x['published_poses'] and x['last_imu_ns']]
T0=1727775008003475000
elapsed=lambda r:(r['last_imu_ns']-T0)*1e-9
conv=read(R/'publication_convergence.csv')
summary=json.loads((R/'summary.json').read_text())
transport=json.loads((L/'transport_status.json').read_text())
expected={}
glog,=[p for p in (S/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
for kind,sid,path in [('glim','glim_hesai',glog),('kimera','kimera_front',S/'estimators.log')]:
    for line in path.open():
        b=parse_belief(kind,line)
        if b is None: continue
        key=(sid,b['i'],b['j']); payload=(b['ti'],b['tj'],*b['mean'],*b['covariance'])
        h=expected.setdefault(key,[])
        if not h or h[-1]!=payload: h.append(payload)
received={}; imu=[]; end=0
for r in csv.reader((L/'events.csv').open()):
    if r[0]=='I': imu.append(int(r[2]))
    if r[0]=='E': end+=1
    if r[0]!='B': continue
    key=(r[2],int(r[6]),int(r[7])); h=received.setdefault(key,[])
    assert int(r[5])==len(h)
    h.append((int(r[8]),int(r[9]),*map(float,r[10:])))
assert received==expected
dispatch=read(R/'causal_dispatch.csv'); unsupported=read(R/'unsupported_beliefs.csv')
versions=sum(map(len,received.values()))
assert versions==len(dispatch)+len(unsupported)==transport['beliefs']
assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in conv)
assert len(conv)==summary['support_states']
assert transport['queued']==0 and summary['pending_at_end']==0
assert not (P/'full160_on_control/complete.json').exists()
assert (P/'full160_on_control/failed.txt').exists()
manifest=json.loads((P/'protected_manifest.json').read_text())
for path,sha in manifest.items(): assert file_sha(path)==sha,path
coverage={}
for role,path,is_k in [('native_glim',S/'odom_imu.txt',False),
    ('native_kimera',S/'kimera_odometry.csv',True),
    ('fused_online',R/'online_trajectory.tum',False),('fused_retrospective',R/'trajectory.tum',False)]:
    t,p=load(path,is_k)
    assert np.isfinite(t).all() and np.isfinite(p).all() and (np.diff(t)>0).all()
    coverage[role]=dict(rows=len(t),start_elapsed_s=float(t[0]-T0*1e-9),
        end_elapsed_s=float(t[-1]-T0*1e-9),span_s=float(t[-1]-t[0]),
        path_length_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
        maximum_pose_gap_s=float(np.diff(t).max()))
source=timing.read(S)
windows=[]
for lo,hi in [(0,120),(120,130),(130,144)]:
    scans=[r for r in source['scan'].values() if lo<=r['stamp']-timing.T0<hi]
    c=[r for r in conv if lo<=(int(r['physical_ns'])-T0)*1e-9<hi]
    pubs=source['cbs'].get('publish_odometry',{})
    ids=[i for i,r in source['scan'].items() if lo<=r['stamp']-timing.T0<hi]
    m=[r for r in monitor if lo<=elapsed(r)<hi]
    windows.append(dict(seconds=[lo,hi],glim_scan_ms={k:timing.stats([r[k] for r in scans])
        for k in ['pre_callback','smoother','post_callback','total']},
        glim_publication_age_s=timing.stats([pubs[i][1]-source['scan'][i]['stamp'] for i in ids if i in pubs]),
        receiver_publication_refinement_s=timing.stats([float(r['seconds']) for r in c]),
        receiver_output_age_s=timing.stats([r['output_age_seconds'] for r in m]),
        receiver_queue=timing.stats([r['queued'] for r in m])))
bad=[r for r in monitor if r['queued']>=1000]
first=bad[0]; peak=max(monitor,key=lambda r:r['queued'])
cov=read(R/'source_model_covariance.csv')
covdiff=max(abs(float(r['reported_'+str(i)+str(j)])-float(r['effective_'+str(i)+str(j)])) for r in cov for i in range(6) for j in range(6))
result=dict(full_160_validation_passed=False,classification='LIVE_THROUGHPUT_GATE_FAILED',
    deliberate_stop=True,source_crash=False,requested_s=160,imu_span_s=(imu[-1]-imu[0])*1e-9,
    first_sampled_queue_failure_elapsed_s=elapsed(first),first_sampled_queue_failure=first['queued'],
    peak_sampled_queue=peak['queued'],peak_sampled_queue_elapsed_s=elapsed(peak),
    transport_peak_queue=transport['max_queue'],final_queue=transport['queued'],
    output_age_s=timing.stats([r['output_age_seconds'] for r in monitor]),
    stationary_publications=len(conv),maximum_gradient=max(float(r['gradient_after']) for r in conv),
    refinement_attempts=timing.stats([int(r['attempted']) for r in conv]),
    exported_versions=versions,exact_journal_payload_and_order_conservation=True,
    graph_dispatch_records=len(dispatch),unsupported_records=len(unsupported),
    unsupported_reasons=sorted({r['reason'] for r in unsupported}),
    inserts=sum(r['outcome']=='insert' for r in dispatch),replacements=sum(r['outcome']=='replace' for r in dispatch),
    active_source_ids=read(R/'source_accounting.csv'),final_covariance_reported_vs_effective_max=covdiff,
    actual_factor_covariance_error=summary['maximum_factor_covariance_abs_difference'],
    protected_files_unchanged=len(manifest),coverage=coverage,timing_windows=windows,
    glim_timing_summary=source['summary'],summary=summary,
    no_new_replay='No deterministic repeat or full postvalidation after the capture gate failed.',
    caveat='Partial capture only. No 160s coverage, ATE comparison, deterministic-live replay certification, or general realtime guarantee.')
with (ROOT/'partial_validation.json').open('x') as f: json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['summary','glim_timing_summary','timing_windows']},indent=2))
print(json.dumps(windows,indent=2))
