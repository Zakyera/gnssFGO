"""Read-only evidence for the stopped validation; no replay or new scoring."""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parent
P=ROOT/'prepared'
SOURCE=P/'full160_on_source'
LIVE=P/'full160_on_receiver'
OLD=ROOT.parent/'gnssfgo_eth1_covariance160_20261010/prepared'
PERF=ROOT.parent/'glim_cbs_diagnostic_budget_20261010'
T0=1727775008.003475
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):
    with p.open() as f:return list(csv.DictReader(f))
def stats(a):
    a=np.asarray(a,float)
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))
protected=json.loads((P/'protected_manifest.json').read_text())
protected.update(json.loads((ROOT/'additional_protected_evidence.json').read_text()))
for path,digest in protected.items():assert sha(path)==digest,path
assert not (P/'full160_on_control/complete.json').exists()
assert (P/'full160_on_control/failed.txt').exists()
assert (LIVE/'failed.txt').exists() and not (LIVE/'complete.marker').exists()
conv=read(LIVE/'receiver/publication_convergence.csv')
bad=[(i+2,r) for i,r in enumerate(conv) if r['status']!='stationary' or float(r['gradient_after'])>1e-5]
assert len(bad)==1 and bad[0][1]['status']=='budget_exhausted'
line,failure=bad[0]
steps=[r for r in read(LIVE/'receiver/publication_steps.csv') if r['index']==failure['index']]
monitor=[json.loads(s) for s in (P/'full160_on_control/monitor.jsonl').open()]
moving=[r for r in monitor if r['published_poses'] and r['last_imu_ns']]
first_queue=next(r for r in moving if r['queued']>=1000)
transport=json.loads((LIVE/'transport_status.json').read_text())
spec=importlib.util.spec_from_file_location('timing',PERF/'compare_timing.py')
timing=importlib.util.module_from_spec(spec);spec.loader.exec_module(timing)
data={k:timing.read(v) for k,v in {
    'corrected_on':SOURCE,'previous_on':OLD/'full160_on_source','off':OLD/'full160_off_source'}.items()}
assert data['corrected_on']['summary']['max_diagnostic_samples']==0
assert not data['corrected_on']['summary']['native_alignment_failures']
glog=Path(data['corrected_on']['summary']['path'])
assert 'MEAN_ALIGNMENT_UNAVAILABLE' not in (SOURCE/'estimators.log').read_text()
windows=[]
for lo,hi in [(0,115),(0,60),(60,80),(80,100),(100,115)]:
    ids=[i for i,r in data['corrected_on']['scan'].items() if lo<=r['stamp']-T0<hi]
    stamps=[data['corrected_on']['scan'][i]['stamp'] for i in ids]
    window=dict(seconds=[lo,hi],scans=len(ids),cases={})
    for name,d in data.items():
        assert [d['scan'][i]['stamp'] for i in ids]==stamps
        pub=d['cbs']['publish_odometry']
        window['cases'][name]=dict(scan_ms={k:stats([d['scan'][i][k] for i in ids])
            for k in ['smoother','pre_callback','post_callback','total']},
            publication_age_s=stats([pub[i][1]-d['scan'][i]['stamp'] for i in ids if i in pub]))
    windows.append(window)
source_diag=data['corrected_on']['summary']
own_pub=data['corrected_on']['cbs']['publish_odometry']
source_diag['all_published_pose_age_s']=stats([own_pub[i][1]-r['stamp']
    for i,r in data['corrected_on']['scan'].items() if i in own_pub])
off_conv=read(OLD/'full160_off_receiver/receiver/publication_convergence.csv')
by_time=[]
for lo,hi in [(0,100),(100,115),(115,120),(120,125),(125,130)]:
    row=dict(seconds=[lo,hi],cases={})
    for name,rows in [('off',off_conv),('on',conv)]:
        selected=[r for r in rows if lo<=int(r['physical_ns'])*1e-9-T0<hi]
        if selected:row['cases'][name]=dict(refinement_seconds=stats([r['seconds'] for r in selected]),
            attempted=stats([r['attempted'] for r in selected]),
            statuses={s:sum(r['status']==s for r in selected) for s in {r['status'] for r in selected}})
    by_time.append(row)
online=np.loadtxt(LIVE/'receiver/online_trajectory.tum')
def trajectory(p,csv_format=False):
    if csv_format:
        rows=read(p)
        assert all(r['%time']==r['field.header.stamp'] for r in rows)
        times=np.array([int(r['field.header.stamp'])*1e-9 for r in rows])
        xyz=np.array([[float(r['field.pose.pose.position.'+a]) for a in ['x','y','z']] for r in rows])
    else:
        a=np.loadtxt(p);times=a[:,0];xyz=a[:,1:4]
    return dict(rows=len(times),first=float(times[0]),last=float(times[-1]),span_s=float(times[-1]-times[0]),
        path_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),sha256=sha(p))
events=list(csv.reader((LIVE/'events.csv').open()))
imu=[int(r[2]) for r in events if r[0]=='I']
versions=sum(r[0]=='B' for r in events)
result=dict(passed=False,classification='FULL160_NOT_VALIDATED_RECEIVER_THROUGHPUT_AND_PUBLICATION_FAILURE',
    stop='Owned source group SIGINT after growing receiver backlog; captured sensor span about130s, not160. No retry.',
    first_queue_limit_exceedance=first_queue,
    maximum_sampled_queue=max(r['queued'] for r in moving),
    maximum_sampled_output_age_s=max(r['output_age_seconds'] for r in moving),
    receiver_failure=dict(csv_line=line,elapsed_from_first_imu_s=int(failure['physical_ns'])*1e-9-T0,
        convergence_row=failure,gradient_limit=1e-5,iteration_limit=20,
        failed_support_present_in_online=bool(np.any(np.abs(online[:,0]-int(failure['physical_ns'])*1e-9)<1e-6)),
        refinement_steps=steps),
    final_transport=transport,clock_samples=len(imu),clock_span_s=(imu[-1]-imu[0])*1e-9,
    journal_versions=versions,source_timing=source_diag,matched_source_timing=windows,
    receiver_refinement_by_time=by_time,
    native_glim=trajectory(SOURCE/'odom_imu.txt'),native_kimera=trajectory(SOURCE/'kimera_odometry.csv',True),
    fused_online=trajectory(LIVE/'receiver/online_trajectory.tum'),
    prior_off_passed=json.loads((OLD/'full160_off_validation.json').read_text())['passed'],
    protected_entries_unchanged=len(protected),
    blocked_checks=['full160 coverage','complete journal consumption','successful final publication','deterministic replay','matched OFF/ON accuracy scoring'],
    note='Failed trajectory.tum/states.csv are a retained diagnostic snapshot, not a valid final full160 result. No covariance/solver tuning or extra optimization executed.')
assert not result['receiver_failure']['failed_support_present_in_online']
with (ROOT/'failure_evidence.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['receiver_failure','matched_source_timing','receiver_refinement_by_time','source_timing']},indent=2))
print('FAILURE',failure)
print('MATCHED TIMING',json.dumps(windows[0]))
print('REFINEMENT',json.dumps(by_time))
