"""Read the failed capture once; no estimator, optimization or retry."""
from pathlib import Path
import collections,csv,hashlib,json,statistics
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared'
S=P/'live60_off_source';R=P/'live60_off_receiver';O=R/'receiver'
BASE=W/'runs/gnssfgo_eig2_live60_off_on_fl4x_20261017/prepared/live60_off_source'
assert 'revision after absorption' in (R/'receiver.log').read_text()
assert (P/'live60_off_control/failed.txt').exists()
assert not (P/'live60_on_control').exists()
def dicts(path):
 with path.open() as f:return list(csv.DictReader(f))
def stat(v):
 a=sorted(v);assert a
 return dict(n=len(a),median=statistics.median(a),mean=statistics.fmean(a),p95=a[min(len(a)-1,int(.95*len(a)))],maximum=max(a))
progress=dicts(O/'stream_progress.csv');last=int(progress[-1]['event'])
with (R/'events.csv').open() as f:events=list(csv.reader(f))
bad=events[last];assert bad[0]=='B'
identity=bad[2:4]+bad[6:8]
entry=O/'archive/ids'/bad[2]/bad[3]/(bad[6]+'_'+bad[7])
absorbed_commit=int(entry.read_text())
with (O/f'archive/commits/{absorbed_commit}/records.txt').open() as f:
 absorption=next(r for r in csv.reader(f) if r[0]=='absorbed' and r[1:5]==identity)
assert last==7119 and absorbed_commit==7118
receipts=[json.loads(x) for x in (R/'receipts.jsonl').read_text().splitlines()]
receipt=receipts[last];assert receipt['event']==last
firstimu=int(next(e[2] for e in events if e[0]=='I'))
source_update=int(receipt['metadata']['source_update'])
# The preceding successful first belief identifies the producing source update's endpoint.
current=events[last-1];assert int(current[7])==source_update and current[5]=='0'
profile={}
for label,source in [('current',S),('frozen',BASE)]:
 rows=[json.loads(x) for x in (source/'e1b_profile.jsonl').read_text().splitlines()]
 profile[label]={r['timestamp']:r['owner_wall']['root'][1]*1e-9 for r in rows if r['context']==4 and not r['aborted']}
common=sorted(set(profile['current'])&set(profile['frozen']))
steady=[t for t in common if t>=common[0]+5]
assert steady
online=dicts(O/'online_states.csv')
source_counts={}
for name,filename in [('native_glim','glim_odometry.csv'),('native_kimera','kimera_odometry.csv')]:
 a=dicts(S/filename);ts=[int(x['field.header.stamp']) for x in a]
 source_counts[name]=dict(rows=len(a),span_s=(ts[-1]-ts[0])*1e-9)
failure=dict(event_one_based=last+1,last_committed_event=last,source=bad[2],session=bad[3],
 source_edge=[int(bad[6]),int(bad[7])],revision=int(bad[5]),source_update=source_update,
 arrival_ns=int(bad[1]),start_ns=int(bad[8]),end_ns=int(bad[9]),
 since_first_imu_s=(int(bad[1])-firstimu)*1e-9,
 age_from_start_s=(int(bad[1])-int(bad[8]))*1e-9,
 age_from_end_s=(int(bad[1])-int(bad[9]))*1e-9,
 source_update_endpoint_delay_s=(int(bad[1])-int(current[9]))*1e-9,
 within_source_history_s=(int(current[9])-int(bad[8]))*1e-9,
 absorbed_commit=absorbed_commit,absorbed_revision=int(absorption[5]),
 absorbed_arrival_ns=int(absorption[-1]),after_absorption_s=(int(bad[1])-int(absorption[-1]))*1e-9)
service=stat([float(r['service_seconds']) for r in progress])
result=dict(status='FAILED_CAPTURE_NOT_A_60_SECOND_RESULT',failure=failure,
 capture_counts=source_counts,online_rows=len(online),
 last_progress=progress[-1],last_transport_status=json.loads((R/'transport_status.json').read_text()),
 event_service_seconds=service,
 revision_profile_same_stamps=dict(common_rows=len(common),steady_rows=len(steady),
 current_seconds=stat([profile['current'][t] for t in steady]),
 frozen_seconds=stat([profile['frozen'][t] for t in steady]),
 paired_ratio=stat([profile['current'][t]/profile['frozen'][t] for t in steady])),
 no_on_or_other_sequence_launched=True,no_performance_scores_computed=True)
protected=json.loads((P/'protected_manifest.json').read_text())
for p,expected in protected.items():
 with Path(p).open('rb') as f:actual=hashlib.file_digest(f,'sha256').hexdigest()
 assert actual==expected,p
result['protected_unchanged']=len(protected)
with (H/'eig2_off_failure.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
