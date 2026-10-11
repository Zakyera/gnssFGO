"""Bounded read-only evidence for aborted ON; no estimator or scoring calls."""
import csv
import json
import re
from collections import Counter
from pathlib import Path
ROOT=Path(__file__).resolve().parent
RUN=ROOT/'prepared'
source=RUN/'full160_on_source'
marker='KIMERA_CBS_MEAN_ALIGNMENT_UNAVAILABLE,'
failures=[];retry=[];warnings=0;line_no=0
with (source/'estimators.log').open() as f:
    for line_no,line in enumerate(f,1):
        if 'performing linearization in sync mode seriously affects the processing speed' in line:warnings+=1
        if marker in line:
            v=line.split(marker,1)[1].strip().split(',')
            failures.append(dict(line=line_no,backend_update=int(v[0]),sender_i=int(v[1]),sender_j=int(v[2]),reason=','.join(v[3:])))
        if 'CBS_ODOM_RETRY_ROW_G2K,' in line and 'mean_alignment' in line:
            v=line.split('CBS_ODOM_RETRY_ROW_G2K,',1)[1].strip().split(',')
            retry.append(dict(line=line_no,edge=v[0],from_s=float(v[1]),to_s=float(v[2]),
                              age_s=float(v[3]),retry_age_limit_s=float(v[4]),reason=v[5],decision=v[6]))
assert failures and retry
with (source/'kimera_odometry.csv').open() as f:krows=list(csv.DictReader(f))
kstamps={int(r['field.header.seq']):int(r['field.header.stamp'])*1e-9 for r in krows}
grows=[list(map(float,line.split())) for line in (source/'odom_imu.txt').read_text().splitlines() if line.strip() and not line.startswith('#')]
imu=[]
with (RUN/'full160_on_receiver/events.csv').open() as f:
    for r in csv.reader(f):
        if r[0]=='I':imu.append(int(r[2])*1e-9)
for x in failures:
    x['receiver_keyframe_time_s']=kstamps.get(x['backend_update'])
    x['receiver_elapsed_s']=(x['receiver_keyframe_time_s']-imu[0]) if x['receiver_keyframe_time_s'] else None
summary=json.loads((RUN/'full160_on_receiver/receiver/summary.json').read_text())
with (RUN/'full160_on_receiver/receiver/publication_convergence.csv').open() as f:conv=list(csv.DictReader(f))
assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in conv)
first=failures[0]
out=dict(status='ABORTED_SOURCE_MEAN_ALIGNMENT_AVAILABILITY_FAILURE',
 first_failure=first,first_retry_decision=retry[0],failure_count=len(failures),
 unique_sender_edges=len({(x['sender_i'],x['sender_j']) for x in failures}),
 failures_by_reason=dict(Counter(x['reason'] for x in failures)),
 retry_decisions=dict(Counter(x['decision'] for x in retry)),
 captured_imu_span_s=imu[-1]-imu[0],imu_samples=len(imu),
 native_glim=dict(rows=len(grows),span_s=grows[-1][0]-grows[0][0],final_timestamp_s=grows[-1][0],
                  final_tail_behind_imu_s=imu[-1]-grows[-1][0]),
 native_kimera=dict(rows=len(krows),span_s=max(kstamps.values())-min(kstamps.values()),
                    final_timestamp_s=max(kstamps.values()),final_tail_behind_imu_s=imu[-1]-max(kstamps.values())),
 fuser_partial=dict(support_states=summary['support_states'],pending=summary['pending_at_end'],
                   maximum_gradient=max(float(r['gradient_after']) for r in conv),
                   publications_all_stationary=True,imu_factor_count=summary['imu_factor_count'],
                   model=summary['receiver_model'],revision_versions=summary['revision_messages_loaded']),
 sync_linearization_warning_lines=warnings,estimators_log_bytes=(source/'estimators.log').stat().st_size,
 evidence_limit='Delay/anchor failure established. Cause of GLIM lag (including possible diagnostic overhead) not isolated. No covariance or accuracy interpretation from partial ON.',
 failures=failures,retry_rows=retry)
with (ROOT/'failure_evidence.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in out.items() if k not in ['failures','retry_rows']},indent=2))
