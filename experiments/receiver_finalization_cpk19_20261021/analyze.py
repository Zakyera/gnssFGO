"""Compare frozen K18 committed prefix and audit explicitly forgone revisions."""
import csv
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from driver import H, R, P, sha, save, verify

def pose(x):
    x=np.asarray(x,dtype=float);w=x[:3];theta=np.linalg.norm(w)
    W=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
    if theta<1e-7:
        a=.5-theta**2/24; b=1/6-theta**2/120
    else:a=(1-math.cos(theta))/theta**2;b=(theta-math.sin(theta))/theta**3
    return Rotation.from_rotvec(w), (np.eye(3)+a*W+b*W@W)@x[3:]

def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(n=len(a),median=float(np.median(a)),rms=float(np.sqrt(np.mean(a*a))),
                p95=float(np.quantile(a,.95)),max=float(np.max(a))) if len(a) else dict(n=0)

def main():
    assert json.loads((R/'replay.json').read_text())['returncode']==0
    out=R/'replay';old=P/'full_off_receiver/receiver'
    same=0;maxseq=0
    for p in (old/'archive/commits').glob('*/records.txt'):
        seq=int(p.parent.name);maxseq=max(maxseq,seq)
        q=out/'archive/commits'/str(seq)/'records.txt'
        assert p.read_bytes()==q.read_bytes(),('committed prefix changed',seq)
        same+=1
    for name in ('online_states.csv','online_trajectory.tum','finalized_states.csv','finalized_trajectory.tum'):
        # Finalized candidate includes a diagnostic EOF tail, not new live data.
        assert (out/name).read_bytes().startswith((old/name).read_bytes()),name
    rows=[]; actions={}; bysource={}
    for p in (out/'archive/commits').glob('*/records.txt'):
        seq=int(p.parent.name)
        if seq<=maxseq:continue
        rr=list(csv.reader(p.open()))
        for b in rr:
            if not b or b[0]!='belief':continue
            actions[b[1]]=actions.get(b[1],0)+1
            if not b[1].startswith('expired_'):continue
            ref=next(x for x in rr if x[:2]==['belief','sealed_reference'] and x[2:6]==b[2:6])
            m=np.asarray(b[13:19],dtype=float);m0=np.asarray(ref[13:19],dtype=float)
            r,t=pose(m);r0,t0=pose(m0)
            cov=np.asarray(b[19:55],dtype=float).reshape(6,6)
            cov0=np.asarray(ref[19:55],dtype=float).reshape(6,6)
            item=dict(event=seq,action=b[1],source=b[2],session=b[3],from_id=int(b[4]),to_id=int(b[5]),
                expired_revision=int(b[6]),sealed_revision=int(ref[6]),start_ns=int(b[8]),end_ns=int(b[9]),
                arrival_ns=int(b[10]),endpoint_age_s=(int(b[10])-int(b[8]))*1e-9,
                version_jump=int(b[6])-int(ref[6]),
                rotation_change_rad=float((r0.inv()*r).magnitude()),translation_change_m=float(np.linalg.norm(t-t0)),
                rotation_trace_ratio=float(np.trace(cov[:3,:3])/np.trace(cov0[:3,:3])),
                translation_trace_ratio=float(np.trace(cov[3:,3:])/np.trace(cov0[3:,3:])),
                full_covariance_relative_frobenius=float(np.linalg.norm(cov-cov0)/np.linalg.norm(cov0)))
            rows.append(item);bysource[item['source']]=bysource.get(item['source'],0)+1
    rows.sort(key=lambda x:x['event'])
    summary=json.loads((out/'summary.json').read_text())
    assert summary['expired']==len(rows)>0
    assert rows[0]['event']==214527 and maxseq==214526
    assert summary['received']==sum(summary[k] for k in ('inserted','replaced','excluded','unsupported_end','duplicate','stale','expired'))
    assert summary['inserted']==summary['active']+summary['absorbed']
    assert summary['pending_at_end']==0
    peakSupports=peakSlots=0; seconds=[]
    with (out/'stream_progress.csv').open() as f:
        for b in csv.DictReader(f):
            peakSupports=max(peakSupports,int(b['supports']));peakSlots=max(peakSlots,int(b['slots']))
            if b['kind']=='I' and float(b['service_seconds'])>.001:seconds.append(float(b['service_seconds']))
    assert peakSupports<=71
    source_counts={};source_ids={};original_events=0
    with (P/'full_off_receiver/events.csv').open() as f:
        for line in f:
            original_events+=1
            b=line.split(',')
            if b[0]=='B':
                source_counts[b[2]]=source_counts.get(b[2],0)+1
                source_ids.setdefault(b[2],set()).add(tuple(b[2:4]+b[6:8]))
    result=dict(passed=True,scope='saved FAILED-prefix replay only, synthetic EOF; no fresh sources or full sequence',
        original_events=original_events,original_last_committed_event=maxseq,
        original_committed_records_byte_identical=same,original_online_and_finalized_prefix_bytes_equal=True,
        expired_by_source=bysource,source_versions=source_counts,source_ids={k:len(v) for k,v in source_ids.items()},
        expired_ids=len(set((b['source'],b['session'],b['from_id'],b['to_id']) for b in rows)),
        expired_mean_covariance_changes_vs_last_accepted={k:stats([b[k] for b in rows]) for k in
          ('endpoint_age_s','version_jump','translation_change_m','rotation_change_rad','rotation_trace_ratio',
           'translation_trace_ratio','full_covariance_relative_frobenius')},
        post_failure_actions=actions,receiver_summary=summary,peak_retained_supports=peakSupports,
        peak_slots=peakSlots,protected_unchanged=verify(),
        receiver_sha256=sha(R/'live'),replay_seconds=json.loads((R/'replay.json').read_text())['seconds'],
        caveat='Only 20 retained events from the failure onward; late-version burden beyond capture remains unknown.')
    save(R/'expired_details.json',rows);save(R/'validation.json',result)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
