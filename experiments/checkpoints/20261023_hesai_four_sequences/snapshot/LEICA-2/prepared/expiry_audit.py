"""Read-only lifecycle/payload accounting; no solver or source invocation."""
from collections import Counter
import csv
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def pose(x):
    x=np.asarray(x,dtype=float);w=x[:3];theta=np.linalg.norm(w)
    W=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])
    if theta<1e-7:a=.5-theta**2/24;b=1/6-theta**2/120
    else:a=(1-math.cos(theta))/theta**2;b=(theta-math.sin(theta))/theta**3
    return Rotation.from_rotvec(w),(np.eye(3)+a*W+b*W@W)@x[3:]


def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(n=len(a),median=float(np.median(a)),rms=float(np.sqrt(np.mean(a*a))),
                p95=float(np.quantile(a,.95)),max=float(np.max(a))) if len(a) else dict(n=0)


class ExpiryAudit:
    def __init__(self):
        self.accepted={};self.absorbed={};self.waiting=None;self.details=[]
        self.versions=Counter();self.ids=set();self.references=0

    def consume(self,r,event):
        if r[0]=='absorbed':
            key=tuple(r[1:5]);assert key in self.accepted and key not in self.absorbed
            accepted=self.accepted[key]
            assert r[5:7]==[accepted[6],accepted[12]],('absorption mismatch',key)
            self.absorbed[key]=(event,int(r[7]))
        if r[0]!='belief':return
        assert len(r)==55,r[:13]
        action=r[1];key=tuple(r[2:6])
        if action=='sealed_reference':
            assert self.waiting is not None and self.waiting[0]==event
            expired=self.waiting[1];assert key==tuple(expired[2:6])
            accepted=self.accepted[key]
            assert r[2:11]==accepted[2:11] and r[12]==accepted[12]
            # Matrix->Pose3->Log terminal serialization can add roundoff only.
            assert np.max(np.abs(np.array(r[13:19],float)-np.array(accepted[13:19],float)))<1e-12
            assert r[19:]==accepted[19:] and r[11]==expired[11]
            assert expired[8:10]==r[8:10] and expired[12]==r[12]
            rot,t=pose(expired[13:19]);rot0,t0=pose(r[13:19])
            c=np.array(expired[19:],float).reshape(6,6);c0=np.array(r[19:],float).reshape(6,6)
            assert np.isfinite(c).all() and np.max(np.abs(c-c.T))<=1e-12
            assert np.linalg.eigvalsh(c).min()>0
            absorbed_event,absorbed_at=self.absorbed[key]
            self.details.append(dict(event=event,action=expired[1],source=key[0],session=key[1],
                from_id=int(key[2]),to_id=int(key[3]),expired_revision=int(expired[6]),
                sealed_revision=int(r[6]),start_ns=int(r[8]),end_ns=int(r[9]),
                arrival_ns=int(expired[10]),absorbed_event=absorbed_event,absorbed_at_ns=absorbed_at,
                endpoint_age_s=(int(expired[10])-int(r[8]))*1e-9,
                since_absorption_s=(int(expired[11])-absorbed_at)*1e-9,
                version_jump=int(expired[6])-int(r[6]),
                translation_change_m=float(np.linalg.norm(t-t0)),
                rotation_change_rad=float((rot0.inv()*rot).magnitude()),
                rotation_trace_ratio=float(np.trace(c[:3,:3])/np.trace(c0[:3,:3])),
                translation_trace_ratio=float(np.trace(c[3:,3:])/np.trace(c0[3:,3:])),
                full_covariance_relative_frobenius=float(np.linalg.norm(c-c0)/np.linalg.norm(c0))))
            self.references+=1;self.waiting=None
            return
        self.versions[key[0]]+=1;self.ids.add(key)
        if action in ('insert','replace'):
            assert key not in self.absorbed
            assert (key in self.accepted)==(action=='replace')
            if action=='replace':assert int(r[6])>int(self.accepted[key][6])
            self.accepted[key]=r
        elif action.startswith('expired_'):
            assert action in ('expired_revision','expired_duplicate','expired_stale')
            assert key in self.absorbed and self.absorbed[key][0]<event
            assert int(r[11])>=self.absorbed[key][1] and self.waiting is None
            self.waiting=(event,r)

    def finish(self,summary):
        assert self.waiting is None
        assert summary['absorbed_revision_policy']=='seal_at_absorption'
        assert summary['expired']==len(self.details)==self.references
        assert summary['inserted']==len(self.accepted)
        assert summary['absorbed']==len(self.absorbed)
        assert summary['received']==sum(self.versions.values())
        actions=Counter(r['action'] for r in self.details)
        assert actions['expired_duplicate']==summary['expired_duplicate']
        assert actions['expired_stale']==summary['expired_stale']
        def cohort(rows):
            return dict(versions=len(rows),ids=len({(r['source'],r['session'],r['from_id'],r['to_id']) for r in rows}),
                changes={k:stats([r[k] for r in rows]) for k in
                    ('endpoint_age_s','since_absorption_s','version_jump','translation_change_m',
                     'rotation_change_rad','rotation_trace_ratio','translation_trace_ratio',
                     'full_covariance_relative_frobenius')})
        return dict(passed=True,policy=summary['absorbed_revision_policy'],expired=cohort(self.details),
            by_source={source:dict(received_versions=n,received_ids=sum(key[0]==source for key in self.ids),
                expired_fraction=sum(r['source']==source for r in self.details)/n,
                **cohort([r for r in self.details if r['source']==source])) for source,n in self.versions.items()},
            reference_rows_not_receipts=self.references,
            caveat='Changes vs last accepted belief, NOT GT errors or proof that omitted corrections are harmless.')


if __name__=='__main__':
    # Check this reporting code on the already-passed K19 archive; no solver replay.
    root=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/replay')
    audit=ExpiryAudit()
    for directory in sorted((root/'archive/commits').iterdir(),key=lambda p:int(p.name)):
        with (directory/'records.txt').open() as f:
            for r in csv.reader(f):audit.consume(r,int(directory.name))
    result=audit.finish(json.loads((root/'summary.json').read_text()))
    assert result['expired']['versions']==19 and result['expired']['ids']==19
    print(json.dumps(result,indent=2))
