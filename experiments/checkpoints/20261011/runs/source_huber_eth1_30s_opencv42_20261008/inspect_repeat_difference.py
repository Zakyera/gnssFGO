"""Read-only failure characterization, not a relaxed validation or new replay."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
live=ROOT/'on_huber_receiver/receiver'
repeat=ROOT/'on_huber_repeat'
files=['trajectory.tum','states.csv','online_trajectory.tum','online_states.csv','support_clock.csv',
       'source_model_covariance.csv','model_covariance.csv','causal_dispatch.csv','measurement_events.csv',
       'publication_convergence.csv','revision_events.csv','source_accounting.csv','unsupported_beliefs.csv']
result=dict(status='VALIDATION_BLOCKED',failed_gate='exact deterministic journal replay',
            new_experiments_run=False,scoring_run=False,files={})
for name in files:
    a=(live/name).read_bytes(); b=(repeat/name).read_bytes()
    result['files'][name]=dict(byte_identical=a==b,live_sha256=hashlib.sha256(a).hexdigest(),
        repeat_sha256=hashlib.sha256(b).hexdigest())
for name in ['trajectory.tum','online_trajectory.tum']:
    a=np.loadtxt(live/name); b=np.loadtxt(repeat/name)
    assert a.shape==b.shape and np.array_equal(a[:,0],b[:,0])
    distance=np.linalg.norm(a[:,1:4]-b[:,1:4],axis=1)
    angle=(Rotation.from_quat(a[:,4:8]).inv()*Rotation.from_quat(b[:,4:8])).magnitude()
    diff=np.flatnonzero(np.any(a!=b,axis=1))
    result['files'][name].update(rows=len(a),max_position_m=float(distance.max()),
        rms_position_m=float(np.sqrt(np.mean(distance**2))),max_rotation_rad=float(angle.max()),
        first_numerically_different_row=int(diff[0]) if len(diff) else None,
        max_quaternion_element_abs=float(np.abs(a[:,4:8]-b[:,4:8]).max()))
for name in ['states.csv','online_states.csv']:
    with (live/name).open() as f: a=list(csv.DictReader(f))
    with (repeat/name).open() as f: b=list(csv.DictReader(f))
    assert len(a)==len(b)
    assert all(x['state_index']==y['state_index'] and x['timestamp_ns']==y['timestamp_ns'] for x,y in zip(a,b))
    fields=[k for k in a[0] if k not in ['state_index','timestamp_ns']]
    result['files'][name]['max_abs_by_field']={k:max(abs(float(x[k])-float(y[k])) for x,y in zip(a,b)) for k in fields}
for name in ['support_clock.csv','causal_dispatch.csv','measurement_events.csv','revision_events.csv']:
    if not result['files'][name]['byte_identical']:
        a=(live/name).read_text().splitlines(); b=(repeat/name).read_text().splitlines()
        first=next((i for i,(x,y) in enumerate(zip(a,b)) if x!=y),min(len(a),len(b)))
        result['files'][name]['first_different_line']=first+1
        result['files'][name]['first_difference']=[a[first] if first<len(a) else None,b[first] if first<len(b) else None]
for name,folder in [('live',live),('repeat',repeat)]:
    with (folder/'publication_convergence.csv').open() as f: rows=list(csv.DictReader(f))
    result[name+'_stationarity']=dict(rows=len(rows),statuses=sorted(set(r['status'] for r in rows)),
        maximum_gradient=max(float(r['gradient_after']) for r in rows))
with (ROOT/'repeat_difference.json').open('x') as f: json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
