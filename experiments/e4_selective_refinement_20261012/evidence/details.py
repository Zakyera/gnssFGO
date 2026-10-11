#!/usr/bin/env python3
import csv,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from analyze import ROOT,CAPTURE
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
a,b=rows(ROOT/'reference/online_states.csv'),rows(ROOT/'candidate/online_states.csv')
q=lambda rr:np.array([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in rr])
d=(Rotation.from_quat(q(a[:len(b)])).inv()*Rotation.from_quat(q(b))).magnitude()
i=int(np.flatnonzero(d>1e-8)[0])
with (CAPTURE/'live160_on_receiver/events.csv').open() as f:
    begin=next(int(line.split(',')[2]) for line in f if line.startswith('I,'))
st=rows(ROOT/'candidate/publication_steps.csv')
rejected=[s for s in st if s['iteration']=='1' and s['accepted']=='0' and s['recovery']=='0']
result=dict(first_orientation_failure=dict(state_index=b[i]['state_index'],timestamp_ns=b[i]['timestamp_ns'],bag_seconds=(int(b[i]['timestamp_ns'])-begin)*1e-9,difference_rad=float(d[i]),limit_rad=1e-8),
    rejected_selective_trials=len(rejected),rejected_nonstationary=sum(float(s['gradient_after'])>1e-5 for s in rejected),
    rejected_objective_increased=sum(float(s['error_after'])>float(s['error_before']) for s in rejected),
    rejected_gradient_median_p95_max=list(map(float,np.quantile([float(s['gradient_after']) for s in rejected],[.5,.95,1]))),
    stop_reason='Live prefix check observed position gate failure; retrospective analysis identifies earlier orientation gate failure. Candidate intentionally SIGTERM, not optimizer crash.')
with (ROOT/'partial_details.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
