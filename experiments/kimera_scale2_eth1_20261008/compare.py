#!/usr/bin/env python3
"""Post-run state response, paired metric changes, and preservation record."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from experiment import ROOT, BASE, HERE, protect_check, rows, save, sha

def stats(a):
    return dict(median=float(np.median(a)),rms=float(np.sqrt(np.mean(a*a))),
                p95=float(np.quantile(a,.95)),maximum=float(np.max(a)))

historical=json.loads((BASE/'manifest.json').read_text())['protected']
assert all(sha(path)==expected for path,expected in historical.items())
result={}
for mode in ('off','on'):
    assert json.loads((ROOT/f'{mode}_validation.json').read_text())['passed']
    assert json.loads((ROOT/f'{mode}_determinism.json').read_text())['passed']
    result[mode]={}
    for product,name in [('online','online_states.csv'),('final','states.csv')]:
        old=rows(BASE/(mode+'_wnoa')/name);new=rows(ROOT/(mode+'_x2')/name)
        assert [r['timestamp_ns'] for r in old]==[r['timestamp_ns'] for r in new]
        def matrix(data,fields):return np.array([[float(r[f]) for f in fields] for r in data])
        response={}
        for key,fields in [('position_m',['px','py','pz']),('velocity_m_s',['vx','vy','vz']),
                           ('angular_rate_rad_s',['wx','wy','wz'])]:
            response[key]=stats(np.linalg.norm(matrix(new,fields)-matrix(old,fields),axis=1))
        qa=Rotation.from_quat(matrix(old,['qx','qy','qz','qw']))
        qb=Rotation.from_quat(matrix(new,['qx','qy','qz','qw']))
        response['rotation_deg']=stats(np.rad2deg((qa.inv()*qb).magnitude()))
        response['description']='Same physical supports and fixed initial gauge, x2 minus x1; estimate differences, not truth errors.'
        result[mode][product]=response
metrics=json.loads((ROOT/'metrics.json').read_text())
change={}
for mode in ('off','on'):
    for product in ('online','final'):
        key=mode+'_fused_'+product
        change[key]={}
        for metric in ('ate_m','translation_rpe_m','rotation_rpe_deg','common_grid_body_path_m'):
            before=metrics[key+'_x1'][metric];after=metrics[key+'_x2'][metric]
            change[key][metric]=dict(before=before,after=after,percent_change=100*(after/before-1))
save(ROOT/'comparison.json',dict(state_responses=result,metric_changes=change,
     protected_hashes_verified=protect_check(),historical_hashes_verified=len(historical),
     protocol_sha256=sha(HERE/'PROTOCOL.txt')))
print(json.dumps(change,indent=2))
