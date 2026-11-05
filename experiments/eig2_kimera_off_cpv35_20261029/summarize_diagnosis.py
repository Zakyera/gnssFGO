"""Summarize saved targeted snapshots; no optimization or ATE/RPE."""
import csv,json
import numpy as np
from analyze_snapshots import H,P,states,local
snap=json.loads((H/'snapshots_final.json').read_text())['updates']
events={e['k']:e for e in json.loads((H.parent/'eig2_kimera_jitter_cpv34_20261029/velodyne_eig2_off_events.json').read_text())}
selected=[]
for r in snap:
    k=r['update']
    if k not in [800,1000,1335,1336,1337,1338,1410,1467]:continue
    s=r['stages']['after_solve'];d=P/'capture/snapshots'/('update_'+str(k))/'after_solve'
    x=states(d/'state.csv');t=states(d/'theta.csv');fixed=[key for key in s['fixed_keys'] if key[0]=='x']
    imu=list(csv.DictReader((d/'imu_residuals.csv').open()))
    def maximum(values):
        value,key=max(values);return dict(value=float(value),key=key)
    selected.append(dict(update=k,time_s=events[k]['time_s'],event=events[k],
        after_solve=s,before_update=r['stages']['before_update'],
        max_fixed_anchor_rotation_deg=maximum((np.linalg.norm(local(key,t[key],x[key])[:3])*180/np.pi,key) for key in fixed),
        max_fixed_anchor_translation_m=maximum((np.linalg.norm(x[key][:3]-t[key][:3]),key) for key in fixed),
        imu_dt_s=[min(float(v['dt']) for v in imu),max(float(v['dt']) for v in imu)],
        solve_pose_revision_m=r['solve_pose_revision_m'],
        marginalization_max_state_component_change=r['marginalization_max_state_component_change']))
out=dict(selected=selected,snapshot_updates=len(snap),
    all_snapshot_marginalization_max_state_component_change=max(r['marginalization_max_state_component_change'] for r in snap),
    interpretation='Internal factor residuals and solver-state diagnostics, not ground-truth accuracy, nonlinear stationarity, or a proven fix.')
with (H/'diagnosis_summary.json').open('x') as f:json.dump(out,f,indent=2)
print('Saved diagnosis_summary.json:',len(selected),'selected updates')
