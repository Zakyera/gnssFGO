"""Analyze the single frozen direction probe, without rerunning an estimator."""
from pathlib import Path
import json,csv
import numpy as np
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_direction_fl4p_20261016')
D=R/'remove_capture';P=D/'direction/update_89'
assert json.loads((D/'validation.json').read_text())['valid']
with (P/'factors.csv').open() as f:rows=list(csv.DictReader(f))
with (P/'cost_curve.csv').open() as f:curve=list(csv.DictReader(f))
families={};totals={};switches={}
for reset in ['0','1']:
    totals[reset]={}
    for r in curve:
        if r['reset']!=reset:continue
        a=float(r['alpha']);totals[reset][a]=totals[reset].get(a,0)+float(r['cost'])
    families[reset]={}
    for name in sorted({r['family'] for r in rows}):
        selected=[r for r in rows if r['reset']==reset and r['family']==name]
        families[reset][name]={'count':len(selected),**{k:sum(float(r[k]) for r in selected) for k in ['cost0','cost1','cached_slope','current_slope','fd_1e5','fd_1e6']}}
    status0={r['slot']:r['status0'] for r in rows if r['reset']==reset and r['family']=='visual_smart'}
    switches[reset]={str(a):sum(r['family']=='visual_smart' and r['reset']==reset and float(r['alpha'])==a and r['status']!=status0[r['slot']] for r in curve) for a in totals[reset]}
H=np.loadtxt(P/'cache_H.txt',delimiter=',');g=np.loadtxt(P/'cache_g.txt',delimiter=',')
d0=np.loadtxt(P/'d0.txt',delimiter=',');d1=np.loadtxt(P/'d1.txt',delimiter=',');s=d1-d0
currentH=np.loadtxt(P/'current_H.txt',delimiter=',');currentg=np.loadtxt(P/'current_g.txt',delimiter=',')
tangent=np.loadtxt(P/'path_tangent_at_before.txt',delimiter=',')
stats=dict(cached_slope=float((H@d0-g)@s),cached_curvature=float(s@H@s),current_gradient_slope=float(-currentg@tangent),current_GN_curvature=float(tangent@currentH@tangent),
    cached_predicted_cost_change=float((H@d0-g)@s+.5*s@H@s),direction_norm=float(np.linalg.norm(s)))
fd={}
for reset,c in totals.items():
    fd[reset]={str(h):{'central_slope':(c[h]-c[-h])/(2*h),'forward_slope':(c[h]-c[0])/h,'backward_slope':(c[0]-c[-h])/h,'curvature':(c[h]+c[-h]-2*c[0])/(h*h)} for h in [1e-2,1e-3,1e-4,1e-5,1e-6,1e-7,1e-8]}
fixed=[];off=0
for line in (P/'keys.txt').read_text().splitlines():
    key,dim,f=line.split();dim=int(dim)
    if key.startswith('x'):fixed.append(dict(key=key,fixed=bool(int(f)),anchor_rotation_rad=float(np.linalg.norm(d0[off:off+3])),anchor_translation_log_m=float(np.linalg.norm(d0[off+3:off+6]))))
    off+=dim
report=dict(update=89,families=families,cost_curve=totals,visual_status_switches=switches,finite_differences=fd,
    linear_models=stats,pose_anchor_offsets=fixed,cache_reset_max_cost_difference=max(abs(totals['0'][a]-totals['1'][a]) for a in totals['0']))
report['dominant_imu_model_mismatches']=sorted(
    [r for r in rows if r['reset']=='0' and r['family']=='native_imu'],
    key=lambda r:abs(float(r['cached_slope'])-float(r['fd_1e5'])),reverse=True)[:6]
report['anchor_summary']=dict(pose_count=len(fixed),fixed_pose_count=sum(r['fixed'] for r in fixed),
    max_rotation_deg=max(r['anchor_rotation_rad'] for r in fixed)*180/np.pi,
    max_translation_log_m=max(r['anchor_translation_log_m'] for r in fixed))
report['visual_status_counts_at_start']={reset:{status:sum(r['reset']==reset and r['family']=='visual_smart' and r['status0']==status for r in rows) for status in sorted({r['status0'] for r in rows if r['family']=='visual_smart'})} for reset in ['0','1']}
report['cache_reset_differences']={str(a):totals['1'][a]-totals['0'][a] for a in totals['0']}
(R/'analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(families=families['0'],linear_models=stats,finite_differences=fd['0'],
    cost_curve=totals['0'],visual_status_switches=switches,cache_reset_max_cost_difference=report['cache_reset_max_cost_difference'],
    anchor_summary=report['anchor_summary'],visual_status_counts_at_start=report['visual_status_counts_at_start'],
    dominant_imu_model_mismatches=report['dominant_imu_model_mismatches'],cache_reset_differences=report['cache_reset_differences']),indent=2))
