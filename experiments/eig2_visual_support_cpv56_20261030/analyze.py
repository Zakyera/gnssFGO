"""Summarize immutable per-factor readouts; no optimizer or evaluation scoring."""
import collections
import csv
import hashlib
import json
from pathlib import Path
import statistics

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'attempt_build2/results'
rows=[json.loads(line) for line in (P/'factors.jsonl').open()]
groups=collections.defaultdict(dict)
for row in rows:
    assert row['slot'] not in groups[row['label']]
    groups[row['label']][row['slot']]=row
names=['valid','degenerate','behind_camera','outlier','far_point']
counts={}
for label,group in groups.items():
    counts[label]={names[i]:sum(r['warm_status']==i for r in group.values()) for i in range(5)}
    counts[label]['nonzero_hessian']=sum(r['warm_hnorm']>0 for r in group.values())
    counts[label]['zero_hessian']=sum(r['warm_hnorm']==0 for r in group.values())
    counts[label]['visual_cost']=sum(r['warm_cost'] for r in group.values())
    counts[label]['saved_cache_status']={names[i]:sum(r['saved_status']==i for r in group.values()) for i in range(5)}
    assert all(r['warm_hnorm']==0 for r in group.values() if r['warm_status']!=0)

def compare(before,after):
    a,b=groups[before],groups[after];assert a.keys()==b.keys()
    transitions=collections.Counter((r['warm_status'],b[i]['warm_status']) for i,r in a.items())
    return {f'{names[i]}->{names[j]}':n for (i,j),n in sorted(transitions.items())}

initial=groups['on_ordinary_theta'];final=groups['on_refresh_estimate']
lost=[]
for slot,a in initial.items():
    b=final[slot]
    if a['warm_status']==0 and b['warm_status']==3:
        ordinary=groups['on_ordinary_estimate'][slot]
        fraction=next((j/16 for j in range(17) if groups[f'on_segment0_fraction{j}'][slot]['cold_status']==3),None)
        worst=b['worst_observation_index']
        lost.append(dict(slot=slot,keys=a['keys'],initial_px=a['max_reprojection_px'],ordinary_px=ordinary['max_reprojection_px'],final_px=b['max_reprojection_px'],threshold_px=b['outlier_threshold_px'],first_sampled_outlier_fraction_ordinary=fraction,worst_key=b['keys'][worst] if worst>=0 else None,worst_is_right=b['worst_is_right']))
        assert a['max_reprojection_px']<=a['outlier_threshold_px'] and b['max_reprojection_px']>b['outlier_threshold_px']

# Previously evaluated CP52 no-factor arm is evidence only, not rerun here.
B=W/'runs/eig2_first_factor_cpv52_20261030/attempt_compile1/results/B_refresh_factors.csv'
brows=[r for r in csv.DictReader(B.open()) if r['type']=='smart']
bcounts={name:sum(int(r['status'])==i for r in brows) for i,name in enumerate(names)}
off_initial=groups['off_ordinary_theta']
assert off_initial.keys()==initial.keys()
initial_counts_equal=all(off_initial[i]['warm_status']==r['warm_status'] and off_initial[i]['keys']==r['keys'] for i,r in initial.items())
ordinary_to_refresh=compare('on_ordinary_theta','on_refresh_theta')
summary=json.loads((P/'summary.json').read_text())
assert summary['passed']
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
result=dict(passed=True,summary=summary,first_admission_capture_seconds=1.599933183,
    initial_OFF_ON_visual_status_and_keys_equal=initial_counts_equal,
    endpoint_counts={k:v for k,v in counts.items() if 'fraction' not in k},
    path_counts={k:v for k,v in counts.items() if 'fraction' in k},
    transitions={
      'on_initial_to_next_refresh_linearization':ordinary_to_refresh,
      'on_initial_to_final_estimate':compare('on_ordinary_theta','on_refresh_estimate'),
      'off_initial_to_final_estimate':compare('off_ordinary_theta','off_refresh_estimate')},
    initially_valid_final_outliers=lost,
    lost_track_worst_key_counts=dict(collections.Counter(r['worst_key'] for r in lost)),
    lost_track_initial_reprojection_range_px=[min((r['initial_px'] for r in lost),default=0),max((r['initial_px'] for r in lost),default=0)],
    lost_track_final_reprojection_range_px=[min((r['final_px'] for r in lost),default=0),max((r['final_px'] for r in lost),default=0)],
    CP52_saved_no_first_factor_counts=bcounts,
    cached_equation_parity=[json.loads(l) for l in (P/'parity.jsonl').open()],
    parameters={k:sorted(set(r[k] for r in rows)) for k in ['outlier_threshold_px','distance_threshold_m','rank_tolerance','retriangulation_threshold','enableEPI','useLOST']},
    inputs={str(p):sha(p) for p in [P/'factors.jsonl',P/'parity.jsonl',P/'summary.json',B]},
    limitations=['Reconstructed native Gaussian support, not an original per-factor active-row telemetry recording.','Same native triangulator with separately assembled cameras and gate calculations, not independent physical triangulation ground truth.','Fractional state probes are diagnostic samples, not actual intermediate optimizer iterates.','No source replay, graph optimization, altered factor/noise/guard, covariance weighting study, trajectory score or production fix.'])
with (H/'analysis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:result[k] for k in ['summary','endpoint_counts','transitions','lost_track_worst_key_counts','lost_track_initial_reprojection_range_px','lost_track_final_reprojection_range_px','parameters','cached_equation_parity']},indent=2))
