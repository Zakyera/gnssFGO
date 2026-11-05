"""Summarize verified saved-only contrasts; no scoring, capture or optimizer."""
import csv,hashlib,json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
D=K/'analysis_v2/results';S=K/'prepared/on_source'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,v):
    with (H/n).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def rows(n):
    with (D/n).open() as f:return list(csv.DictReader(f))
assert read(K/'analysis_v2/passed.json')['passed']
assert read(K/'analysis_v2/cleanup.json')['owned_stage_jobs_exited']
hist=read(H/'historical_states.json');old=read(E/'eth1_kimera_early_covariance_cpv97_20261103/analysis.json')
summary={(int(r['update']),r['variant']):{k:(float(v) if k=='H_trace' else int(v)) for k,v in r.items() if k not in ('update','variant')} for r in rows('summary.csv')}
smart={(int(r['update']),r['variant'],int(r['slot'])):r for r in rows('smart.csv')}
cov={(int(r['update']),r['variant']):np.array([float(r[f'c{i}']) for i in range(36)]).reshape(6,6) for r in rows('covariance.csv')}
parity=[];cache=[];contrasts=[]
names=['VALID','DEGENERATE','BEHIND_CAMERA','OUTLIER','FAR_POINT']
for k in range(3,26):
    factual=cov[k,'refreshed_savedcache'];logged=np.array(hist['sources']['capture']['first_covariance'][str(k)]).reshape(6,6)
    err=float(np.linalg.norm(factual-logged)/np.linalg.norm(logged));assert err<1e-9
    parity.append(dict(update=k,full_matrix_relative_error=err))
    fresh=cov[k,'refreshed_freshcache']
    cache.append(dict(update=k,relative_matrix_difference=float(np.linalg.norm(fresh-factual)/np.linalg.norm(factual)),
        translation_trace_ratio=float(np.trace(fresh[3:,3:])/np.trace(factual[3:,3:]))))
    a={slot:r for (u,v,slot),r in smart.items() if u==k and v=='history91_freshcache'}
    b={slot:r for (u,v,slot),r in smart.items() if u==k and v=='history95_freshcache'}
    assert set(a)==set(b)
    def transitions(touch=False):
        return dict(Counter(names[int(a[s]['status'])]+' -> '+names[int(b[s]['status'])] for s in a
            if a[s]['status']!=b[s]['status'] and (not touch or a[s]['touches_endpoint']=='1')))
    contrast=dict(update=k,sequence_time_s=float(hist['sources']['capture']['to_stamps'][str(k)]-1727775007.855396309),
        fixed_measurement_set_smart_factors=len(a),old_pose_information=summary[k,'history91_freshcache'],new_pose_information=summary[k,'history95_freshcache'],
        global_status_changes=transitions(),endpoint_status_changes=transitions(True),
        visual_H_trace_ratio=float(summary[k,'history95_freshcache']['H_trace']/summary[k,'history91_freshcache']['H_trace']),
        original_native_translation_traces={lab:float(np.trace(np.array(hist['sources'][lab]['first_covariance'][str(k)]).reshape(6,6)[3:,3:])) for lab in ('old','new','capture')},
        within_capture_fixed_factor_translation_traces={v:float(np.trace(cov[k,v][3:,3:])) for v in ['initial_freshcache','ordinary_freshcache','refreshed_freshcache','refreshed_savedcache']})
    contrasts.append(contrast)
assert max(x['relative_matrix_difference'] for x in cache)<.08
br=rows('bias.csv');vector=lambda r:np.array([float(r[c]) for c in ['ax','ay','az','gx','gy','gz']])
latest={int(r['update']):vector(r) for r in br if r['kind']=='refreshed' and r['index']==r['update']}
pims={int(r['update']):vector(r) for r in br if r['kind']=='pim_biasHat' and int(r['index'])+1==int(r['update'])}
bias_matches=[]
for k in range(5,26):
    assert np.array_equal(pims[k],latest[k-2])
    bias_matches.append(dict(newest_factor_update=k,integrated_interval=[k-1,k],bias_exactly_matches_MAIN_update=k-2))
a=np.loadtxt(K/'historical_poses/old_23.txt');b=np.loadtxt(K/'historical_poses/new_23.txt')
q=lambda x:np.column_stack([x[:,5:8],x[:,4]])
td=np.linalg.norm(a[:,1:4]-b[:,1:4],axis=1);rd=(Rotation.from_quat(q(a)).inv()*Rotation.from_quat(q(b))).magnitude()*180/np.pi
event=next(x for x in contrasts if x['update']==23)
assert event['old_pose_information']['touch_nonzero']==16 and event['new_pose_information']['touch_nonzero']==66
assert event['endpoint_status_changes']['OUTLIER -> VALID']==52
limits=[
 'One fresh12s diagnostic, not a matched historical repeat or full-sequence result; observer added6.96s aggregate wall-time and can alter scheduling.',
 'The fixed measurement set belongs to CP98. CP91/CP95 actual visual observations remain unavailable; cross-run state sensitivity is proven on this set, not a complete decomposition of their historic difference.',
 'Historical variants replace poses only and test smart factors only. No claim to reproduce historical velocity/bias/IMU covariance or full nonlinear stationarity.',
 'Different counts of rejected/accepted factors can change confidence legitimately. This does not certify covariance calibration, statistical independence or prove the whole-run GLIM ATE cause.',
 'PIM biasHat proves the bias used by preintegration, not every frontend callback or RANSAC use. The observed two-update feedback timing is not labelled a bug.',
 'No tuning or scientific build changes. Offline fixture compile type mismatch was corrected in a new preserved analysis_v2; capture was not repeated.']
result=dict(passed=True,capture=read(H/'snapshot_validation.json'),coverage=read(S/'source_coverage.json'),
    covariance_reproduction=dict(updates=23,max_full_matrix_relative_error=max(x['full_matrix_relative_error'] for x in parity),rows=parity),
    cache_reset_sensitivity=cache,fixed_measurement_state_contrasts=contrasts,
    historical_pose_differences_event23=dict(median_translation_m=float(np.median(td)),max_translation_m=float(td.max()),median_rotation_deg=float(np.median(rd)),max_rotation_deg=float(rd.max())),
    bias_feedback=dict(exact_matches=bias_matches,interpretation='PIM biasHat matches the previously delivered MAIN estimate at k-2 on all21 checked intervals; no frontend per-call trace.'),
    gate=dict(profile_outlier_threshold_pixels=3.0,implementation='maximum individual mono reprojection norm, not average despite comments',invalid_hessian='ZERO_ON_DEGENERACY',landmark_distance_m=10),
    scope=dict(source_captures=1,source_binaries_changed=False,optimizer_runs=0,accuracy_scores_computed=0,fusion_runs=0),limitations=limits)
save('analysis.json',result)
pins=read(K/'prepared/protected_manifest.json');pins.update(read(H/'capture_complete.json')['output_sha256']);pins.update(read(K/'analysis_v2/inputs.json'))
for root in [K/'analysis',K/'analysis_v2',K/'historical_poses']:
    for p in root.rglob('*'):
        if p.is_file():pins[str(p)]=sha(p)
for p in [Path(__file__),W/'devel/include/gtsam/geometry/triangulation.h',W/'devel/include/gtsam_unstable/slam/SmartStereoProjectionFactor.h',
 W/'runs/persistent_compatibility_cpv76_20261101/VioBackend.cpp',H/'PRELAUNCH_REVIEW.txt']:
    pins[str(p)]=sha(p)
for p,digest in pins.items():assert sha(p)==digest,p
save('analysis_inputs.json',pins)
save('verification.json',dict(passed=True,protected_inputs=len(pins),source_capture_count=1,source_settings_unchanged=True,all_snapshots_unchanged=True,
    historical_CP91_CP95_terminal_outputs_preserved=True,output_sha256={str(H/n):sha(H/n) for n in ('analysis.json','analysis_inputs.json')}))
print(json.dumps(dict(event=event,max_covariance_reproduction_error=result['covariance_reproduction']['max_full_matrix_relative_error'],max_cache_difference=max(x['relative_matrix_difference'] for x in cache),bias_exact_matches=len(bias_matches)),indent=2))
