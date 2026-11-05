"""Write one final evidence report, without re-running capture or scoring."""
import json
from pathlib import Path
H=Path(__file__).resolve().parent
d=json.loads((H/'results.json').read_text());assert d['passed']
lines=[]
def add(s=''):lines.append(s)
def change(a,b):return 100*(a-b)/a
add('CP-V39 — FULL EIG-2 KIMERA-ONLY CBS-OFF REFRESH EXPERIMENT')
add('Original355.02s usable window, start5.7s,0.20x,25s drain. One capture, no retry.')
add('Analysis passed. Final independent integrity/cleanup receipt: closeout.json.')
add()
add('SCOPE / CANDIDATE')
add('Private CP-V38 opt-in: ONE additional current-state native GN refresh before')
add('each marginalization. Immutable original prior function/noise and native smart')
add('factors/IMU weights retained. Not a stronger prior and not a converged solve.')
add('CP-K9=1/FL4Q0/remove/25states; native IMU override unset. No GLIM/fusion,')
add('incoming CBS or CBS-ON. All source defaults and original runs remain unchanged.')
add('Original OFF Kimera traces are identical in Hesai and Velodyne runs. This is')
add('one camera+CPT7 experiment, not two independent LiDAR experiments or a new pair.')
add()
add('DIRECT EVO1.36.3 — ORIGINAL VELODYNE EIG-2 COMMON MASK')
add('Method | ATE baseline -> candidate (m) | ATE improvement | path baseline -> candidate (m) | 0.5m RPE translation (m) | 0.5m RPE rotation (deg)')
for method,label in [('method1','1: newest absolute backend'),('method2','2: native causal incremental')]:
    b=d['common']['baseline_'+method];c=d['common']['candidate_'+method]
    rt=lambda x,k:x['RPE'][k]['statistics']['rmse']
    add(f"{label} | {b['ATE_RMSE_m']:.6f} -> {c['ATE_RMSE_m']:.6f} | {change(b['ATE_RMSE_m'],c['ATE_RMSE_m']):+.2f}% | {b['path_length_m']:.3f} -> {c['path_length_m']:.3f} | {rt(b,'translation_m'):.6f} -> {rt(c,'translation_m'):.6f} | {rt(b,'rotation_deg'):.6f} -> {rt(c,'rotation_deg'):.6f}")
cov=d['common_coverage']
add(f"Coverage: {cov['valid_queries']} common100ms queries; {cov['supported_s']:.1f}s /355.02s, {cov['supported_capture_percent']:.2f}%. Reference path {d['reference_common_length_m']:.3f}m.")
add('Same original349 reference-distance pairs, separate gap segments; no new mask,')
add('no smoothing, scale fitting or changed timestamp alignment. ATE uses one SE3')
add('fit per curve over the identical mask. Each new evo CLI error array agrees')
add('with evo API; baseline ATE/path/RPE reproduce CP-V32 within1e-8.')
add('Method1 keeps each newest absolute estimate once, never replacing older points.')
add('Method2 uses native Kimera incremental ROS unchanged, without double integration.')
add('Neither is a finalized active-window history or actual physical travel.')
add('IE-TC shares CPT7 with the estimator; NOT independent prism ground truth.')
add()
add('FULL SAVED NATIVE OUTPUT (SEPARATE COVERAGE/CADENCE)')
add('Curve | ATE (m) | polyline length (m) | no-gap length (m) | ref native length (m) | rows | first..last(s)')
for name,x in d['full_saved'].items():
    add(f"{name} | {x['ATE_RMSE_m']:.6f} | {x['native_polyline_length_m']:.3f} | {x['no_gap_bridging_length_m']:.3f} | {x['reference_native_polyline_length_m']:.3f} | {x['rows']} | {x['first_s']:.6f}..{x['last_s']:.6f}; gap chords={x['gap_chords']}")
add('Both candidate output policies have exactly the original native keyframe IDs,')
add('timestamps and count. Full capture duration does not imply every instant scored.')
add()
add('REVISION / INTERNAL CONSISTENCY / COST — NOT ACCURACY METRICS')
for name in ('baseline','candidate'):
    w=d['window'][name];l=d['logs'][name]
    add(name+':')
    for key in ('previous_revision_m','absolute_step_m','same_update_motion_m','revision_rotation_deg','shape_max_m'):
        x=w[key];add(f"  {key}: median={x['median']:.6g},p95={x['p95']:.6g},max={x['maximum']:.6g}")
    add(f"  Previous-pose revisions >0.5m: {w['revisions_over_half_m']}; >1m: {w['revisions_over_one_m']}; missing paired steps: {w['missing_step_ids']}")
    for key in ('post_solve_imu_cost','post_solve_imu_cost_after_250s','post_solve_total_cost','optimize_ms','smoother_ms'):
        x=l[key];add(f"  {key}: median={x['median']:.6g},p95={x['p95']:.6g},max={x['maximum']:.6g}")
    c=l['covariance'];add(f"  MAIN covariance rows={c['rows']}, first/active={c['first']}/{c['active']}; min eigenvalue={c['eigenvalue_min']:.6g}; symmetry max={c['symmetry_max']:.6g}")
    add('  Relative pose covariance traces: '+json.dumps({k:c[k] for k in ('rot_trace_rad2','translation_trace_m2')}))
add('Runtime is measured at0.20x under this capture/audit workload, not a real-time')
add('benchmark. Old run included other estimators; compare timings with that caveat.')
add('MAIN finite/SPD/symmetry and exact native same-update increment consistency')
add('are checked. These are not statistical calibration, independence or NEES.')
add('Legacy FL4M native-prior-only columns do NOT count OriginalChartPrior adapters;')
add('their zero value does not mean prior removal. Total factor cost includes them.')
add('No CP-V35/36 passive full-factor observer is in this DSO. No fresh unwhitened')
add('component-residual or complete smart-factor validity conclusion is available.')
add()
add('TRANSPORT / OMISSIONS / REPRODUCIBILITY')
add(json.dumps(d['topic_headers'],indent=2))
add('Compressed/decompressed stereo timestamp arrays agree per side. Native output')
add('rows, IDs and timestamps exactly match original OFF. Native timing success')
add('flags all pass. Full rosbag odometry/belief recording, MAIN logs and raw costs')
add('are retained. Rounded absolute log poses use exact ROS header timestamps;')
add('each logged incremental pose is checked against native within logging bounds.')
add('Shutdown-only rostopic warning exclusions: '+json.dumps(d['nondata_warnings']))
add('Preparation helper initially double-matched a duration replacement; failed')
add('helper/receipt retained, repaired before any container/source launch. No model,')
add('candidate DSO, weights or numerical gate changed to obtain better metrics.')
add('See results.json, comparison.png, window_events.json, evo/* archives/TUMs,')
add('evo_commands.json, pair_indices.json and exact capture/effective.json.')
add('No production/default change, CBS-ON, GLIM/fusion run, deletion, commit or push.')
with (H/'REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
print('CP-V39 report written')
