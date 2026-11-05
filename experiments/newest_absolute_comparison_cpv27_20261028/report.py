"""Create the requested CP-V27 tables from verified saved results only."""
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
import analyze as m
import numpy as np

H=Path(__file__).resolve().parent
assert not (H/'REPORT.txt').exists() and not (H/'summary.json').exists()
verification=json.loads((H/'verification.json').read_text())
assert verification['passed']
cases={r[0]:json.loads((H/(r[0]+'.json')).read_text()) for r in m.c.b.ROWS}
labels={'glim_initial':'GLIM','kimera_absolute':'Kimera absolute backend','fusion_online':'Fusion ONLINE'}
lines=['CP-V27 — SEVEN-PAIR NEWEST ABSOLUTE GRAPH OUTPUT COMPARISON',
       'Completed 2026-10-28. Saved-data evaluation only; original receiver Qc1.0.', '',
       '1. MEANING OF THIS NEW EVALUATION', '',
       'All primary products use the newest absolute graph pose available at their output update.',
       'GLIM: first-published optimized MAIN poses in glim_odometry.csv, unchanged from CP-V10.',
       'Kimera: absolute W_Pose_B_lkf_from_state_ recovered from same-output diagnostics. This is NOT its actual incremental ROS odometry publication. No native output was replaced or relabelled.',
       'Fusion: original concurrent ONLINE newest-support poses, unchanged from CP-V10; no new replay.',
       'No fixed-lag-finalized/retirement output and no accumulating relative-motion chain enter the primary tables.',
       'This matches the publication rule, not cadence, evidence age, latency or fixed-lag duration.',
       'CP-V10 native incremental Kimera results and CP-V26 all-incremental Leica-2 remain separate valid product measurements.', '',
       '2. PRIMARY MATCHED-TIME POSITION ATE AND CONNECTED PUBLICATION PATH', '',
       'All distances in metres. Same100ms query grid and exact CP-V10 common mask for all six products within each pair. Each trajectory gets one rigid SE(3) no-scale reference fit over its complete common mask; no per-segment realignment. No smoothing.',
       'Improvement=100*(OFF-ON)/OFF: positive better, negative worse. Single-run descriptive differences, not significance tests.',
       'Path is the length of the connected published-position trace on supported grid segments, not physical distance flown. No chords across masked gaps.',
       'The table is NOT a claim of100% time coverage. Full-saved own-sampling values are in section4.', '']
for sensor in ('Hesai','Velodyne'):
    lines += [sensor.upper(),'', '| Sequence | Estimator | RMSE OFF m | RMSE ON m | Improvement | Estimated path OFF -> ON m |',
              '|---|---|---:|---:|---:|---|']
    for d in cases.values():
        if d['sensor']!=sensor: continue
        for role,label in labels.items():
            off=d['common']['off_'+role]; on=d['common']['on_'+role]
            lines.append(f"| {d['sequence']} | {label} | {off['ATE_RMSE_m']:.4f} | {on['ATE_RMSE_m']:.4f} | {d['off_on'][role]:+.1f}% | {off['path_length_m']:.2f} -> {on['path_length_m']:.2f} |")
    lines.append('')
lines += ['3. COMMON COVERAGE AND REFERENCE DISTANCE', '',
          '| Sensor / sequence | Capture s | Supported s (%) | Reference path m | First / last supported s |',
          '|---|---:|---:|---:|---|']
for d in cases.values():
    v=d['common_coverage']; ref=d['common']['off_glim_initial']['reference_path_length_m']
    lines.append(f"| {d['sensor']} {d['sequence']} | {v['capture_s']:.2f} | {v['supported_s']:.1f} ({v['supported_capture_percent']:.1f}%) | {ref:.2f} | {v['first_s']:.3f} / {v['last_s']:.3f} |")
lines += ['', 'Supported duration is summed over connected segments, not simply last minus first. Sparse Kimera keyframes exceed the unchanged500ms interpolation bound in some intervals/tails. This is an evaluation-support limitation, not evidence that the complete capture failed or that camera data were missing.',
          'Small differences in Kimera OFF common scores between Hesai/Velodyne arise from different common masks; the actual OFF Kimera absolute arrays are exactly identical for each shared mission.', '',
          '4. FULL-SAVED OWN-SAMPLING OUTPUTS — SECONDARY', '',
          'Every actual saved pose is reference-supported. These values use each product\'s own sample times/span and are not matched-time rankings. Native path lengths join adjacent saved points, including any long-gap chords; gap counts are included. No output is called physically travelled distance.', '',
          '| Sensor / sequence | Estimator | ATE OFF / ON m | Path OFF / ON m | Rows OFF / ON | Span OFF / ON s | Gap chords OFF / ON |',
          '|---|---|---|---|---|---|---|']
for d in cases.values():
    for role,label in labels.items():
        off=d['full_saved']['off_'+role]; on=d['full_saved']['on_'+role]
        lines.append(f"| {d['sensor']} {d['sequence']} | {label} | {off['ATE_RMSE_m']:.6f} / {on['ATE_RMSE_m']:.6f} | {off['native_polyline_length_m']:.3f} / {on['native_polyline_length_m']:.3f} | {off['rows']} / {on['rows']} | {off['span_s']:.3f} / {on['span_s']:.3f} | {off['gap_chord_count']} / {on['gap_chord_count']} |")
lines += ['', '5. WHAT CHANGED FOR KIMERA?', '',
          '| Sensor / sequence | Incremental common ATE OFF / ON m | Absolute common ATE OFF / ON m |',
          '|---|---|---|']
for d in cases.values():
    v=d['common']
    lines.append(f"| {d['sensor']} {d['sequence']} | {v['off_kimera']['ATE_RMSE_m']:.4f} / {v['on_kimera']['ATE_RMSE_m']:.4f} | {v['off_kimera_absolute']['ATE_RMSE_m']:.4f} / {v['on_kimera_absolute']['ATE_RMSE_m']:.4f} |")
winners={r:0 for r in labels}
for d in cases.values():
    for mode in ('off','on'):
        best=min(labels,key=lambda r:d['common'][mode+'_'+r]['ATE_RMSE_m'])
        winners[best]+=1
assert winners=={'glim_initial':13,'kimera_absolute':1,'fusion_online':0}
lines += ['',
          'Under this policy GLIM has the lowest common ATE in13/14 modes, Kimera in1/14 (Velodyne Leica-2 OFF), and fusion in0/14. CBS lowers ATE in3/7 GLIM,6/7 Kimera and6/7 fusion pairs.',
          'Leica-2 Velodyne still contains the important counterexample: Kimera OFF1.005375m beats GLIM2.782169m. ON improves GLIM to0.848869m but worsens Kimera to1.532446m. Fusion ON1.867962m is worse than both sources.',
          'Kimera absolute output is NOT uniformly better than its incremental output. ETH1 ON changes0.340734->1.606029m; Leica-2 Velodyne ON changes2.585334->1.532446m. Output policy can improve or worsen ATE.',
          'Absolute-history Kimera paths are very long in ETH1 and EIG-2, including782.32m Hesai ETH1 ON against201.38m matched reference and790.17m Velodyne EIG-2 ON against188.02m. These are real saved graph-publication discontinuities, not injected by an incremental reconstruction or by smoothing. They do not establish physical zigzag or the motion between two simultaneously revised active states. No specific optimizer bug or cause is diagnosed here.',
          'Hesai SPX-3 GLIM ON worsens by only0.465mm common ATE despite the displayed-2.8%; do not exaggerate this small absolute change.', '',
          '6. EXTRACTION AND VALIDATION', '',
          'Pinned VioBackend.cpp:1967 logs timestamp/keyframe, absolute state pose, incremental pose and a separate state-to-increment transform immediately before BackendOutput creation. The recorded flag is no_incremental_pose=false. updateStates reads the current MAIN state into W_Pose_B_lkf_from_state_; the alternate incremental branch composes same-solution between-keyframe motion.',
          'All21,608Kimera output rows across14runs have exactly one unique contiguous keyframe ID matching CSV header.seq. Every paired logged incremental pose matches the original ROS pose within its six-significant-digit rounding interval. The original exact nanosecond CSV header stamp is used. The coarse logged epoch stamp is NEVER used for timestamp association or scoring.',
          'Kimera absolute pose coordinates are rounded in the original log, not double-precision exact. Conservative maximum prism-position rounding bound0.503628mm; actual rounded-incremental-control ATE discrepancy <=0.001837mm. Treat last printed decimals as numerical reporting, not proven physical accuracy. Quaternions are normalized; no trajectory smoothing or additional solve.',
          'Independent graph-to-increment translation/norm/yaw diagnostics match within rounding bounds; pose ordering, quaternion ordering and frame use are checked. Exact OFF absolute arrays match across sensors on the three shared missions.',
          'All42unchanged GLIM/Kimera-incremental/fusion products reproduce prior full/common ATE exactly. All common masks and reference arrays are unchanged. Independent evo rigid alignment maximum difference1.29e-14m; homogeneous-matrix RPE check maximum3.91e-14. Existing identity/frame/known-angle/gap/exact-sample tests pass.',
          '109input SHA256 pins verified unchanged after scoring. All14source protected manifests match the pinned Kimera implementation. verification.json contains exact bounds/checks; each case JSON keeps1s/10s/20s RPE, full-saved details, omissions and input identities.', '',
          '7. LIMITS AND PRESERVATION', '',
          'Reference is IE-TC GNSS/INS with shared CPT7 IMU, not independent prism ground truth. Estimate/reference receive the same original CPT7-to-prism lever transform. Reference fitting is offline and uses the complete evaluation mask; this is not raw unaligned navigation error.',
          'Same publication rule does not equalize latency, native update cadence, evidence age or lag. Offline interpolation is not a newly causal streaming product. This study evaluates a saved Kimera backend output alternative; it does not change the deployed ROS publisher or CBS exchange.',
          'Four missions, seven sensor pairs; shared Kimera OFF data are not independent repeats. Rear Velodyne versus Hesai also changes viewpoint. No statistical significance, covariance calibration, healthy-estimator generalization or0.5x claim.',
          'Preserved terminal-factor omissions: Hesai EIG-2 ON1; Hesai SPX-3 OFF1/ON1; Velodyne SPX-3 OFF1. The other successful baseline modes have0. ARC-5 failed incomplete, ARC-3 separate failures and rejected tuning/projection candidates are excluded, not relabelled as full results.',
          'No estimator run, receiver replay, container start, build, tuning, default promotion, deletion, commit or push. Original Qc1.0 retained;0.25/.1receiver candidates are not included. Original reports/trajectories and CP-V10/CP-V26 are preserved.', '',
          'FILES: analyze.py, verify.py, report.py, PROTOCOL.txt, verification.json, summary.json, <case>.json and <case>.npz. NPZ includes recovered raw Kimera absolute poses/timestamps and all common-grid positions/rotations; no separate smoothing step.']
# Derive a diagnostic directly from recovered native graph points, independent
# of the common-grid path summation. No event is discarded because it is large.
steps={}
for name,d in cases.items():
    z=np.load(H/(name+'.npz')); steps[name]={}
    for mode in ('off','on'):
        p=z[mode+'_kimera_absolute_body_position'];r=z[mode+'_kimera_absolute_body_rotation']; t=z[mode+'_kimera_timestamp_ns']
        p,_=m.a.prism(p,r); delta=np.linalg.norm(np.diff(p,axis=0),axis=1); i=int(delta.argmax())
        steps[name][mode]=dict(max_adjacent_publication_step_m=float(delta[i]),interval_s=float((t[i+1]-t[i])*1e-9),
                              previous_header_stamp_ns=int(t[i]),current_header_stamp_ns=int(t[i+1]))
m.a.save(H/'summary.json',dict(cases=cases,verification=verification,winners=winners,largest_kimera_publication_steps=steps))
with (H/'REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
print(json.dumps(dict(report=str(H/'REPORT.txt'),winners=winners,largest_steps=steps),indent=2))
