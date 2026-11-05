"""Assemble the complete method2 comparison without changing any estimates."""
import json,sys
from pathlib import Path
sys.dont_write_bytecode=True
import analyze as m
H=Path(__file__).resolve().parent
assert not (H/'REPORT.txt').exists() and not (H/'summary.json').exists()
v=json.loads((H/'verification.json').read_text());assert v['passed']
cases={r[0]:json.loads((H/(r[0]+'.json')).read_text()) for r in m.c.b.ROWS}
labels={'glim':'GLIM','kimera':'Kimera','fusion':'Fusion'}
lines=['CP-V28 — ALL SEVEN BASELINE PAIRS: METHOD 2',
       '2026-10-28 | saved-data reconstruction and receiver-only replay | Qc multiplier1.0', '',
       '1. WHAT IS EVALUATED', '',
       'Every primary trajectory is accumulated same-update relative motion:',
       'O[0]=X[0|0]; O[k]=O[k-1] inverse(X[k-1|k]) X[k|k].',
       'Both the plotted/path positions AND the positions used for RMSE come from O. This is not absolute-pose RMSE paired with a different incremental plot.',
       'GLIM: saved same-update MAIN pose pairs reconstructed at exact native CSV timestamps.',
       'Kimera: original native incremental ROS odometry, unchanged and NOT integrated twice.',
       'Fusion: same-publication pose pairs recovered using the existing read-only CP-V23 observer binary and original saved event journals. Twelve receiver replays; completed Leica-2 Velodyne replay reused. No GLIM/Kimera/sensor rerun or new build.',
       'The receiver keeps original Qc1.0, seven-second fixed lag, source_wnoa,100ms supports, startup59, priors, source data, revisions, caches and EOF semantics. These are NOT Qc0.25 or0.1 results.',
       'CP-V27 is the separate method1 report and does not satisfy the method2 request. No CP-V27 absolute-Kimera score is used here.', '',
       '2. MATCHED-TIME PRIMARY RESULTS', '',
       'RMSE is position ATE after one rigid SE3 no-scale fit per trajectory over the complete frozen CP-V10 common mask. Same100ms queries for all six outputs within a pair. No smoothing, per-segment realignment or outcome-dependent trimming.',
       'Improvement =100*(OFF-ON)/OFF. Positive is better; negative is worse. Estimated path sums supported adjacent segments only, never bridging masked gaps. It is not physical distance flown.', '']
for sensor in ('Hesai','Velodyne'):
    lines += [sensor.upper(),'', '| Sequence | Estimator | RMSE OFF (m) | RMSE ON (m) | Improvement | Estimated path OFF -> ON (m) |','|---|---|---:|---:|---:|---|']
    for d in cases.values():
        if d['sensor']!=sensor:continue
        for role,label in labels.items():
            off,on=d['common']['off_'+role],d['common']['on_'+role]
            lines.append(f"| {d['sequence']} | {label} | {off['ATE_RMSE_m']:.4f} | {on['ATE_RMSE_m']:.4f} | {d['off_on'][role]:+.1f}% | {off['path_length_m']:.2f} -> {on['path_length_m']:.2f} |")
    lines.append('')
lines += ['3. COMMON COVERAGE AND REFERENCE LENGTH','','| Sensor / sequence | Capture s | Supported s (%) | Reference path m | First / last s |','|---|---:|---|---:|---|']
for d in cases.values():
    cv=d['common_coverage'];ref=d['common']['off_glim']['reference_path_length_m']
    lines.append(f"| {d['sensor']} {d['sequence']} | {cv['capture_s']:.2f} | {cv['supported_s']:.1f} ({cv['supported_capture_percent']:.1f}%) | {ref:.2f} | {cv['first_s']:.3f} / {cv['last_s']:.3f} |")
lines += ['', 'Coverage is connected support duration, not last minus first. Gaps exceeding500ms in a source are not interpolated. Full saved output was retained; incomplete common coverage is not labelled a failed capture or missing-camera diagnosis.',
          'The reference is IE-TC GNSS/INS sharing CPT7 IMU with the estimators, not independent prism ground truth. All estimates/reference use the unchanged body-to-prism lever and orientation.', '',
          '4. FULL SAVED NATIVE OUTPUTS — SECONDARY, DIFFERENT SAMPLE TIMES', '',
          'All actual saved rows have reference support. These values include each estimator\'s own time span/cadence, so use section2 for matched-time comparisons. Full native polyline length includes any long-gap chord, whose count is listed.', '',
          '| Sensor / sequence | Estimator | RMSE OFF / ON m | Path OFF / ON m | Rows OFF / ON | Span OFF / ON s | Gaps >500ms OFF / ON |','|---|---|---|---|---|---|---|']
for d in cases.values():
    for role,label in labels.items():
        off,on=d['full_saved']['off_'+role],d['full_saved']['on_'+role]
        lines.append(f"| {d['sensor']} {d['sequence']} | {label} | {off['ATE_RMSE_m']:.6f} / {on['ATE_RMSE_m']:.6f} | {off['native_polyline_length_m']:.3f} / {on['native_polyline_length_m']:.3f} | {off['rows']} / {on['rows']} | {off['span_s']:.3f} / {on['span_s']:.3f} | {off['gap_chord_count']} / {on['gap_chord_count']} |")
lines += ['', '5. ONE-SECOND RELATIVE-POSE ERRORS ON THE SAME COMMON MASK', '',
          '| Sensor / sequence | Estimator | Translation RMSE OFF / ON m | Rotation RMSE OFF / ON deg |','|---|---|---|---|']
for d in cases.values():
    for role,label in labels.items():
        off,on=(d['common'][m+'_'+role]['horizons']['1'] for m in ('off','on'))
        lines.append(f"| {d['sensor']} {d['sequence']} | {label} | {off['translation_RMSE_m']:.6f} / {on['translation_RMSE_m']:.6f} | {off['rotation_RMSE_deg']:.6f} / {on['rotation_RMSE_deg']:.6f} |")
counts={r:sum(d['off_on'][r]>0 for d in cases.values()) for r in labels}
winners={r:0 for r in labels}
for d in cases.values():
    for mode in ('off','on'):winners[min(labels,key=lambda r:d['common'][mode+'_'+r]['ATE_RMSE_m'])]+=1
lines += ['', '6. INTERPRETATION', '',
          'CBS lowers method2 common ATE in '+', '.join(f'{labels[r]} {counts[r]}/7 pairs' for r in labels)+'.',
          'Lowest common ATE across14modes: '+', '.join(f'{labels[r]} {winners[r]}/14' for r in labels)+'. These are single-run descriptive counts, not statistical significance.',
          'Method2 removes accumulated graph correction jumps from the output chain, but it also discards later corrections to already accumulated motion. Smooth/short paths can therefore drift substantially. This measures a different causal odometry output, not a mere plotting repair.',
          'Do not infer that GLIM authors report this derived output or that this is its standard benchmark score. No author-evaluation claim is made. Native/internal corrected graph states and fixed-lag-finalized outputs remain distinct products.',
          'Matching the publication convention does not equalize source cadence, evidence age, latency, lag or input information. Offline reference alignment/interpolation are evaluation operations, not causal onboard corrections.',
          'Kimera OFF files for shared missions are byte-identical between Hesai/Velodyne, not independent repeats; small common-score differences reflect different pair masks. Rear Velodyne also changes viewpoint. Four physical missions, seven sensor pairs.', '',
          '7. VALIDATION, PRESERVATION AND LIMITS', '',
          'All twelve replays reproduce original ONLINE/finalized state CSVs and TUMs byte-for-byte, exact summaries, all deterministic event fields and every publication commit record. Existing Leica-2 Velodyne CP-V23 parity is reused, not rerun. Runtime/dependency and input hashes are unchanged. No extra solve is performed by the observer.',
          'GLIM same-update rows match every native current publication to double precision, no missing pair. Exact nanosecond native timestamps replace rounded log stamps. Chaining independently agrees with quaternion/vector recurrence; telescoping/no-revision controls, global/body-frame equivariance, graph reconstruction and prism edge checks pass.',
          f"Read back {v['recovered_glim_and_fusion_poses_read_back']} chained poses with exact timestamps. Independent evo rigid alignment maximum difference {v['evo_alignment_max_difference_m']:.3g}m; independent matrix RPE maximum difference {v['matrix_RPE_max_difference']:.3g}.",
          'All14native Kimera full/common scores reproduce CP-V10; all Leica-2 Velodyne method2 full/common scores reproduce CP-V26. Frozen common masks/reference arrays unchanged. Per-case JSON retains1s/10s/20s RPE, full-saved details and input hashes.',
          'Terminal-factor omissions retained: Hesai EIG-2 ON1; Hesai SPX-3 OFF1/ON1; Velodyne SPX-3 OFF1; other successful baseline modes0. No relaxation/retry to get better metrics. ARC failed prefixes and tuning candidates excluded.',
          'All new bulky replay files retained losslessly in verified compressed archives. Only verified duplicate temporary RAM working trees removed; no original artifact deleted. Container/process/checkpoint closeout is recorded separately in closeout.json.',
          'Storage-only interruption: after three successful cases, the original controller stopped before the fourth launch because SSD free space crossed its inherited8GiB threshold. Outputs were on RAM tmpfs. The failed receipt and first three successes were preserved; a documented continuation checked actual tmpfs capacity and projected persistent SSD headroom (>4GiB), then launched only the nine never-started cases. No duplicate replay, changed estimator setting or numerical/parity guard relaxation. See STORAGE_CONTINUATION.txt.',
          'No new chained-output covariance or velocity is assigned; no production change, tuning, default promotion, nonlinear-stationarity certification, commit/push or source run. Keep both corrected graph and chained odometry products labelled; this table alone does not select a replacement publisher.', '',
          'FILES: summary.json, <case>.json/.npz, per-case method2 TUMs, GLIM/fusion publication products, reconstruction/parity receipts, verification.json, input pins, replay_evidence/*.tar.gz and member manifests, closeout.json.']
m.a.save(H/'summary.json',dict(method=2,cases=cases,improved_pair_counts=counts,lowest_ATE_counts=winners,verification=v))
with (H/'REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
print('\n'.join(lines[:lines.index('3. COMMON COVERAGE AND REFERENCE LENGTH')]))
