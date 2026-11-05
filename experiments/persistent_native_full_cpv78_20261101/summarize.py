"""Format saved EVO results without rerunning scoring."""
import json
from pathlib import Path
H=Path(__file__).resolve().parent;r=json.loads((H/'results.json').read_text());assert r['passed']
lines=['CP-V78 — EIG-2 HESAI,FULL355.02s, NATIVE MATCHING: TEMPORARY vs GENERAL PERSISTENT',
 'Both arms have CBS-ON saved inputs; this is NOT OFF/ON or FL4m RETAIN.',
 'Complete original raw wire stream, native admission/retries/mean alignment.',
 'Same CP76 binary, original weights and25states; no new GLIM/fusion/return peer.',
 'Method1 first absolute Kimera backend publication; not recursive ROS incremental or finalized.',
 'EVO1.36.3 only: SE3/no-scale ATE; reference-distance0.5m translation/rotation RPE.',
 'IE-TC shares CPT7; not independent prism GT. Path is estimated polyline, not real travel.', '']
for section in ('common','full_saved'):
    lines += [section.upper(),'Mode | ATE RMSE m | path m | reference path m | tRPE m | rRPE deg | poses']
    for mode,v in r[section].items():
        lines.append('%s | %.6f | %.6f | %.6f | %.6f | %.6f | %d'%(mode,v['ATE_RMSE_m'],v['path_length_m'],v['reference_path_length_m'],v['RPE']['translation_m']['statistics']['rmse'],v['RPE']['rotation_deg']['statistics']['rmse'],v['rows']))
        if section=='full_saved':lines.append('  first %.9fs, last %.9fs, span %.9fs, gaps%d; supported%.9fs'%(v['first_s'],v['last_s'],v['span_s'],v['gaps'],v['coverage_supported_s']))
lines+=['Common coverage: '+json.dumps(r['common_coverage']),'Common ATE improvement: %+.3f%%'%r['ATE_improvement_percent'],'']
for mode,d in r['diagnostics'].items():
    v=d['validation'];lines += [mode.upper()+' DIAGNOSTICS',
        'Raw incoming messages/beliefs: %d/%d; exact payload/order/transport covariance verified.'%(v['incoming_messages'],v['incoming_beliefs']),
        'Aligned candidates: %d; native admission statuses: %s'%(v['aligned_candidates'],v['native_admission_status_counts']),
        'Native decisions (events, includes retries): '+json.dumps(v['native_decision_counts']),
        'Unique last decisions: '+json.dumps(v['unique_last_decision_counts']),
        'Received sender IDs without match row: '+json.dumps(v['received_sender_ids_without_match_row']),
        'Whole previous-pose revisions: '+json.dumps(d['whole_revisions']['previous_revision_m']),
        'Late40..60s previous-pose revisions: '+json.dumps(d['late_40_60s']['previous_revision_m']),
        'Late40..60s same-update movement: '+json.dumps(d['late_40_60s']['same_update_motion_m']),
        'Final60s previous-pose revisions: '+json.dumps(d['tail_295_355s']['previous_revision_m']),
        'Costs: '+json.dumps(d['cost']),'Timings: '+json.dumps(d['timing'])]
lines += ['', 'LIMITS: complete raw input does not force identical accepted factors, keyframes or callback ordering.',
 'General persistent changes admission/replacement as well as retirement: not a retention-only causal ablation.',
 'Different total graph objectives are not directly comparable across these policies.',
 'Saved stream contains original ON feedback history; no new live bidirectional inference.',
 'CP76 runtime slot/replacement/absorption assertions active, but no logged absorption-count claim.',
 'CP67 native-only LOCAL provenance gate remains failed. No default or live-bidirectional promotion.',
 'No covariance tuning, stronger weights, output smoothing, nonlinear-stationarity claim or automatic next run.',
 'Evidence: results.json, report_verification.json, evaluation/commands.json, EVO ZIPs, per-arm validation.json.',
 'Integrity/container/owned-job closeout: verification.json and complete.json.']
with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
print('\n'.join(lines[:22]))
