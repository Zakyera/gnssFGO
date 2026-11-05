"""Format completed EVO results only; never launches or evaluates an estimator."""
import json
from pathlib import Path
H=Path(__file__).resolve().parent
r=json.loads((H/'results.json').read_text());assert r['passed']
lines=['CP-V73 -- CONDITIONAL REMOVE / RETAIN, EIG-2 HESAI, FULL355.02s',
       'Method1 first-published absolute Kimera backend poses. EVO1.36.3 only.',
       'Same1743 recorded post-aligned incoming beliefs, updates and endpoints.',
       'Both arms CBS-ON inputs: REMOVE is not CBS-OFF. No live GLIM/fusion.',
       'Original feedback history held fixed; no new return feedback.',
       'IE-TC shares CPT7; not independent prism ground truth.',
       'ATE SE3 alignment, no scale. RPE reference-distance0.5m consecutive pairs.',
       'Path is estimated polyline, not physical distance travelled.', '',
       'COMMON MASK',
       'Mode | ATE RMSE m | path m | translation RPE m | rotation RPE deg | rows']
def row(mode,v):
    return '%s | %.6f | %.6f | %.6f | %.6f | %d'%(mode,v['ATE_RMSE_m'],v['path_length_m'],
        v['RPE']['translation_m']['statistics']['rmse'],v['RPE']['rotation_deg']['statistics']['rmse'],v['rows'])
for mode,v in r['common'].items():lines.append(row(mode,v))
lines+=['Reference path: %.6fm; matched RPE pairs: %d'%(r['reference_common_path_m'],r['RPE_pairs']),
        'Common coverage: '+json.dumps(r['common_coverage']),
        'ATE improvement remove -> retain: %+.3f%%'%r['ATE_improvement_percent'], '', 'FULL SAVED -- native cadence']
for mode,v in r['full_saved'].items():
    lines+=[row(mode,v),'  first %.9fs, last %.9fs, span %.9fs, gaps %d; reference path %.6fm'%(v['first_s'],v['last_s'],v['span_s'],v['gaps'],v['reference_path_length_m'])]
lines+=['','PREVIOUS-POSE REVISION / WITHIN-SOLUTION MOTION (geometric diagnostics, not ATE/RPE)',
        'Mode | window | revision median/p95/max m | within-update motion median/p95/max m']
for mode,d in r['diagnostics'].items():
    for name,key in [('whole','whole_revisions'),('40..60s','late_40_60s'),('295.02..355.02s','final_60s')]:
        s=d[key];a=s['previous_revision_m'];b=s['same_update_motion_m']
        lines.append('%s | %s | %.6f / %.6f / %.6f | %.6f / %.6f / %.6f'%(mode,name,a['median'],a['p95'],a['maximum'],b['median'],b['p95'],b['maximum']))
    lines+=['  Costs: '+json.dumps(d['cost']), '  Timing: '+json.dumps(d['timing']),
            '  Separate replay readiness: '+json.dumps(d['validation']['readiness']),
            '  Early messages held until target update: %d'%d['validation']['held_early_count']]
frozen=json.loads((Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'replay/frozen_inputs.json').read_text())
lines+=['','Original unaccepted wire beliefs omitted from both arms: '+str(len(frozen['omitted_original_unaccepted']))+'; exact list: replay/frozen_inputs.json (workspace run root).',
        'Original native timestamp gaps >0.5s: '+json.dumps(frozen['original_gaps_over_half_second']),
        'LIMITS: full selected355.02s conditional replay, not a live bidirectional safety certificate.',
        'CP67 LOCAL provenance gate remains FAILED. Retained marginal history includes peer information.',
        'No covariance weights, prior functions, native smart factors or solver iterations changed.',
        'No nonlinear stationarity, independent covariance, default promotion or final universal fix claim.',
        'Machine-readable evidence: results.json, evaluation/commands.json and EVO ZIP archives.',
        'Closeout: verification.json and complete.json; original CP69 and CP71 failures preserved.']
with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
print('\n'.join(lines[:20]))
