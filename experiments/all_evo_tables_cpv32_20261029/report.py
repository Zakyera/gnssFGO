"""Format only: every reported accuracy statistic comes from saved evo results."""
import json
from pathlib import Path

H = Path(__file__).resolve().parent
data = json.loads((H/'results.json').read_text())
assert data['passed']
cases = data['cases']


def text_file(name, value):
    with (H/name).open('x') as stream:
        stream.write(value.rstrip()+'\n')


def improve(off,on):
    return 100*(off-on)/off


def rmse(row,name):
    return row['RPE'][name]['statistics']['rmse']


def table(method,sensor):
    lines = ['| Sequence | Estimator | ATE OFF (m) | ATE ON (m) | ATE improvement | Estimated path OFF → ON (m) | RPE translation OFF → ON (m) | RPE rotation OFF → ON (°) |',
             '|---|---|---:|---:|---:|---|---|---|']
    for case,d in cases.items():
        if not case.startswith(sensor+'_'):
            continue
        for role in ('glim','kimera','fusion'):
            off=d['results'][method+'_off_'+role]; on=d['results'][method+'_on_'+role]
            lines.append(f'| {d["sequence"]} | {off["estimator"]} | {off["ATE_RMSE_m"]:.4f} | {on["ATE_RMSE_m"]:.4f} | '
                f'{improve(off["ATE_RMSE_m"],on["ATE_RMSE_m"]):+.1f}% | {off["path_length_m"]:.2f} → {on["path_length_m"]:.2f} | '
                f'{rmse(off,"translation_m"):.4f} → {rmse(on,"translation_m"):.4f} | '
                f'{rmse(off,"rotation_deg"):.3f} → {rmse(on,"rotation_deg"):.3f} |')
    return '\n'.join(lines)


coverage = ['| Sensor / sequence | Supported / capture (s) | Supported (%) | Reference path (m) | RPE pairs | Actual distance range (m) |',
            '|---|---:|---:|---:|---:|---|']
segments = []
for case,d in cases.items():
    cov=d['common_coverage']; active=[s for s in d['segments'] if s['pairs']]
    coverage.append(f'| {case} | {cov["supported_s"]:.1f} / {cov["capture_s"]:.2f} | {cov["supported_capture_percent"]:.1f}% | '
        f'{d["reference_path_length_m"]:.2f} | {d["RPE_pairs"]} | {min(s["actual_min_m"] for s in active):.6f}–{max(s["actual_max_m"] for s in active):.6f} |')
    segments.append(case+': '+', '.join(f'{s["rows"]} rows/{s["pairs"]} pairs' for s in d['segments']))
counts={}
for method in ('method1','method2'):
    counts[method]={}
    for role in ('glim','kimera','fusion'):
        pairs=[(d['results'][method+'_off_'+role],d['results'][method+'_on_'+role]) for d in cases.values()]
        counts[method][role]=dict(ATE_improves=sum(on['ATE_RMSE_m']<off['ATE_RMSE_m'] for off,on in pairs),
            translation_RPE_improves=sum(rmse(on,'translation_m')<rmse(off,'translation_m') for off,on in pairs),
            rotation_RPE_improves=sum(rmse(on,'rotation_deg')<rmse(off,'rotation_deg') for off,on in pairs))
stats = dict(passed=True, cases=7, table_rows=42, ATE_entries=84, RPE_entries=168,
    new_CLI_calls=data['new_CLI_calls'], reused_ETH1_RPE_metrics=24, input_files_verified=data['input_files_verified'],
    CLI_API_error_max_difference=max(d['verification']['CLI_API_error_max_difference'] for d in cases.values()),
    old_ATE_max_difference_m=max(d['verification']['old_ATE_max_difference_m'] for d in cases.values()),
    old_path_max_difference_m=max(d['verification']['old_path_max_difference_m'] for d in cases.values()),
    TUM_roundtrip_pose_max_component=max(d['verification']['TUM_roundtrip_pose_max_component'] for d in cases.values()),
    CBS_improvement_counts_out_of_7=counts)
with (H/'verification.json').open('x') as stream:
    json.dump(stats,stream,indent=2,allow_nan=False); stream.write('\n')

intro = '''CP-V32 — ALL SEVEN PAIRS: METHOD1 / METHOD2 WITH EVO DISTANCE RPE
Completed 2026-10-29. Original baseline Qc1.0. Saved-data evaluation only.

All ATE and RPE values in these NEW tables are computed directly by evo1.36.3.
ATE: evo_ape translation RMSE after one whole-common-mask SE3 fit, no scale.
RPE: evo_rpe translation/rotation RMSE, delta0.5m along the reference path,
consecutive pairs, no alignment. Exact same reference pairs across all methods,
estimators and OFF/ON within each sensor/mission. No pair crosses a masked gap.
Evo distance selection operates on sampled poses; actual intervals can exceed
0.5m. This is not exact spatial resampling or overlapping --all_pairs.

Estimated path: sum evo path_length over valid common segments, not physical
travel or full-native-cadence path length. Both added RPE columns show OFF→ON.
Positive ATE improvement means lower ON error. All metrics: lower is better.
Counts/coverage are below; do not label these matched-mask scores full coverage.
All historical ATE/path values reproduce within numerical precision; the NEW
provenance is direct evo scoring, not custom formulas plus evo cross-checks.
'''
definitions = {
    'method1': 'METHOD 1 — NEWEST ABSOLUTE OUTPUTS\nGLIM first-published MAIN, Kimera recovered absolute backend, fusion ONLINE.\nKimera here is NOT native incremental ROS. Older points are not replaced.\n',
    'method2': 'METHOD 2 — CAUSAL INCREMENTAL OUTPUTS\nGLIM/fusion accumulate same-update relative motions; Kimera native incremental\nROS is used unchanged, never double-integrated. Not finalized/retired output.\n',
}
sections=[]
for method in ('method1','method2'):
    block=definitions[method]+'\nHESAI\n\n'+table(method,'hesai')+'\n\nVELODYNE\n\n'+table(method,'velodyne')
    sections.append(block)
    text_file(method.upper()+'_TABLE.txt',intro+'\n'+block+'\n\nCOMMON COVERAGE\n\n'+'\n'.join(coverage))

notes = '''METRIC AND PRODUCT QUALIFICATIONS
---------------------------------
RPE translation is the translation component of relative SE3 error (metres),
not point-distance magnitude, percent drift, or ATE. Rotation is angular RMSE
in degrees, not degrees-per-metre. ATE improvement column is NOT RPE improvement.
Pooling uses raw evo error arrays plus evo statistics, never averaging segment
RMSEs. ETH1 RPE is reused exactly from CP-V31; other six pairs newly evaluated.
ATE is newly scored directly by evo for all84 product/mode/method entries.
Pair-selection flags/settings remain exactly CP-V31. delta_tol0.1 is unused by
evo's consecutive-distance mode. Segments with fewer than two triggered
endpoints produce no pair; their poses still contribute to ATE and path length.
All data are the frozen100ms grid/prism-frame poses from CP-V27/28, no new
interpolation/smoothing, change of timestamp mask or frame transformation.
Offline time association/SE3 reference alignment does not imply equal estimator
latency/evidence age or an onboard causal reference correction.
Reference IE-TC shares CPT7 with estimators; not independent prism ground truth.
Seven sensor pairs cover four missions, not seven independent experiments.
Shared OFF Kimera records are identical between sensors; small metric changes
arise from common masks and distance pairing, not independent repeated runs.
Method1 Kimera retains original six-digit log rounding bound<=0.504mm at prism.
Original terminal omissions unchanged (Hesai EIG-2 ON1, Hesai SPX-3 OFF1/ON1,
Velodyne SPX-3 OFF1); failed ARC cases are excluded, never labelled full runs.
Original WNOA Qc1.0, not later0.25/.1. Fresh ETH1 CP-V29 OFF not mixed in.

REPRODUCIBILITY
---------------
Each case folder: common and per-segment TUMs, exact pair_indices.json,
commands.json, per-call command/log/evo ZIP, pooled RPE ZIP. Epoch-to-relative
timestamp origin is stored, original integer pair timestamps retained.
evo CLI config RPE: CP-V31/evo_config.json. APE: local ape_config.json.
All CLI calls compared against evo API error arrays/statistics on same TUMs.
Path lengths from evo per-segment, independently compared to historical tables.
Inputs verified against old closeout manifests before scoring and after work.
No source/receiver run, build, container, model/parameter edit, tuning, deletion,
production/default promotion or push. Original reports and data preserved.
Final hash/checkpoint/archive readback receipt: closeout.json.

Official metric documentation: https://github.com/MichaelGrupp/evo/wiki/Metrics
'''
text_file('REPORT.txt',intro+'\n\n'.join(sections)+'\n\nCOMMON COVERAGE AND REFERENCE LENGTH\n\n'+'\n'.join(coverage)+
    '\n\nPer-segment row/pair counts (zero-pair segments are not bridged):\n'+'\n'.join(segments)+
    '\n\n'+notes+'\nNUMERICAL VERIFICATION AND DESCRIPTIVE IMPROVEMENT COUNTS\n'+json.dumps(stats,indent=2))
print(json.dumps(stats,indent=2))
