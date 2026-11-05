"""Summarize existing native inspector outputs; no replay, scoring or solve."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P = W / 'runs' / H.name
Q = W / 'runs/eig2_on_checkpoint_cpv44_20261029'
pins = {}


def pin(path):
    path = Path(path)
    with path.open('rb') as stream:
        pins[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return path


def table(path):
    with pin(path).open() as stream:
        return [dict((k, v if k == 'phase' else float(v)) for k, v in row.items())
                for row in csv.DictReader(stream)]


early = json.loads(pin(H / 'early_analysis.json').read_text())
pins.update(early['input_sha256'])
init = table(P / 'attempt_api1/results/initialization.csv')
imu = table(P / 'attempt_api1/results/imu_rows.csv')
history = table(P / 'attempt_api1/results/pim_history.csv')
join = table(P / 'rotation_join.log')
with pin(Q / 'prepared/full_on_source/kimera_odometry.csv').open() as stream:
    stamps = [int(r['field.header.stamp']) for r in csv.DictReader(stream)
              if r.get('field.header.seq', '').isdigit()]
duration_errors = [abs(r['dt'] - (stamps[int(r['to'])] - stamps[int(r['from'])]) * 1e-9)
                   for r in imu]
assert max(duration_errors) < 1e-12, max(duration_errors)
assert len(init) == len(join) == 5 and len(imu) == 455 and len(history) == 340
assert all(r['unchanged'] == 1 for r in history)
assert all(r['new_bias_copy_difference'] == r['new_velocity_predict_difference'] == 0 for r in init)
assert max(r['old_initial_tangent_difference'] for r in init) < 1e-12
assert max(r['previous_final_continuity'] for r in init) < 1e-12
assert all(r['bias_hat_older_published_acc_difference'] == r['bias_hat_older_published_gyro_difference'] == 0
           for r in init[1:])
assert max(r['initial_vs_raw_pim_rotation_deg'] for r in join) < 1e-12
assert all(r['velocity_vs_corrected_predict_m'] == 0 for r in join)
assert max(r['predict_self_error'] for r in imu) < 1e-9
for a, b in zip(init, join):
    assert a['update'] == b['update']
    assert abs(a['new_pose_predict_rotation_deg'] - b['initial_vs_corrected_rotation_deg']) < 1e-12
    assert abs(a['new_pose_predict_position_difference'] - b['position_vs_corrected_predict_m']) < 1e-12

costs = []
for k in range(21, 26):
    for phase in ('initialized', 'after_ordinary', 'after_refresh'):
        rows = [r for r in imu if r['update'] == k and r['phase'] == phase]
        newest = next(r for r in rows if r['to'] == k)
        total = sum(r['cost'] for r in rows)
        costs.append(dict(update=k, phase=phase, total_imu_cost=total,
                          newest_fraction_of_imu_cost=newest['cost'] / total,
                          newest=newest))

for p in [W / 'src/Kimera-VIO/src/frontend/Tracker.cpp',
          Q / 'prepared/full_on_source/kimera_profile/FrontendParams.yaml',
          Q / 'prepared/full_on_source/kimera_profile/flags/stereoVIOEuroc.flags',
          H / 'summarize.py', H / 'analyze_early.py', H / 'rotation_join.py',
          P / 'RotationJoin.cpp', P / 'attempt_api1/Inspect.cpp',
          P / 'attempt_api1/release_selftest.log', P / 'attempt_api1/san_selftest.log']:
    pin(p)

off_on = early['comparisons']['off42_vs_on42']
assert off_on['first_difference']['update'] == 6
assert all(r['same_pose_values'] for r in off_on['rows'] if r['update'] <= 5)
assert all(not e['admissions'] for e in early['events']['off42'])
assert next(e['update'] for e in early['events']['on42'] if e['admissions']) == 6

out = dict(
    scope='Saved diagnosis only; mixed-reference initialization observed, root cause not established.',
    first_difference=off_on['first_difference'],
    initialization=init, raw_corrected_rotation_join=join, newest_imu_costs=costs,
    checks=dict(
        saved_windows=5, native_phase_snapshots=15,
        imu_rows=len(imu), phase_counts=dict(Counter(r['phase'] for r in imu)),
        pim_appearances=len(history), unique_pims=len({r['to'] for r in history}),
        repeated_pim_byte_comparisons=int(sum(r['previously_seen'] for r in history)),
        unchanged_pims=all(r['unchanged'] == 1 for r in history),
        max_endpoint_duration_error_s=max(duration_errors),
        max_predict_self_error=max(r['predict_self_error'] for r in imu),
        max_raw_rotation_identity_error_deg=max(r['initial_vs_raw_pim_rotation_deg'] for r in join),
        previous_final_continuity_comparisons=4, bias_hat_older_published_comparisons=4,
        max_old_initial_tangent_difference=max(r['old_initial_tangent_difference'] for r in init),
        max_previous_final_continuity=max(r['previous_final_continuity'] for r in init)),
    source_replays=0, optimizer_calls=0, production_edits=False,
    covariance_analysis=False, new_trajectory_scores=False, input_sha256=pins,
    limitations=[
        'Full X/V/b/PIM snapshots begin at21; first CBS admission6 is only pose/cost chronology.',
        'CP-V44 first differs from CP-V42 at6; it is not an exact replay of CP-V42.',
        'No raw IMU reintegration; first-order correction accuracy is untested.',
        'No frontend bias-reset runtime receipt or complete early visual-inlier history.',
        'Current frontend/GTSAM code is separately hashed inspection evidence, not asserted archived build provenance.',
        'A stereo pose and IMU velocity may legitimately disagree. Observed mixed references are not alone a proven bug.',
        'Changing initialization would not undo frontend matching/inlier choices or establish first cause.',
        'Tilt is inter-update window motion, not attitude error against independent ground truth.'])
with (H / 'analysis.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print(json.dumps(out['checks'], indent=2))
for r in costs:
    if r['phase'] == 'initialized':
        print(r['update'], r['total_imu_cost'], r['newest_fraction_of_imu_cost'])
