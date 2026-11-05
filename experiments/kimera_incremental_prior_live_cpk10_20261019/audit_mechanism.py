"""Read-only post-capture chronology; no replay, optimization or scoring."""
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
R = W / 'kimera_incremental_prior_live_cpk10_20261019'
D = R / 'on_capture'
assert not (H / 'mechanism.json').exists(), 'Refuse overwrite'
assert json.loads((D / 'cleanup.json').read_text())['kimera_process_gone']
assert (D / 'capture_complete.json').exists() != (D / 'failed.txt').exists()

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def read_objective(p):
    rows = [[float(x) for x in row] for row in csv.reader(p.open())]
    assert rows and all(len(r) == 10 and all(map(math.isfinite, r)) for r in rows)
    assert [r[0] for r in rows] == list(range(len(rows)))
    assert all(r[1] == 0 for r in rows)
    return rows

def summarize(rows):
    retirement = [int(r[0]) for r in rows if r[2] > 0]
    return dict(
        updates=len(rows), first_retirement=retirement[0] if retirement else None,
        retirement_updates=len(retirement), retired_factors=int(sum(r[2] for r in rows)),
        observed_total_cost_increases=sum(r[5] > r[4] + 1e-10 for r in rows),
        imu_after=dict(max=max(r[7] for r in rows), median=statistics.median(r[7] for r in rows),
                       last=rows[-1][7]),
        checkpoints={str(i): dict(total_before=rows[i][4], total_after=rows[i][5],
                                 imu_before=rows[i][6], imu_after=rows[i][7])
                     for i in (4, 27, 28, 44, 100, 200, 300, 400, 438, 463, 494, 495, 531, 538)
                     if i < len(rows)})

paths = {
    'CPK10_candidate_ON': D / 'objective.csv',
    'CPK5_ordinary_ON_stopped': W / 'kimera_incremental_on_cpk5_20261019/on_capture/objective.csv',
    'CPK4b_ordinary_OFF_completed': W / 'kimera_incremental_control_cpk4b_20261019/off_capture/objective.csv',
}
rows = {name: read_objective(p) for name, p in paths.items()}
promotions = []
lines = (D / 'kimera.log').read_text().splitlines()
for line_number, line in enumerate(lines, 1):
    if 'CPK9_PRIOR_REFRESH,' not in line:
        continue
    fields = line.split('CPK9_PRIOR_REFRESH,', 1)[1].split(',')
    assert len(fields) == 5
    t, retired, priors, released, error = map(float, fields)
    assert all(map(math.isfinite, (t, retired, priors, released, error)))
    promotions.append(dict(line=line_number, backend_time=t, retirements=int(retired),
                           priors=int(priors), released_keys=int(released), cache_relative_error=error))

assert promotions, 'Candidate was never exercised'
assert max(r['cache_relative_error'] for r in promotions) <= 1e-8
prefix_n = min(len(v) for v in rows.values())
binding = {}
for name, path in paths.items():
    binding[name] = []
    for line in (path.parent / 'kimera.log').read_text().splitlines():
        if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
            fields = line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,', 1)[1].split(',')
            binding[name].append(tuple(map(int, fields[:9])))
old = binding['CPK5_ordinary_ON_stopped']
new = binding['CPK10_candidate_ON']
n = min(len(old), len(new))
first_binding_difference = next((i for i in range(n) if old[i] != new[i]), None)
old_objective_lines = paths['CPK5_ordinary_ON_stopped'].read_text().splitlines()
new_objective_lines = paths['CPK10_candidate_ON'].read_text().splitlines()
first_objective_difference = next((i for i, (a, b) in enumerate(zip(old_objective_lines, new_objective_lines))
                                   if a != b), None)
out = dict(
    capture_completed=(D / 'capture_complete.json').exists(),
    promotion_count=len(promotions), first_promotion=promotions[0], last_promotion=promotions[-1],
    max_prior_cache_relative_error=max(r['cache_relative_error'] for r in promotions),
    promotion_retirements=sum(r['retirements'] for r in promotions),
    cases={name: summarize(v) for name, v in rows.items()},
    common_update_prefix_count=prefix_n,
    common_prefix={name: summarize(v[:prefix_n]) for name, v in rows.items()},
    alignment_binding_comparison=dict(compared_rows=n, candidate_rows=len(new), old_rows=len(old),
                                      first_difference=first_binding_difference,
                                      prefix_equal=first_binding_difference is None),
    first_exact_objective_row_difference_from_CPK5=first_objective_difference,
    first_28_objective_rows_byte_equal=old_objective_lines[:28] == new_objective_lines[:28],
    source_manifest={str(p): digest(p) for p in paths.values()},
    log_sha256=digest(D / 'kimera.log'),
    qualification='Observational factor errors, not true NIS, trajectory accuracy, or nonlinear stationarity. '
                  'Promotion preserves the existing prior approximation, not eliminated nonlinear information. '
                  'Conditional saved-input ON, not a new bidirectional or fusion experiment.')
with (H / 'mechanism.json').open('x') as f:
    json.dump(out, f, indent=2, allow_nan=False)
print(json.dumps(out, indent=2, allow_nan=False))
