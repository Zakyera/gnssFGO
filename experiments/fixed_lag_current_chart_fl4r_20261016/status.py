"""Read-only in-flight progress; check completed rows, never start/retry a run."""
from pathlib import Path
import csv, json, re

R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4r_20261016')
Q = R.parent / 'fixed_lag_current_chart_fl4q_20261016/remove_capture'
D = R / 'remove_capture'
def rows(p, width):
    if not p.exists(): return []
    text = p.read_text()
    # Concurrent reader: only fully written, newline-terminated records.
    text = text[:text.rfind('\n') + 1]
    data = [[float(v) for v in row] for row in csv.reader(text.splitlines())]
    assert all(len(row) == width for row in data), p
    return data
a = rows(D / 'acceptance.csv', 9)
qa = rows(Q / 'acceptance.csv', 9)
for i, row in enumerate(a):
    assert row[0] == i
    assert row[3] <= row[1] + 1e-10 * (1 + abs(row[1])), ('cost increased', i)
    assert max(row[7:]) <= 1e-8, ('model check', i)
    if i < len(qa):
        assert row[:6] + row[7:] == qa[i][:6] + qa[i][7:], ('short prefix differs', i)
fallback = [row for row in a if row[4] == 1]
text = (D/'kimera.log').read_text() if (D/'kimera.log').exists() else ''
lines = text[:text.rfind('\n')+1].splitlines()
expected = json.loads((R/'frozen_inputs.json').read_text())['rows']
bound = [line.split('FL4M_FROZEN_FACTOR_ROW,',1)[1].split(',')
         for line in lines if 'FL4M_FROZEN_FACTOR_ROW,' in line]
assert len(bound) <= len(expected)
for v, r in zip(bound, expected):
    assert list(map(int,v[:5])) == [r['update']] + r['sender'] + r['receiver']
    assert max(abs(float(x)-y) for x,y in zip(v[5].split(';'),r['aligned'])) < 1e-12
    assert [float(x) for x in re.split('[;:]',v[6])] == r['cov']
print(json.dumps(dict(completed_updates=len(a), last_update=int(a[-1][0]) if a else None,
    fallback_updates=[int(row[0]) for row in fallback],
    accepted_nonincrease_and_model_checks=True, short_prefix_rows_equal=min(len(a),len(qa)),
    exact_logged_input_bindings=len(bound),
    complete=(D/'complete.json').exists(), failure=(D/'failed.txt').read_text() if (D/'failed.txt').exists() else None)))
