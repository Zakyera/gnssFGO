"""Audit every retained timing sample; no estimation, reruns or data selection."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import xml.etree.ElementTree as ET

def quantile(values, p):
    v = sorted(values)
    index = (len(v) - 1) * p
    lo, hi = math.floor(index), math.ceil(index)
    return v[lo] + (v[hi] - v[lo]) * (index - lo)

def stats(values):
    return dict(n=len(values), mean=statistics.mean(values),
                median=statistics.median(values), p95=quantile(values, .95),
                minimum=min(values), maximum=max(values), total=sum(values))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = args.directory
    assert json.loads((root / 'passed.json').read_text())['passed']
    commands = json.loads((root / 'commands.json').read_text())
    assert len(commands) == 1 and commands[0]['returncode'] == 0
    xml = ET.parse(root / 'test_solver.xml').getroot()
    assert int(xml.attrib['tests']) == 1 and int(xml.attrib['failures']) == 0
    assert int(xml.attrib.get('errors', 0)) == 0
    assert not xml.findall('.//failure') and not xml.findall('.//error')
    rows, batches, states, errors, ordering = {}, {}, [], [], []
    for line in (root / 'test_solver.log').read_text().splitlines():
        if line.startswith('E1B4_TIMING,'):
            f = line.split(',')
            assert len(f) == 11, f
            key = tuple(map(int, f[1:5]))
            assert key not in rows, key
            row = dict(zip(['milliseconds', 'gpu_calls', 'sync_calls', 'cache_hits',
                            'active_keys', 'factor_slots'],
                           [float(f[5]), *map(int, f[6:])]))
            assert math.isfinite(row['milliseconds']) and row['milliseconds'] > 0
            rows[key] = row
            if key[-1] == 0:
                ordering.append(key[:3])
        elif line.startswith('E1B4_BATCH_COUNTS,'):
            f = list(map(int, line.split(',')[1:]))
            assert len(f) == 6 and tuple(f[:3]) not in batches
            batches[tuple(f[:3])] = dict(zip(['requests', 'batched_graphs', 'prepared_factors'], f[3:]))
        elif line.startswith('E1B4_STATE_MAX,'):
            states.append(list(map(float, line.split(',')[1:])))
        elif line.startswith('E1_ERROR '):
            match = re.fullmatch(r'E1_ERROR rows=(\d+) cols=(\d+) max_abs=(\S+) bound=(\S+)', line)
            assert match, line
            n, m, difference, bound = match.groups()
            assert float(difference) <= float(bound)
            errors.append(dict(rows=int(n), columns=int(m), max_abs=float(difference), bound=float(bound)))
    expected = {(w, r, mode, i) for w in [0, 64] for r in range(6) for mode in [0, 1] for i in range(40)}
    assert rows.keys() == expected and len(batches) == 24
    assert len(states) == 12 and len(errors) == 24
    assert ordering == [(w, r, int((r % 2) == order)) for w in [0, 64] for r in range(6) for order in [0, 1]]
    maxima = [max(s[i] for s in states) for i in range(5)]
    assert maxima[0] <= 1e-9 and maxima[1] <= 1e-10 and max(maxima[2:4]) <= 1e-9
    for w, r, mode, i in expected:
        row = rows[w, r, mode, i]
        other = rows[w, r, 1-mode, i]
        for field in ['gpu_calls', 'active_keys', 'factor_slots']:
            assert row[field] == other[field], (w, r, i, field)
        assert row['gpu_calls'] == row['sync_calls'] + row['cache_hits']
    results = {}
    for w in [0, 64]:
        cases = {}
        for mode, name in [(0, 'ordinary'), (1, 'batched')]:
            subset = [rows[w, r, mode, i] for r in range(6) for i in range(40)]
            cases[name] = dict(timing_ms=stats([row['milliseconds'] for row in subset]),
                totals={field: sum(row[field] for row in subset) for field in ['gpu_calls', 'sync_calls', 'cache_hits']},
                active_key_range=[min(row['active_keys'] for row in subset), max(row['active_keys'] for row in subset)],
                factor_slot_range=[min(row['factor_slots'] for row in subset), max(row['factor_slots'] for row in subset)],
                batch_totals={field: sum(batches[w, r, mode][field] for r in range(6)) for field in ['requests', 'batched_graphs', 'prepared_factors']},
                rounds=[stats([rows[w, r, mode, i]['milliseconds'] for i in range(40)]) for r in range(6)])
        paired = []
        for r in range(6):
            a, b = [cases[name]['rounds'][r] for name in ['ordinary', 'batched']]
            paired.append(dict(round=r, first='batched' if r % 2 == 0 else 'ordinary',
                               median_ratio=b['median']/a['median'], p95_ratio=b['p95']/a['p95'],
                               median_delta_ms=b['median']-a['median']))
        ratios = {k: cases['batched']['timing_ms'][k] / cases['ordinary']['timing_ms'][k] for k in ['mean', 'median', 'p95', 'total']}
        results[str(w)] = dict(**cases, ratios=ratios, paired_rounds=paired)
    result = dict(passed_numerical_checks=True, all_samples_accounted=True, timing_rows=len(rows),
                  paired_histories=len(states), matrix_comparisons=len(errors),
                  state_maxima=dict(zip(['position', 'orientation_rad', 'velocity', 'bias', 'covariance'], maxima)),
                  matrix_max_abs=max(e['max_abs'] for e in errors), workloads=results,
                  gpu_median_p95_gate=all(results['64']['ratios'][k] <= .8 for k in ['median', 'p95']),
                  wall_seconds=commands[0]['seconds'], command=commands[0]['command'],
                  evidence_hashes={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in root.iterdir() if p.is_file()},
                  caveats=['CPU fallback acceptance is interpreted against all six paired rounds; no new threshold invented.',
                           'All40 steps, including initial prior-only step; percentile linear interpolation.',
                           'Times are solver.update scope, not complete source latency; printed log precision retained.',
                           'No whole-system or live-throughput inference.'])
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ['evidence_hashes', 'command', 'caveats']}, indent=2))

if __name__ == '__main__':
    main()
