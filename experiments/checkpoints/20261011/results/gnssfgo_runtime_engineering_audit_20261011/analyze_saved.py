"""Read retained artifacts only. No estimator, optimizer, or replay is run."""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
RUN = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_vw_jacobian_live160_20261011')
P = RUN / 'prepared'
S = P / 'full160_on_source'
R = P / 'full160_on_receiver/receiver'
spec = importlib.util.spec_from_file_location('timing', RUN.parent / 'glim_cbs_diagnostic_budget_20261010/compare_timing.py')
timing = importlib.util.module_from_spec(spec)
spec.loader.exec_module(timing)
d = timing.read(S)
read = lambda p: list(csv.DictReader(p.open()))
conv = read(R / 'publication_convergence.csv')
steps = read(R / 'publication_steps.csv')
rev = read(R / 'revision_events.csv')
by_index = {int(r['index']): r for r in conv}
elapsed = lambda r: (int(r['physical_ns']) - 1727775008003475000) * 1e-9
stats = timing.stats
windows = []
for lo, hi in [(0, 20), (20, 40), (40, 60), (60, 80), (80, 100), (100, 120), (120, 130), (130, 144), (0, 144)]:
    ids = [i for i, r in d['scan'].items() if lo <= r['stamp'] - timing.T0 < hi]
    scans = [d['scan'][i] for i in ids]
    c = [r for r in conv if lo <= elapsed(r) < hi]
    st = [r for r in steps if int(r['index']) in by_index and lo <= elapsed(by_index[int(r['index'])]) < hi]
    rv = [r for r in rev if lo <= (int(r['record_timestamp_ns']) - 1727775008003475000) * 1e-9 < hi]
    nested = {}
    for kind, cols in {
        'on_smoother_update': {'sidecar_update': 10},
        'publish_outgoing': {'local_covariance_rebuild': 17, 'belief_query': 18, 'ros_publish': 20},
        'on_smoother_update_finish': {'outgoing_plus_revisions': 12}
    }.items():
        rows = [d['cbs'][kind][i][0] for i in ids if i in d['cbs'].get(kind, {})]
        nested.update({k: stats([r[j] for r in rows]) for k, j in cols.items()})
    total_ms = sum(r['total'] for r in scans)
    windows.append(dict(seconds=[lo, hi], scan_ms={k: stats([r[k] for r in scans]) for k in
        ['total', 'pre_callback', 'smoother', 'post_callback', 'marginalize', 'deskew', 'pointcov', 'createfactors', 'updateframes']},
        scan_counts={k: stats([r[k] for r in scans]) for k in ['points', 'nimu', 'nfactors', 'active', 'marginalized']},
        nested_ms=nested,
        glim_main_stage_fraction={k: sum(r[k] for r in scans)/total_ms for k in ['pre_callback','smoother','post_callback','marginalize']},
        refinement_seconds=stats([float(r['seconds']) for r in c]),
        refinement_seconds_sum=sum(float(r['seconds']) for r in c),
        refinement_attempts=stats([int(r['attempted']) for r in c]),
        recovery_rows=sum(int(r['recovery']) for r in st),
        recovery_publications=len({r['index'] for r in st if int(r['recovery'])}),
        local_prediction_rows=sum(int(r['local_prediction']) for r in st),
        refinement_rows=len(st),
        relinearized=stats([int(r['relinearized']) for r in st if not int(r['recovery'])]),
        revision_updates=len(rv), revision_update_seconds_sum=sum(float(r['update_seconds']) for r in rv)))

sizes = {str(p): p.stat().st_size for p in [S/'estimators.log', Path(d['summary']['path']),
    P/'full160_on_receiver/events.csv', P/'full160_on_receiver/receipts.jsonl']}
result = dict(scope='Read-only analysis of existing partial 143.79-second capture; no new estimator or replay.',
    run=str(RUN), windows=windows, source_summary=d['summary'], file_bytes=sizes,
    publication_rows=len(conv), refinement_step_rows=len(steps), revision_rows=len(rev),
    largest_publications=sorted(conv, key=lambda r: float(r['seconds']), reverse=True)[:10],
    graph_summary=json.loads((R/'summary.json').read_text()))
with (ROOT/'saved_timing_analysis.json').open('x') as f:
    json.dump(result, f, indent=2)
print('Saved timing analysis; all durations below are retained wall seconds.')
for w in windows:
    g=w['scan_ms']; n=w['nested_ms']; sc=w['scan_counts']
    print(w['seconds'], 'GLIM mean', round(g['total']['mean']/1000,4),
          'sidecar/local/revision-total', [round(n[k]['mean']/1000,4) for k in ['sidecar_update','local_covariance_rebuild','outgoing_plus_revisions']],
          'refinement mean', round(w['refinement_seconds']['mean'],4),
          'recovery pubs', w['recovery_publications'], 'active scans', sc['active']['max'],
          'marginalize mean ms', round(g['marginalize']['mean'],4))
print('Source summary:', json.dumps(d['summary']))
print('File sizes:', json.dumps(sizes))
print('All-window totals:', json.dumps({k:v for k,v in windows[-1].items() if k not in ['scan_ms','scan_counts','nested_ms']}))
