"""Saved CP-V49 time/pose comparisons and logged factor-ownership ledger.

No estimator, optimizer, covariance analysis, interpolation or accuracy scoring.
"""
import csv
import json
import sys
from collections import Counter
import numpy as np
sys.dont_write_bytecode = True
import analyze as a

assert not (a.H/'chronology.json').exists()
a.pin(__file__)
a.pin(a.__file__)
shape = json.loads(a.pin(a.H/'window_events.json').read_text())
edges = json.loads(a.pin(a.H/'edge_events.json').read_text())
windows = {}
stamps = {}
cost = {}
stdout = {}
for mode in ('off', 'on'):
    root = a.P/f'full_{mode}_source'
    windows[mode] = a.parse_source(root/'estimators.log')
    with a.pin(root/'kimera_odometry.csv').open() as f:
        stamps[mode] = [int(r['field.header.stamp']) for r in csv.DictReader(f)
                        if r.get('field.header.seq', '').isdigit()]
    cost[mode] = np.loadtxt(a.pin(root/'objective.csv'), delimiter=',')
    logs = [p for p in (root/'ros_logs').glob('*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log')
            if p.parent.name != 'latest']
    assert len(logs) == 1
    stdout[mode] = a.parse_stdout(logs[0])

def timing(mode, k):
    ns = stamps[mode][k]
    return dict(k=k, stamp_ns=ns, capture_time_s=(ns-a.EPOCH)*1e-9,
                first_native_time_s=(ns-stamps[mode][0])*1e-9,
                original_bag_time_s=(ns-a.EPOCH)*1e-9+5.7)

# Exact common state IDs and native physical timestamps; no alignment of the two runs.
comparisons = []
for k in range(1, 36):
    off = windows['off']['windows'].get(k, {})
    on = windows['on']['windows'].get(k, {})
    assert off and set(off) == set(on) and stamps['off'][k] == stamps['on'][k]
    assert all(stamps['off'][i] == stamps['on'][i] for i in off)
    comparisons.append(dict(**timing('on', k), same_pose_values=all(np.array_equal(off[i], on[i]) for i in off),
        active_poses=len(off), max_position_difference_m=max(a.diff(off[i], on[i])['translation_m'] for i in off),
        max_rotation_difference_deg=max(a.diff(off[i], on[i])['rotation_deg'] for i in off),
        off_source_line=windows['off']['lines'][k], on_source_line=windows['on']['lines'][k]))

# Reconstruct only LOGGED ownership. This is not an independently exported iSAM2 graph.
ledger = []
actions = stdout['on']['actions']
by_update = {}
for action in actions:
    assert action['k'] is not None
    by_update.setdefault(action['k'], []).append(action)
active = {}
slots = {}
counts = Counter()
ever = set()
for k, life in stdout['on']['life'].items():
    assert life['active_factors'] == len(active), (k, life, active)
    start = len(active)
    staged = set()
    inserted = []
    removed = []
    for x in by_update.get(k, []):
        assert x['line'] > life['line'], (k, x)
        action = x['action']; edge = x['edge']
        detail = dict(item.split('=', 1) for item in x['details'])
        counts[action] += 1
        if action == 'active_window_stage_insert':
            assert edge not in active and edge not in staged and edge not in ever
            staged.add(edge)
            assert int(detail['total_active']) == len(active)
            assert int(detail['total_staged']) == len(staged)
        elif action == 'active_window_remove_scheduled':
            slot = int(detail['slot'])
            assert active[edge] == slot and slots[slot] == edge
            assert detail['slot_valid'] == 'true'
            i,j = (int(s[1:]) for s in edge.split('->'))
            assert min(i,j) < k-life['lag'], (k, edge, life)
            del active[edge]; del slots[slot]
            assert int(detail['total_active']) == len(active)
            removed.append(dict(edge=edge, slot=slot, line=x['line']))
        elif action == 'active_window_slot_tracked':
            slot = int(detail['slot'])
            assert edge in staged and edge not in active and slot not in slots
            staged.remove(edge); ever.add(edge)
            active[edge] = slot; slots[slot] = edge
            assert int(detail['total_active']) == len(active)
            inserted.append(dict(edge=edge, slot=slot, line=x['line']))
        else:
            assert action == 'active_window_reelim_keys', x
            assert detail['force_relinearize'] == detail['force_full_solve'] == 'true'
    assert not staged and len(removed) == life['retiring_factors']
    expected = {f"x{e['receiver'][0]}->x{e['receiver'][1]}" for e in edges if e['k'] == k}
    assert expected == {e['edge'] for e in inserted}, (k, expected, inserted)
    assert len(active) == start+len(inserted)-len(removed)
    ledger.append(dict(**timing('on', k), **life, before=start, after=len(active),
                       inserted=inserted, removed=removed))

shape_map = {(e['mode'], e['k']):e for e in shape}
def row(k):
    out = dict(**timing('on', k), exact_matching_off_timestamp=stamps['off'][k] == stamps['on'][k],
               admissions=[e for e in edges if e['k'] == k], ledger=ledger[k])
    for mode in ('off','on'):
        out[mode] = dict(**timing(mode,k), weighted_imu_cost=float(cost[mode][k,7]),
                        total_before=float(cost[mode][k,4]), total_after=float(cost[mode][k,5]),
                        window_change=shape_map.get((mode,k)))
    return out

thresholds = {}
for field, limits in [('previous_revision_m', [.1,.5,1.0]), ('rigid_tilt_deg', [1,2,3])]:
    for threshold in limits:
        for mode in ('off','on'):
            e = next((e for e in shape if e['mode'] == mode and e[field] > threshold), None)
            thresholds[f'{mode}:{field}>{threshold}'] = None if e is None else dict(**timing(mode,e['k']), event=e)
first = next(x for x in comparisons if not x['same_pose_values'])
selected = sorted(set([6,9,21,24,25,26,30] + [e['k'] for e in thresholds.values() if e and e['k'] < len(stamps['off'])]))
assert all(a.sha(p) == v for p,v in a.PINS.items())
result = dict(scope='Saved geometry and logged ownership only; no covariance or optimizer',
    time_origin='Selected CP-V49 capture start; add 5.7s for original bag start. Native first pose is ~0.2s after capture start.',
    first_difference=first, first_admission=edges[0], early_pose_comparisons=comparisons,
    first_threshold_crossings=thresholds, early=[row(k) for k in range(1,36)], selected=[row(k) for k in selected],
    ledger=dict(updates=len(ledger), inserts=len(ever), removals=counts['active_window_remove_scheduled'],
                remaining=len(active), counts=dict(counts), all_logged_slots_unique_and_valid=True,
                endpoints_older_than_cutoff_on_removal=True, rows=ledger),
    input_sha256=a.PINS, inputs_unchanged=True,
    limits=['Pose comparisons begin at k1: revision-edge log needs two poses.',
            'Threshold crossings are descriptive, not proof of the first software bug.',
            'Factor ledger verifies logged slots/edges/counts, not complete live graph/cache equations.',
            'Final remaining factors belong to the last active window, not a claim of leaked process memory.'])
a.save('chronology.json', result)
print('FIRST_DIFFERENCE', json.dumps(first))
print('LEDGER', json.dumps({k:v for k,v in result['ledger'].items() if k != 'rows'}))
for name,e in thresholds.items():
    print(name, None if e is None else {k:v for k,v in e.items() if k != 'event'})
for k in [6,9,21,24,25,26,30]:
    r=row(k)
    print(k, r['capture_time_s'], [(m,r[m]['weighted_imu_cost'], (r[m]['window_change'] or {}).get('rigid_tilt_deg')) for m in ['off','on']],
          'factors',r['ledger']['before'],r['ledger']['after'])
