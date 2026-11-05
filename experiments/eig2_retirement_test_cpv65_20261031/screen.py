"""Read-only CP-V49 lifecycle/Method-1 screen; writes exclusive CP-V65 results.

No sources, graph solvers, covariance analysis, trajectory scoring or smoothing.
The retired factor's causal effect cannot be recovered from poses alone.
"""
import csv
import datetime
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.dont_write_bytecode = True
H = Path(__file__).resolve().parent
R = H.parents[1]
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
H50 = R / 'experiments/eig2_hesai_onset_cpv50_20261030'
P49 = W / 'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared'
P51 = W / 'runs/eig2_hesai_insertion_cpv51_20261030/prepared'
EPOCH = 1730638816563546034


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(name, value):
    with (H / name).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def decomposition(old_previous, new_previous, new_current):
    correction = new_previous[:3, 3] - old_previous[:3, 3]
    local = new_current[:3, 3] - new_previous[:3, 3]
    published = new_current[:3, 3] - old_previous[:3, 3]
    rc = old_previous[:3, :3].T @ new_previous[:3, :3]
    rm = new_previous[:3, :3].T @ new_current[:3, :3]
    rp = old_previous[:3, :3].T @ new_current[:3, :3]
    translation_error = float(np.linalg.norm(published - correction - local))
    rotation_error = float(np.linalg.norm(rp - rc @ rm))
    assert translation_error < 1e-10 and rotation_error < 1e-10
    return dict(correction_world_m=correction.tolist(),
                within_solution_world_m=local.tolist(), published_world_m=published.tolist(),
                correction_m=float(np.linalg.norm(correction)),
                within_solution_m=float(np.linalg.norm(local)),
                published_m=float(np.linalg.norm(published)),
                correction_rotation_deg=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(rc).as_rotvec()))),
                within_solution_rotation_deg=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(rm).as_rotvec()))),
                published_rotation_deg=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(rp).as_rotvec()))),
                translation_identity_error_m=translation_error,
                rotation_composition_error=rotation_error)


def cosine(a, b):
    a, b = np.asarray(a), np.asarray(b)
    denominator = np.linalg.norm(a) * np.linalg.norm(b)
    if min(np.linalg.norm(a), np.linalg.norm(b)) <= 1e-9:
        return None
    return float(np.clip(np.dot(a, b) / denominator, -1.0, 1.0))


def stats(values):
    a = np.asarray(values, float)
    assert np.isfinite(a).all()
    if not a.size:
        return dict(n=0)
    return dict(n=len(a), minimum=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()), sum=float(a.sum()))


def summarize(rows):
    out = dict(n=len(rows))
    for field in ('correction_m', 'within_solution_m', 'published_m', 'correction_rotation_deg'):
        out[field] = stats([r[field] for r in rows])
    for field in ('correction_direction_cosine', 'published_direction_cosine', 'within_solution_direction_cosine'):
        eligible = [r[field] for r in rows if r[field] is not None]
        out[field] = dict(eligible=len(eligible), negative_dot_count=sum(c < 0 for c in eligible))
    out['correction_over_0_5m'] = sum(r['correction_m'] > .5 for r in rows)
    out['with_removal'] = sum(r['removals'] > 0 for r in rows)
    out['with_insertion'] = sum(r['insertions'] > 0 for r in rows)
    out['with_both'] = sum(r['removals'] > 0 and r['insertions'] > 0 for r in rows)
    return out


def reconcile(parsed, old_rows):
    """Independent edge-slot ledger, cross-checked against CP-V50, no solver claim."""
    actions = {}
    for action in parsed['actions']:
        actions.setdefault(action['k'], []).append(action)
    active, occupied, seen, lifetimes, rows = {}, {}, set(), [], []
    reference = {r['k']: r for r in old_rows}
    for k, life in sorted(parsed['life'].items()):
        assert k in reference and life['active_factors'] == len(active)
        before = len(active)
        pending, added, removed = set(), [], []
        for action in actions.get(k, []):
            edge, kind = action['edge'], action['action']
            detail = dict(item.split('=', 1) for item in action['details'])
            if kind == 'active_window_stage_insert':
                assert edge not in active and edge not in pending and edge not in seen
                pending.add(edge)
            elif kind == 'active_window_remove_scheduled':
                slot = int(detail['slot'])
                assert edge in active and active[edge]['slot'] == slot and occupied[slot] == edge
                assert detail['slot_valid'] == 'true'
                first, last = (int(x[1:]) for x in edge.split('->'))
                assert min(first, last) < k - life['lag']
                record = active.pop(edge)
                del occupied[slot]
                lifetimes.append(dict(edge=edge, slot=slot, insert_k=record['insert_k'], remove_k=k,
                                      lifetime_s=reference[k]['capture_time_s'] - reference[record['insert_k']]['capture_time_s']))
                removed.append(dict(edge=edge, slot=slot))
            elif kind == 'active_window_slot_tracked':
                slot = int(detail['slot'])
                assert edge in pending and slot not in occupied and edge not in active
                pending.remove(edge)
                seen.add(edge)
                active[edge] = dict(slot=slot, insert_k=k)
                occupied[slot] = edge
                added.append(dict(edge=edge, slot=slot))
            else:
                assert kind == 'active_window_reelim_keys'
        assert not pending and len(removed) == life['retiring_factors']
        assert len(active) == before + len(added) - len(removed)
        old = reference[k]
        assert (before, len(active)) == (old['before'], old['after'])
        for key, actual in [('inserted', added), ('removed', removed)]:
            assert actual == [dict(edge=x['edge'], slot=x['slot']) for x in old[key]]
        rows.append(dict(k=k, capture_time_s=old['capture_time_s'], before=before, after=len(active),
                         inserted=added, removed=removed, marginal_keys=life['marginal_keys']))
    assert len(rows) == len(reference)
    return rows, lifetimes


def main():
    for name in ('analysis.json', 'event_manifest.json'):
        assert not (H / name).exists(), 'refuse result overwrite'
    spec = importlib.util.spec_from_file_location('cpv50_readonly', H50 / 'analyze.py')
    source = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(source)  # main guarded in the original parser.
    chronology_path = source.pin(H50 / 'chronology.json')
    chronology = json.loads(chronology_path.read_text())
    shape_path = source.pin(H50 / 'window_events.json')
    shape = json.loads(shape_path.read_text())
    expected = chronology['input_sha256']
    for path in (H / 'PROTOCOL.txt', Path(__file__), H / 'test_screen.py', H50 / 'analyze.py'):
        source.pin(path)
    # Verify legacy evidence hashes before calling the imported read-only parsers.
    for path in (shape_path, H50 / 'analyze.py'):
        assert sha(path) == expected[str(path)], path
    data, stamps = {}, {}
    for mode in ('off', 'on'):
        root = P49 / f'full_{mode}_source'
        log = root / 'estimators.log'
        assert sha(log) == expected[str(log)]
        data[mode] = source.parse_source(log)
        trajectory = source.pin(root / 'kimera_odometry.csv')
        assert sha(trajectory) == expected[str(trajectory)]
        with trajectory.open() as stream:
            stamps[mode] = [int(r['field.header.stamp']) for r in csv.DictReader(stream)
                            if r.get('field.header.seq', '').isdigit()]
    root = P49 / 'full_on_source/ros_logs'
    stdout = [p for p in root.glob('*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log') if p.parent.name != 'latest']
    assert len(stdout) == 1 and sha(stdout[0]) == expected[str(stdout[0])]
    native = source.parse_stdout(stdout[0])
    assert not native['failures']
    ledger, lifetimes = reconcile(native, chronology['ledger']['rows'])
    by_k = {r['k']: r for r in ledger}
    first_insert = next(r['k'] for r in ledger if r['inserted'])
    first_remove = next(r['k'] for r in ledger if r['removed'])
    assert first_insert == 6 and first_remove == 30
    shapes = {(r['mode'], r['k']): r for r in shape}
    rows, omitted = [], []
    for mode in ('off', 'on'):
        windows = data[mode]['windows']
        previous_row = None
        for k in range(1, len(stamps[mode])):
            old, new = windows.get(k-1, {}), windows.get(k, {})
            if k-1 not in old or k-1 not in new or k not in new:
                omitted.append(dict(mode=mode, k=k, reason='missing required overlapping saved poses'))
                previous_row = None
                continue
            d = decomposition(old[k-1], new[k-1], new[k])
            legacy = shapes.get((mode, k))
            if legacy is not None:
                assert abs(d['correction_m'] - legacy['previous_revision_m']) < 1e-10
            life = by_k[k] if mode == 'on' else None
            row = dict(mode=mode, k=k, stamp_ns=stamps[mode][k],
                       capture_time_s=(stamps[mode][k] - EPOCH)*1e-9,
                       stage='before_first_peer' if k < first_insert else
                             'peers_no_retirement' if k < first_remove else 'retirement_era',
                       removals=len(life['removed']) if life else 0,
                       insertions=len(life['inserted']) if life else 0,
                       active_peer_count=life['after'] if life else 0, **d)
            for name, vector in [('correction', 'correction_world_m'),
                                 ('published', 'published_world_m'),
                                 ('within_solution', 'within_solution_world_m')]:
                row[name + '_direction_cosine'] = (cosine(previous_row[vector], row[vector])
                                                   if previous_row is not None else None)
            rows.append(row)
            previous_row = row
    summary = {}
    for mode in ('off', 'on'):
        for stage in ('all', 'before_first_peer', 'peers_no_retirement', 'retirement_era'):
            summary[mode + ':' + stage] = summarize([r for r in rows if r['mode'] == mode and
                                                    (stage == 'all' or r['stage'] == stage)])
    on = [r for r in rows if r['mode'] == 'on']
    large = [r for r in on if r['correction_m'] > .5]
    causal_keys = [28, 29] + [r['k'] for r in ledger if r['removed']][:6]
    stages = ('before_ordinary', 'after_ordinary', 'after_refresh', 'after_marginalization')
    required = []
    for k in causal_keys:
        missing = []
        for stage in stages:
            path = P51 / f'full_on_source/checkpoints/update_{k}' / stage / 'checkpoint.bin'
            if not path.is_file():
                missing.append(str(path))
        required.append(dict(k=k, ledger=by_k[k], missing_native_snapshots=missing,
                             available_pose_screen=next((r for r in on if r['k'] == k), None)))
    checkpoint_source = W / 'runs/eig2_hesai_insertion_cpv51_20261030/Checkpoint.h'
    text = source.pin(checkpoint_source).read_text()
    assert 'k<6 || k>25' in text and 'dynamic_cast<const OriginalChartPrior*>' not in text
    snapshot_dirs = P51 / 'full_on_source/checkpoints'
    available = sorted(int(p.name.split('_')[1]) for p in snapshot_dirs.glob('update_*') if p.is_dir())
    assert available == list(range(6, 26))
    event_manifest = dict(status='NATIVE_CAUSAL_STAGE_NOT_READY_NOT_RUN',
                          selected_early_updates=causal_keys, first_retirement_k=first_remove,
                          selection='two immediately preceding controls plus first six retirements',
                          events=required, existing_native_update_ids=available,
                          diagnostic_serializer_requires_original_chart_prior_support=True,
                          source_capture_launched=False, native_optimizer_calls=0,
                          later_followup_events_not_in_10s_prefix=[large[0], max(on, key=lambda r: r['correction_m'])])
    after_admission = [r for r in ledger if r['k'] >= first_insert]
    lifecycle = dict(updates=len(ledger), inserted=sum(len(r['inserted']) for r in ledger),
                     removed=len(lifetimes), remaining=ledger[-1]['after'],
                     active_after_first_admission=stats([r['after'] for r in after_admission]),
                     zero_active_updates_after_first_admission=[r['k'] for r in after_admission if r['after'] == 0],
                     simultaneous_insert_remove_updates=sum(bool(r['inserted']) and bool(r['removed']) for r in ledger),
                     removal_updates=sum(bool(r['removed']) for r in ledger),
                     lifetime_s=stats([r['lifetime_s'] for r in lifetimes]),
                     first_retirement=by_k[first_remove], lifetimes=lifetimes)
    result = dict(status='SAVED_DATA_SCREEN_COMPLETE_CAUSAL_TEST_NOT_RUN',
                  finished=datetime.datetime.now().astimezone().isoformat(),
                  lifecycle=lifecycle, summary=summary, rows=rows, omissions=omitted,
                  complete_decomposition_translation_max=max(r['translation_identity_error_m'] for r in rows),
                  complete_decomposition_rotation_max=max(r['rotation_composition_error'] for r in rows),
                  limits=['No cause can be inferred from mostly co-occurring admission/removal/new native data.',
                          'Within-solve increments are not ground truth or a new full trajectory.',
                          'Absolute-output correction can be legitimate; signs/magnitudes do not certify accuracy.',
                          'No bias values or intermediate retirement-only solve are reconstructed from pose logs.',
                          'No native causal branch, estimator source replay or EVO metric was run.'],
                  input_sha256=source.PINS.copy())
    for path, expected_hash in source.PINS.items():
        assert sha(path) == expected_hash, path
    result['inputs_unchanged'] = True
    save('event_manifest.json', event_manifest)
    save('analysis.json', result)
    print(json.dumps(dict(status=result['status'], lifecycle={k:v for k,v in lifecycle.items() if k != 'lifetimes'},
                          summary=summary, omissions=len(omitted), causal_ready=False), indent=2))


if __name__ == '__main__':
    main()
