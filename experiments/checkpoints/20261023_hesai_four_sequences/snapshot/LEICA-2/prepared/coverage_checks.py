"""CP-K22: distinguish sparse native keyframes from a truncated capture."""
import csv
import datetime
import json
import re
import sys
from pathlib import Path
import numpy as np

KEYFRAME_LIMIT = 5.0
FRAME_PERIOD = .1
GAP_LIMIT = KEYFRAME_LIMIT + FRAME_PERIOD + .05


def check(t, p, duration, first_imu, last_imu, role):
    t = np.asarray(t); p = np.asarray(p)
    assert len(t) > 10 and p.shape == (len(t), 3), (role, 'missing poses')
    assert np.isfinite(t).all() and np.isfinite(p).all(), role
    gaps = np.diff(t)
    assert (gaps > 0).all(), (role, 'unordered stamps')
    assert last_imu-first_imu >= duration-.3, 'IMU stream truncated'
    span = float(t[-1]-t[0]); tail = float(last_imu-t[-1]); start = float(t[0]-first_imu)
    later = gaps[t[:-1] >= first_imu+5]
    if role == 'native_kimera':
        assert -.1 <= start <= 1.5 and -.1 <= tail <= GAP_LIMIT, (role, 'start/tail', start, tail)
        assert gaps.max() <= GAP_LIMIT, (role, 'keyframe gap', gaps.max())
    else:
        assert role == 'native_glim'
        assert span >= duration-2, (role, 'truncated coverage', span, duration)
        assert -.1 <= tail <= .5 and -.1 <= start <= 1.5, (role, 'start/tail', start, tail)
        assert not len(later) or later.max() <= 1, (role, 'post-startup gap', later.max())
    return dict(rows=len(t), start_s=float(t[0]), end_s=float(t[-1]), span_s=span,
        maximum_pose_gap_s=float(gaps.max()), post_startup_max_gap_s=float(later.max()) if len(later) else None,
        start_after_first_imu_s=start, tail_behind_last_imu_s=tail,
        path_length_m=float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
        gaps_over_1s=[dict(before_index=int(i), after_index=int(i+1), start_s=float(t[i]),
                          end_s=float(t[i+1]), gap_s=float(gaps[i])) for i in np.flatnonzero(gaps > 1)],
        path_note='Polyline through actual samples, including gap chords; no dense coverage claim.')


def load(path, kimera=False):
    if kimera:
        with path.open() as f: rows = list(csv.DictReader(f))
        assert [int(r['field.header.seq']) for r in rows] == list(range(len(rows))), 'native output ID loss'
        return np.array([int(r['field.header.stamp'])*1e-9 for r in rows]), np.array(
            [[float(r['field.pose.pose.position.'+c]) for c in 'xyz'] for r in rows])
    a = np.loadtxt(path, ndmin=2)
    return a[:, 0], a[:, 1:4]


def processing(logs, t):
    """Use already-enabled native counters; no new observer in estimator path."""
    counters = []; updates = {}; low = []
    prefix = re.compile(r'^[IWEF](\d{4}) (\d\d:\d\d:\d\d\.\d+)')
    for path in logs:
        wall = None
        for raw in path.open(errors='strict'):
            line = re.sub(r'\x1b\[[0-9;]*m', '', raw)
            m = prefix.match(line)
            if m:
                wall = datetime.datetime.strptime('2026'+m[1]+' '+m[2], '%Y%m%d %H:%M:%S.%f').replace(tzinfo=datetime.timezone.utc).timestamp()
            if 'KIMERA_BACKEND_SPINONCE_TIMING_ROW,' in line:
                f = line.split('KIMERA_BACKEND_SPINONCE_TIMING_ROW,', 1)[1].strip().split(',')
                assert wall is not None
                k = int(f[0]); value = (wall, float(f[1]))
                assert k not in updates or updates[k] == value, ('conflicting timing ID', k)
                updates[k] = value
            fields = line.strip().split('\t')
            if len(fields) >= 6 and fields[0].strip() in ('VioFrontend [ms]', 'VioBackend [ms]'):
                assert wall is not None and fields[1].strip().isdigit()
                counters.append((wall, fields[0].strip(), int(fields[1])))
            if 'Low mono disparity' in line or 'Low disparity: adding' in line:
                low.append(wall)
    assert sorted(updates) == list(range(len(t))), ('backend/output mismatch', len(updates), len(t))
    frontend = sorted(set((w, n) for w, name, n in counters if name == 'VioFrontend [ms]'))
    backend = sorted(set((w, n) for w, name, n in counters if name == 'VioBackend [ms]'))
    assert frontend and backend
    assert all(b[1] >= a[1] for a, b in zip(frontend, frontend[1:])), 'frontend counter regressed'
    assert 4284 <= frontend[-1][1] <= 4287, ('full-window frontend incomplete', frontend[-1])
    assert backend[-1][1] == len(t), ('backend final count', backend[-1], len(t))
    evidence = []
    for i in np.flatnonzero(np.diff(t) > 1):
        lo, hi = updates[int(i)][0], updates[int(i+1)][0]
        samples = [(w, n) for w, n in frontend if lo <= w <= hi]
        assert len(samples) >= 2 and samples[-1][1] > samples[0][1], ('no frontend progress in keyframe gap', int(i))
        evidence.append(dict(before_id=int(i), after_id=int(i+1), gap_s=float(t[i+1]-t[i]),
            wall_interval_s=hi-lo, frontend_counter_first=samples[0][1], frontend_counter_last=samples[-1][1],
            backend_before_ms=updates[int(i)][1], backend_after_ms=updates[int(i+1)][1],
            low_disparity_lines=sum(w is not None and lo-.3 <= w <= hi+.3 for w in low)))
    return dict(passed=True, logs=[str(p) for p in logs], frontend_processed=frontend[-1][1],
        synchronized_input_pairs_reference=4287, frontend_reference_after_startup=4284,
        backend_processed=backend[-1][1], sequential_native_ids=True, gaps=evidence,
        scope='Native counters, not a new per-image receipt or branch-decision audit.')


def main():
    source = Path(sys.argv[1]); duration = float(sys.argv[2]); journal = Path(sys.argv[3])
    assert duration == 428.75, 'This coverage contract is for the frozen full LEICA-2 window'
    profile = (source/'kimera_profile/FrontendParams.yaml').read_text()
    assert float(re.search(r'^max_intra_keyframe_time:\s*(\S+)', profile, re.M)[1]) == KEYFRAME_LIMIT
    imu = []
    with journal.open() as f:
        for line in f:
            v = line.strip().split(',')
            if v[0] == 'I': imu.append(int(v[2])*1e-9)
    assert len(imu) > 100 and np.all(np.diff(imu) > 0)
    result = dict(passed=True, requested_s=duration, imu_samples=len(imu), imu_span_s=imu[-1]-imu[0],
                  coverage_contract='native_keyframes_cpk22', kimera_gap_bound_s=GAP_LIMIT)
    for role, filename in [('native_glim', 'odom_imu.txt'), ('native_kimera', 'kimera_odometry.csv')]:
        result[role] = check(*load(source/filename, role == 'native_kimera'), duration, imu[0], imu[-1], role)
    logs = [source/'estimators.log', *source.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]
    result['kimera_processing'] = processing(logs, load(source/'kimera_odometry.csv', True)[0])
    with (source/'source_coverage.json').open('x') as f: json.dump(result, f, indent=2)
    print(json.dumps(result))


if __name__ == '__main__': main()
