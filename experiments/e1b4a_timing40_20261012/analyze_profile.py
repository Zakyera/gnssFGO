"""Validate and summarize the completed E1b.3c opt-in timing profile.

All timings are host wall durations. Nested inclusive stages are never added.
"""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


def summary(values):
    values = sorted(values)
    assert values and all(math.isfinite(value) and value >= 0 for value in values)
    def percentile(q):
        position = (len(values) - 1) * q
        lower = int(position)
        upper = min(lower + 1, len(values) - 1)
        return values[lower] + (values[upper] - values[lower]) * (position - lower)
    return dict(count=len(values), mean=statistics.fmean(values),
                median=percentile(.5), p95=percentile(.95), max=values[-1])


def legacy_timers(path):
    scans = {}
    local = {}
    for line in path.open():
        if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
            fields = re.split('[\x1b\r\n]', line.split('GLIM_ODOM_IMU_TIMING_ROW,', 1)[1])[0].split(',')
            assert len(fields) == 25 and fields[24] == 'processed'
            frame = int(fields[1])
            assert frame not in scans
            scans[frame] = dict(timestamp=float(fields[0]), total_ms=float(fields[23]),
                                main_ms=float(fields[17]), pre_ms=float(fields[16]))
        if 'GLIM_CBS_TIMING_ROW,on_smoother_update,' in line:
            fields = re.split('[\x1b\r\n]', line.split('GLIM_CBS_TIMING_ROW,', 1)[1])[0].split(',')
            frame = int(fields[1])
            assert frame not in local
            local[frame] = float(fields[11])
    # The callback's smoother-update counter is 0-based; processed scan frames
    # are 1-based. Both have one row per completed scan in chronological order.
    assert scans and len(scans) == len(local)
    assert sorted(scans) == list(range(1, len(scans) + 1))
    assert sorted(local) == list(range(len(local)))
    return scans, local


def analyze(root):
    prepared = root / 'prepared'
    source = prepared / 'profile40_on_source'
    validation = json.loads((prepared / 'profile40_on_validation.json').read_text())
    assert validation['passed']
    profile = source / 'e1b_profile.jsonl'
    data = profile.read_bytes()
    assert data.endswith(b'\n'), 'truncated last row'
    records = [json.loads(line) for line in data.splitlines()]
    assert records
    by_context = defaultdict(list)
    stage_exclusive = Counter()
    stage_calls = Counter()
    for row in records:
        assert row['schema'] == 1 and row['context'] in range(5)
        assert math.isfinite(row['timestamp']) and not row['aborted']
        assert len(row['metrics']) == 9 and all(value >= 0 for value in row['metrics'])
        assert not row['worker_sum'], 'Unexpected worker timings in pinned non-TBB build'
        owner = row['owner_wall']
        assert 'root' in owner and owner['root'][0] == 1
        for name, cell in owner.items():
            assert len(cell) == 4 and all(isinstance(value, int) and value >= 0 for value in cell)
            assert cell[1] >= cell[2] and cell[3] == 0
            if row['context'] == 2:
                stage_exclusive[name] += cell[2]
                stage_calls[name] += cell[0]
        assert sum(cell[2] for cell in owner.values()) == owner['root'][1]
        by_context[row['context']].append(row)
    assert len(by_context[2]) >= 350
    for context in by_context.values():
        timestamps = [row['timestamp'] for row in context]
        assert timestamps == sorted(timestamps)
    local = by_context[2]
    local_ns = sum(row['owner_wall']['root'][1] for row in local)
    assert sum(stage_exclusive.values()) == local_ns
    log, = [path for path in (source / 'ros_logs').glob('*/glim_rosnode-*-stdout.log')
            if path.parent.name != 'latest']
    scans, legacy_local = legacy_timers(log)
    ordered_scans = [scans[key] for key in sorted(scans)]
    ordered_legacy = [legacy_local[key] for key in sorted(legacy_local)]
    # A startup LOCAL update is timestamped just before the first processed scan.
    matched = [row for row in local if any(abs(row['timestamp'] - scan['timestamp']) < 1e-6
                                           for scan in ordered_scans)]
    assert len(matched) == len(ordered_scans) == len(ordered_legacy)
    assert len(local) - len(matched) == 1
    assert max(abs(row['timestamp'] - scan['timestamp'])
               for row, scan in zip(matched, ordered_scans)) < 1e-6
    first_scan = ordered_scans[0]['timestamp']
    def scope_ms(row, name):
        return row['owner_wall'].get(name, [0, 0, 0, 0])[1] / 1e6
    def window(start, end):
        rows = [row for row in matched if start <= row['timestamp'] - first_scan < end]
        sync_calls = sum(row['owner_wall'].get('gpu_sync_linearize', [0])[0] for row in rows)
        sync_ns = sum(row['owner_wall'].get('gpu_sync_linearize', [0, 0])[1] for row in rows)
        return dict(local_root_ms=summary(scope_ms(row, 'root') for row in rows),
                    sync_fallback_ms=summary(scope_ms(row, 'gpu_sync_linearize') for row in rows),
                    sync_fallback_calls=summary(row['owner_wall'].get(
                        'gpu_sync_linearize', [0])[0] for row in rows),
                    sync_fallback_weighted_ms_per_call=sync_ns / sync_calls / 1e6 if sync_calls else None,
                    active_keys=summary(row['metrics'][0] for row in rows),
                    factor_slots=summary(row['metrics'][1] for row in rows),
                    nonnull_factors=summary(row['metrics'][2] for row in rows))
    result = dict(
        passed=True,
        profile_sha256=hashlib.sha256(data).hexdigest(),
        profile_rows=len(records),
        contexts={str(key):dict(rows=len(value), root_wall_s=sum(
            row['owner_wall']['root'][1] for row in value) / 1e9)
            for key, value in sorted(by_context.items())},
        context_gpu={str(key):dict(
            linearize_calls=sum(row['owner_wall'].get('gpu_linearize', [0])[0] for row in value),
            sync_fallback_calls=sum(row['owner_wall'].get('gpu_sync_linearize', [0])[0] for row in value),
            cache_hits=sum(row['metrics'][7] for row in value),
            sync_fallback_exclusive_s=sum(row['owner_wall'].get(
                'gpu_sync_linearize', [0, 0, 0])[2] for row in value) / 1e9)
            for key, value in sorted(by_context.items())},
        local_rows=len(local), matched_scan_rows=len(matched), unmatched_startup_rows=1,
        physical_profile_span_s=local[-1]['timestamp'] - local[0]['timestamp'],
        local_root_wall_s=local_ns / 1e9,
        local_exclusive_stage_s={key: value / 1e9 for key, value in stage_exclusive.most_common()},
        local_exclusive_stage_percent={key: 100 * value / local_ns
                                       for key, value in stage_exclusive.most_common()},
        local_stage_calls=dict(stage_calls),
        gpu_linearization_cache_hits=sum(row['metrics'][7] for row in local),
        gpu_error_cache_hits=sum(row['metrics'][8] for row in local),
        all_local_root_ms=summary(scope_ms(row, 'root') for row in matched),
        all_local_sync_fallback_ms=summary(scope_ms(row, 'gpu_sync_linearize') for row in matched),
        early_0_10s=window(0, 10), late_30_40s=window(30, 40),
        legacy=dict(processed_scans=len(ordered_scans),
                    scan_total_ms=summary(scan['total_ms'] for scan in ordered_scans),
                    local_ms=summary(ordered_legacy),
                    local_wall_sum_s=sum(ordered_legacy) / 1e3,
                    matched_profile_root_sum_s=sum(scope_ms(row, 'root') for row in matched) / 1e3),
        cautions=['Host elapsed time, not GPU kernel execution time',
                  'gpu_sync_linearize includes warning output and derivatives->linearize',
                  'Nested inclusive stages overlap; only owner-exclusive stages partition LOCAL',
                  'This first-40-second profile does not characterize the 120-160-second tail',
                  'Profile JSON serialization/flush is outside the measured root'],
    )
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    output = json.dumps(analyze(args.run_root), indent=2) + '\n'
    if args.out:
        with args.out.open('x') as stream:
            stream.write(output)
    else:
        print(output, end='')
