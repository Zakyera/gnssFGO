#!/usr/bin/env python3
"""Read-only E1b.1 analysis; no estimator imports, launches or altered inputs."""
import argparse
import collections
from decimal import Decimal
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import statistics
import subprocess

BASE_NS = 1727775008003475000
RUN = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011/prepared')
WORKSPACE = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src')
SOURCE = RUN / 'live160_on_source'
STDOUT = SOURCE / 'ros_logs/fd776fca-c59c-11f1-82ac-62f7ac56faf4/glim_rosnode-1-stdout.log'


def fields(line, marker):
    if marker not in line:
        return None
    return re.split('[\x1b\r\n]', line.split(marker, 1)[1])[0].split(',')


def ns(value):
    return int((Decimal(value) * 1000000000).to_integral_value())


def percentile(values, q):
    values = sorted(values)
    pos = q * (len(values)-1)
    i = int(pos)
    return values[i] + (values[min(i+1, len(values)-1)]-values[i]) * (pos-i)


def summary(values):
    values = list(values)
    if not values:
        return {'n': 0}
    if not all(map(math.isfinite, values)):
        raise ValueError('nonfinite metric')
    return dict(n=len(values), min=min(values), mean=statistics.fmean(values),
                median=statistics.median(values), p95=percentile(values, .95),
                max=max(values))


def parse_scan(line):
    f = fields(line, 'GLIM_ODOM_IMU_TIMING_ROW,')
    if f is None:
        return None
    if len(f) != 25 or f[24] != 'processed':
        raise ValueError('scan schema')
    metric = {name: float(f[i]) for name, i in
              [('pre_ms', 16), ('main_ms', 17), ('post_ms', 18), ('total_ms', 23)]}
    if not all(math.isfinite(x) and x >= 0 for x in metric.values()):
        raise ValueError('invalid scan timer')
    return dict(frame=int(f[1]), sec=(ns(f[0])-BASE_NS)/1e9,
                active_frames=int(f[5]), retired_frames=int(f[6]), **metric)


def fingerprint(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return dict(bytes=path.stat().st_size, sha256=h.hexdigest())


def source_git():
    result = {}
    for name in ('cbs', 'glim', 'glim_ros1', 'gtsam', 'gtsam_points', 'Kimera-VIO'):
        path = WORKSPACE / name
        if not path.exists():
            raise ValueError(f'missing source repo: {path}')
        def git(*args):
            return subprocess.check_output(['git', '-C', str(path), *args], text=True)
        result[str(path)] = dict(head=git('rev-parse', 'HEAD').strip(),
                                 status=git('status', '--porcelain=v1'),
                                 tracked_diff_sha256=hashlib.sha256(
                                     git('diff', 'HEAD', '--binary').encode()).hexdigest())
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError(args.out)
    inputs = [STDOUT, SOURCE/'estimators.log', RUN/'live160_on_receiver/events.csv',
              RUN/'live160_on_receiver/receipts.jsonl', SOURCE/'resolved_glim_params.yaml',
              SOURCE/'effective_mode_check.json', SOURCE/'run_metadata.txt']
    protected = {str(p): fingerprint(p) for p in inputs}
    git_before = source_git()
    scans, stages, producers = {}, collections.defaultdict(dict), {}
    lifecycle = []
    for line in STDOUT.open():
        scan = parse_scan(line)
        if scan:
            if scan['frame'] in scans:
                raise ValueError('duplicate scan')
            scans[scan['frame']] = scan
        f = fields(line, 'GLIM_CBS_TIMING_ROW,')
        if f:
            if len(f) < 17:
                raise ValueError('CBS timer schema')
            frame = int(f[1])
            if f[0] in stages[frame]:
                raise ValueError('duplicate stage timer')
            stages[frame][f[0]] = f
        f = fields(line, 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,')
        if f:
            if len(f) != 7 or f[0] != 'g':
                raise ValueError('lifecycle schema')
            lifecycle.append(dict(sec=(ns(f[1])-BASE_NS)/1e9, lag=float(f[2]),
                                  active=int(f[3]), marginalizing=int(f[4]),
                                  external=int(f[5]), removing_external=int(f[6])))
        for marker in ('GLIM_CBS_EXPORT_BELIEF_ROW,', 'GLIM_CBS_REVISION_BELIEF_ROW,'):
            f = fields(line, marker)
            if f:
                if len(f) != 10 or f[0] != 'g':
                    raise ValueError('producer schema')
                key = (int(f[1]), int(f[2]), int(f[3]))
                if key in producers:
                    raise ValueError('duplicate producer version')
                producers[key] = dict(record=ns(f[4]), tj=ns(f[7]))

    missing_stage_frames = []
    for frame, scan in scans.items():
        if not stages.get(frame):
            # The retained final scan has a native timer but no ROS callback
            # timing rows. Keep its total, never invent substage times.
            missing_stage_frames.append(frame)
            continue
        pre = stages[frame]['on_smoother_update']
        scan['local_ms'] = float(pre[11])
        scan['pre_other_ms'] = scan['pre_ms']-scan['local_ms']
        out = stages[frame].get('publish_outgoing')
        if out:
            scan['covariance_prep_ms'] = float(out[18])
            scan['covariance_query_ms'] = float(out[19])
        finish = stages[frame]['on_smoother_update_finish']
        scan['outgoing_and_revisions_ms'] = float(finish[13])
        scan['remaining_scan_ms'] = scan['total_ms'] - scan['pre_ms'] - scan['main_ms'] - scan['post_ms']
        if min(scan['pre_other_ms'], scan['remaining_scan_ms']) < -.01:
            raise ValueError('nested timer mismatch')

    latency = []
    seen = set()
    with (RUN/'live160_on_receiver/events.csv').open() as events, \
            (RUN/'live160_on_receiver/receipts.jsonl').open() as receipts:
        for index, pair in enumerate(itertools.zip_longest(events, receipts)):
            event, receipt = pair
            if event is None or receipt is None:
                raise ValueError('event/receipt length mismatch')
            f, r = event.strip().split(','), json.loads(receipt)
            if index != r['event'] or f[0] != r['kind'] or int(f[1]) != r['receipt_ros_ns']:
                raise ValueError('event/receipt identity mismatch')
            if f[0] != 'B' or f[2] != 'glim_hesai':
                continue
            meta = r['metadata']
            key = int(f[6]), int(f[7]), int(meta['source_revision'])
            if key in seen:
                raise ValueError('duplicate receiver source version')
            seen.add(key)
            producer = producers[key]
            if producer['record'] != ns(meta['source_record']) or producer['tj'] != int(f[9]):
                raise ValueError('producer/transport mismatch')
            age = (producer['record']-producer['tj']) / 1e9
            transport = (int(f[1])-producer['record']) / 1e9
            # ROS nanoseconds vs producer seconds printed as binary-double decimal.
            if transport < -1e-6 or age < 0:
                raise ValueError('negative latency beyond timestamp precision')
            latency.append(dict(sec=(producer['tj']-BASE_NS)/1e9, rev=key[2],
                                source_age_sec=age, logtail_observation_sec=transport))
    if seen != producers.keys():
        raise ValueError('not every logged GLIM version was matched')

    blocks = {}
    for lo, hi in [(0,40), (40,80), (80,120), (120,160), (0,160)]:
        selected = [s for s in scans.values() if lo <= s['sec'] < hi]
        first = [s for s in latency if s['rev'] == 0 and lo <= s['sec'] < hi]
        life = [s for s in lifecycle if lo <= s['sec'] < hi]
        keys = ['total_ms','main_ms','local_ms','pre_other_ms','post_ms','remaining_scan_ms',
                'covariance_prep_ms','covariance_query_ms','outgoing_and_revisions_ms','active_frames']
        complete = [s for s in selected if 'local_ms' in s]
        total = sum(s['total_ms'] for s in complete)
        # Disjoint wall timers, not sums of percentiles and not CPU/GPU utilization.
        shares = {key: 100*sum(s[key] for s in complete)/total for key in
                  ('main_ms','local_ms','pre_other_ms','post_ms','remaining_scan_ms')}
        blocks[f'{lo}-{hi}'] = dict(timing={k: summary(s[k] for s in selected if k in s) for k in keys},
                                   measured_scan_wall_percent=shares, share_complete_rows=len(complete),
                                   over_1000ms=sum(s['total_ms']>1000 for s in selected),
                                   first_belief_latency={k: summary(s[k] for s in first) for k in
                                                        ('source_age_sec','logtail_observation_sec')},
                                   lifecycle={k: summary(s[k] for s in life) for k in
                                              ('active','marginalizing','external','removing_external')})
    warning_counts = collections.Counter()
    warning_bytes = 0
    with (SOURCE/'estimators.log').open('rb') as stream:
        for line in stream:
            for name, marker in [('linearization', b'warning: performing linearization'),
                                 ('error', b'warning: computing error')]:
                if marker in line:
                    warning_counts[name] += 1
                    warning_bytes += len(line)
    result = dict(schema=1, units='timers: wall ms; latencies: ROS simulated seconds',
                  scan_count=len(scans), stage_frame_count=len(stages), missing_stage_frames=missing_stage_frames,
                  matched_glim_versions=len(seen), first_versions=sum(s['rev']==0 for s in latency),
                  lifecycle_lags=sorted(set(s['lag'] for s in lifecycle)), blocks=blocks,
                  all_revision_transport_sec=summary(s['logtail_observation_sec'] for s in latency),
                  timestamp_roundoff_negative_transport_count=sum(s['logtail_observation_sec']<0 for s in latency),
                  warning_counts=dict(warning_counts), warning_line_bytes=warning_bytes,
                  protected_inputs=protected, source_git=git_before)
    assert protected == {str(p): fingerprint(p) for p in inputs}, 'input mutation'
    assert git_before == source_git(), 'source worktree mutation'
    result['inputs_and_source_git_unchanged'] = True
    with args.out.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('scan_count', 'matched_glim_versions', 'first_versions',
                                            'warning_counts','inputs_and_source_git_unchanged')}))


if __name__ == '__main__':
    main()
