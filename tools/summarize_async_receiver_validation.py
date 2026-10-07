#!/usr/bin/env python3
"""Assert receiver timing/accounting invariants and retain a compact manifest."""
import argparse
import csv
import hashlib
import json
import statistics
from pathlib import Path


def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def stats(values):
    ordered = sorted(values)
    if not ordered:
        return None
    at = .95*(len(ordered)-1)
    i = int(at)
    return dict(count=len(values), mean=statistics.mean(values),
                median=statistics.median(values),
                p95=ordered[i]+(ordered[min(i+1, len(ordered)-1)]-ordered[i])*(at-i),
                maximum=max(values))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--products', type=Path, required=True)
    p.add_argument('--frozen', type=Path, required=True)
    args = p.parse_args()
    root = args.products
    independent = json.loads((root/'uniform/summary.json').read_text())
    legacy = json.loads((root/'legacy/summary.json').read_text())
    repeat = json.loads((root/'repeat/summary.json').read_text())
    hashes = {}
    for filename in ['trajectory.tum', 'states.csv', 'online_trajectory.tum', 'online_states.csv']:
        assert (root/'legacy'/filename).read_bytes() == (args.frozen/filename).read_bytes()
        assert (root/'uniform'/filename).read_bytes() == (root/'repeat'/filename).read_bytes()
        hashes[filename] = hashlib.sha256((root/'uniform'/filename).read_bytes()).hexdigest()
    for filename in ['support_clock.csv', 'support_accelerations.csv', 'causal_dispatch.csv']:
        assert (root/'uniform'/filename).read_bytes() == (root/'repeat'/filename).read_bytes()
    for name in ['gravity', 'initial_accelerometer_bias', 'initial_gyroscope_bias', 'startup_imu_samples']:
        assert independent[name] == legacy[name]
    clock = rows(root/'uniform/support_clock.csv')
    assert all(int(b['physical_timestamp_ns'])-int(a['physical_timestamp_ns']) == 100_000_000
               for a, b in zip(clock, clock[1:]))
    events = rows(root/'uniform/causal_dispatch.csv')
    for event in events:
        required = int(event['required_support'])
        assert required <= int(event['created_support'])
        assert int(event['insertion_ns']) >= int(event['arrival_ns'])
        assert int(event['insertion_ns']) >= int(clock[required]['available_timestamp_ns'])
    outside = rows(root/'uniform/unsupported_beliefs.csv')
    assert len(events)+len(outside) == independent['revision_messages_loaded']
    outside_ids = sorted({(r['source'], r['session'], r['ordinal'], r['reason']) for r in outside})
    timings = rows(root/'uniform/revision_events.csv')
    residuals = rows(root/'uniform/belief_residuals.csv')
    report = dict(passed=True, independent_summary=independent,
                  repeat_wall_seconds=repeat['wall_seconds'],
                  legacy_byte_identical=True, repeat_byte_identical=True,
                  initialization_values_identical=True, hashes=hashes,
                  unsupported_ids=outside_ids,
                  insertion_wait_seconds=stats([(int(r['insertion_ns'])-int(r['arrival_ns']))*1e-9 for r in events]),
                  optimizer_update_seconds=stats([float(r['update_seconds']) for r in timings]),
                  support_availability_lag_seconds=stats([(int(r['available_timestamp_ns'])-int(r['physical_timestamp_ns']))*1e-9 for r in clock]),
                  final_whitened_residual_squared={source: stats([float(r['mahalanobis_squared'])
                      for r in residuals if r['source'] == source]) for source in ['glim', 'kimera']})
    (root/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
