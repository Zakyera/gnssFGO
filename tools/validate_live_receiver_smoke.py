#!/usr/bin/env python3
"""Verify captured transport payloads and deterministic replay products."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input-journal', type=Path, required=True)
    p.add_argument('--live', type=Path, required=True)
    p.add_argument('--repeat', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    def data(path):
        imus, beliefs = [], {}
        for line in path.read_text().splitlines():
            f = line.split(',')
            if f[0] == 'I':
                imus.append((int(f[2]), *map(float, f[3:])))
            elif f[0] == 'B':
                source = {'g': 'glim_hesai', 'glim': 'glim_hesai',
                          'k': 'kimera_front', 'kimera': 'kimera_front'}.get(f[2], f[2])
                key = (source, int(f[6]), int(f[7]))
                beliefs.setdefault(key, []).append((int(f[8]), int(f[9]), *map(float, f[10:])))
        return imus, beliefs
    expected = data(args.input_journal)
    received = data(args.live/'events.csv')
    assert expected == received, 'source payload/timestamp/order changed in transport'
    identity = {}
    for name in ('trajectory.tum', 'states.csv', 'online_trajectory.tum', 'online_states.csv',
                 'support_accelerations.csv', 'support_clock.csv', 'model_covariance.csv'):
        original = (args.live/'receiver'/name).read_bytes()
        assert original == (args.repeat/name).read_bytes(), name
        identity[name] = hashlib.sha256(original).hexdigest()
    args.out.write_text(json.dumps(dict(passed=True, exact_payload_conservation=True,
                                       imu_samples=len(received[0]),
                                       belief_snapshots=sum(map(len, received[1].values())),
                                       source_ids=sorted(set(k[0] for k in received[1])),
                                       repeat_byte_identity_sha256=identity), indent=2)+'\n')


if __name__ == '__main__':
    main()
