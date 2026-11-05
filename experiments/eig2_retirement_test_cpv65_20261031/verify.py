"""Independent raw-position cross-check, fixture log and preservation closeout."""
import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
H = Path(__file__).resolve().parent
R = H.parents[1]
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')


def sha(p):
    with Path(p).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(name, value):
    with (H / name).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def main():
    assert not (H / 'verification.json').exists()
    analysis = json.loads((H / 'analysis.json').read_text())
    events = json.loads((H / 'event_manifest.json').read_text())
    for p, expected in analysis['input_sha256'].items():
        assert sha(p) == expected, p
    process = subprocess.run([sys.executable, '-B', '-m', 'unittest', '-v', 'test_screen'],
                             cwd=H, capture_output=True, text=True)
    with (H / 'fixture_verification.log').open('x') as stream:
        stream.write(process.stdout + process.stderr)
    assert process.returncode == 0
    checked = 0
    max_error = 0.
    # Read raw xyz fields independently, without the CP50 pose/quaternion parser.
    for mode in ('off', 'on'):
        path = W / 'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared' / f'full_{mode}_source/estimators.log'
        positions = {}
        tag = 'KIMERA_CBS_REVISION_EDGE_ROW,'
        with path.open() as stream:
            for line in stream:
                if tag not in line:
                    continue
                fields = line.split(tag, 1)[1].split(',')
                k, i, j = int(fields[0]), int(fields[2]), int(fields[3])
                window = positions.setdefault(k, {})
                for key, offset in ((i, 8), (j, 15)):
                    point = np.array([float(v) for v in fields[offset:offset+3]])
                    if key in window:
                        assert np.array_equal(window[key], point)
                    window[key] = point
        for row in analysis['rows']:
            if row['mode'] != mode:
                continue
            k = row['k']
            old = positions[k-1][k-1]
            revised = positions[k][k-1]
            new = positions[k][k]
            independent = dict(correction_world_m=revised-old,
                               within_solution_world_m=new-revised,
                               published_world_m=new-old)
            for name, value in independent.items():
                error = float(np.linalg.norm(value-np.array(row[name])))
                assert error <= 1e-12
                max_error = max(max_error, error)
                checked += 1
    assert checked == 3*len(analysis['rows'])
    assert len(events['events']) == 8
    assert all(len(e['missing_native_snapshots']) == 4 for e in events['events'])
    assert events['source_capture_launched'] is False and events['native_optimizer_calls'] == 0
    checkpoint = R / 'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
    process = subprocess.run([sys.executable, '-B', str(checkpoint), '--originals'],
                             capture_output=True, text=True)
    with (H / 'hesai_checkpoint.log').open('x') as stream:
        stream.write(process.stdout + process.stderr)
    assert process.returncode == 0, process.stdout + process.stderr
    assert 'PASS: 4 cases, 667 saved files; originals=True' in process.stdout
    for p, expected in analysis['input_sha256'].items():
        assert sha(p) == expected, p
    save('verification.json', dict(passed=True, scope='saved-data screen and test design, NOT causal solver arms',
         fixture_tests=9, fixture_coordinate_cases=100, independently_checked_vectors=checked,
         maximum_raw_position_vector_difference_m=max_error, original_input_hashes=len(analysis['input_sha256']),
         hesai_checkpoint=process.stdout.strip(), missing_requested_native_snapshots=32,
         source_capture_launched=False, native_optimizer_calls=0,
         finished=datetime.datetime.now().astimezone().isoformat(),
         output_sha256={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()}))
    print('PASS: saved-data screen; 9 fixture tests; raw vectors', checked,
          '; native causal stage NOT RUN, missing 32 snapshots.')


if __name__ == '__main__':
    main()
