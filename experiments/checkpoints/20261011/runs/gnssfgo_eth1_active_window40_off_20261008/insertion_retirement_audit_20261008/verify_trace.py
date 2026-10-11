"""Additional read-only consistency checks; refuses to replace audit output."""
import json
from pathlib import Path

import numpy as np

import analyze as audit


def main():
    destination = audit.ROOT / 'trace_validation.json'
    assert not destination.exists(), 'Refuse to overwrite retained validation'
    result = json.loads((audit.ROOT / 'summary.json').read_text())
    on, times, _, _ = audit.parse_snapshots(audit.ON / 'pilot_on_source/estimators.log')
    off, _, _, _ = audit.parse_snapshots(audit.OFF / 'pilot_off_source/estimators.log')
    pre_differences = {}
    for update in (1, 2, 3):
        assert set(on[update]) == set(off[update])
        pre_differences[update] = float(max(
            np.max(np.abs(on[update][key] - off[update][key]))
            for key in on[update]))
    assert max(pre_differences.values()) == 0.0

    wire = json.loads((audit.ROOT / 'wire_beliefs.json').read_text())
    outputs = {item['update']: item for item in wire['kimera_outputs']}
    import csv
    with (audit.ROOT / 'accepted_insertions.csv').open() as stream:
        events = list(csv.DictReader(stream))
    beliefs = {(item['i'], item['j']): item for item in wire['beliefs']}
    causal = []
    for event in events:
        belief = beliefs[(int(event['sender_i']), int(event['sender_j']))]
        causal.append(belief['record_ns'] <= outputs[int(event['update'])]['record_ns'])
    assert all(causal)
    unchanged = {path: audit.sha(Path(path)) == digest
                 for path, digest in result['input_sha256'].items()}
    assert all(unchanged.values())
    validation = dict(
        pre_first_insertion_snapshot_max_element_difference=pre_differences,
        all_accepted_wire_arrivals_before_applied_update_output=all(causal),
        accepted_count=len(events),
        first_marginalization_update=result['first_marginalization_update'],
        first_marginalization_physical_time_s=(
            outputs[result['first_marginalization_update']]['header_ns'] * 1e-9 - audit.T0),
        maximum_pre_residual_reconstruction_error=max(float(e['pre_residual_check']) for e in events),
        maximum_post_residual_reconstruction_error=max(float(e['post_final_residual_check']) for e in events),
        input_hashes_unchanged=unchanged,
        caveat='Arrival ordering is checked against bag record times; event times in the report are physical output timestamps, not wall-clock execution times.')
    with destination.open('x') as stream:
        json.dump(validation, stream, indent=2, allow_nan=False)
    print(json.dumps(validation, indent=2))


if __name__ == '__main__':
    main()
