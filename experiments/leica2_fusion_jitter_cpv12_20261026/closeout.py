"""Non-mutating original-input verification and a new audit receipt."""
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

H=Path(__file__).resolve().parent
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()

assert not (H/'closeout.json').exists()
main=json.loads((H/'results.json').read_text())
phase=json.loads((H/'phase_checks.json').read_text())
combined=dict(main['input_sha256'])
for p,h in phase['input_sha256'].items():
    assert p not in combined or combined[p]==h
    combined[p]=h
fail=[p for p,h in combined.items() if sha(p)!=h]
assert not fail,fail
outputs={p.name:sha(p) for p in H.iterdir() if p.is_file()}
result=dict(time_utc=datetime.now(timezone.utc).isoformat(),
    status='saved-data audit complete; individual-update causal trace unavailable in original logs',
    input_files_verified=len(combined),all_consumed_inputs_unchanged=True,
    output_sha256=outputs,source_capture=False,receiver_replay=False,
    estimator_build_or_settings_change=False,container_start=False,
    independent_TUM_path_and_ATE_checks=True,
    original_receiver_binary_and_headers_match_capture=True,
    figures_visually_inspected=['jitter_diagnosis.png','publication_cycle.png'],
    root_cause_isolated=False,next_proposed_replay_executed=False)
with (H/'closeout.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in result.items() if k!='output_sha256'},indent=2))
