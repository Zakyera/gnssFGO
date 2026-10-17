"""Admit one matched live ON case only after fresh OFF capture validates."""
import hashlib
import json
from pathlib import Path

P = Path(__file__).resolve().parent
RESULT = Path('/home/yeranis/results/gnssfgo_fixed_lag_live_fl4t_20261016/off')

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()

assert json.loads((P/'live120_off_source/candidate_validation.json').read_text())['passed']
assert json.loads((RESULT/'validation.json').read_text())['passed']
assert json.loads((RESULT/'repeat_validation.json').read_text())['passed']
assert (P/'live120_off_control/complete.json').is_file()
assert not any((P/f'live120_on_{part}').exists() for part in ('source','receiver','control'))
manifest=json.loads((P/'protected_manifest.json').read_text())
for path,want in manifest.items():
    assert sha(path)==want,path
off=json.loads((P/'live120_off.json').read_text())
on=json.loads((P/'live120_on.json').read_text())
for config in (off,on):
    config.pop('sequence')
    for source in config['sources']:
        source.pop('session')
assert off==on
assert sha(P/'receiver_fl4e.sh')==manifest[str(P/'receiver_fl4e.sh')]
assert sha('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live') == '53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'
assert 'assert enabled' in (P/'verify_effective_on.py').read_text()
evidence=dict(passed=True,off_validation=str(RESULT/'validation.json'),
              off_repeat=str(RESULT/'repeat_validation.json'),
              protected_unchanged=len(manifest),receiver_config='identical except labels/sessions',
              source_difference='classic bidirectional receive/injection ON; frozen data/profile/binary')
with (P/'preflight_on.json').open('x') as f:
    json.dump(evidence,f,indent=2)
print(json.dumps(evidence,indent=2))
