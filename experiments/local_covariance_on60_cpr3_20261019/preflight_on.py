"""Explicit standalone60s ON admission based on validated CP-R2, not a60s OFF."""
from pathlib import Path
import hashlib,json
P=Path(__file__).resolve().parent
Q=P.parents[1]/'gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
V=Path('/home/yeranis/results/gnssfgo_eig2_local_cov30_cpr2_20261019/off')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for p in [Q/'live30_off_source/candidate_validation.json',Q/'live30_off_source/local_provenance.json',V/'validation.json',V/'repeat_validation.json']:
 assert json.loads(p.read_text())['passed'],p
assert (Q/'live30_off_control/complete.json').is_file()
assert not any((P/f'live60_on_{part}').exists() for part in ('source','receiver','control'))
manifest=json.loads((P/'protected_manifest.json').read_text())
for path,want in manifest.items():assert sha(path)==want,path
assert 'assert enabled' in (P/'verify_effective_on.py').read_text()
on=json.loads((P/'live60_on.json').read_text())
off=json.loads((Q/'live30_off.json').read_text())
for c in (on,off):
 for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
 for source in c['sources']:source.pop('session')
assert on==off
result=dict(passed=True,admission='explicit user request for standalone60s ON after CP-R2 shortOFF',
 matched60s_off_exists=False,prior_validated_off=str(Q),protected_unchanged=len(manifest),
 receiver_config_unchanged=True,source_change='classic bidirectional receiving ON,60s duration; cache and all estimator/model settings frozen')
with (P/'preflight_on.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
