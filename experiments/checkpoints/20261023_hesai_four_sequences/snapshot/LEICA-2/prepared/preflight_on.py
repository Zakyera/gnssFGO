"""Admit ON only after this exact matched OFF passes ordinary capture/log checks; no repeat replay."""
from pathlib import Path
import hashlib,json
P=Path(__file__).resolve().parent
V=Path('/home/yeranis/results/gnssfgo_leica2_full_cpk29_20261023/off')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for p in [P/'full_off_source/candidate_validation.json',P/'full_off_source/local_provenance.json',
 V/'validation.json']:
 assert json.loads(p.read_text())['passed'],p
assert (P/'full_off_control/complete.json').is_file()
assert not any((P/f'full_on_{part}').exists() for part in ('source','receiver','control'))
manifest=json.loads((P/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p
assert 'assert enabled' in (P/'verify_effective_on.py').read_text()
cfg=[json.loads((P/f'full_{m}.json').read_text()) for m in ('off','on')]
for c in cfg:
 for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
 for source in c['sources']:source.pop('session')
assert cfg[0]==cfg[1]
result=dict(passed=True,admission='same428.75s OFF passes all gates before matched ON',
 protected_unchanged=len(manifest),receiver_config_unchanged=True)
with (P/'preflight_on.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
