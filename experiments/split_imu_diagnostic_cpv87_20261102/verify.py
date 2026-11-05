"""Integrity checks only. Creates one new CP87 verification receipt."""
import ast
import datetime
import hashlib
import json
from pathlib import Path

H=Path(__file__).resolve().parent
assert not (H/'verification.json').exists()
cache={}
def sha(p):
    p=Path(p)
    if str(p) not in cache:
        with p.open('rb') as f:cache[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return cache[str(p)]
def check_manifest(m):
    for p,s in m.items():assert sha(p)==s,p
    return len(m)
out={'finished_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
     'estimator_runs_started':0,'settings_changed':False,'new_trajectory_scoring':False}
out['supplement_inputs']={}
for fn in ['input_sha256.json','supplement_1_inputs.json','supplement_2_inputs.json']:
    out['supplement_inputs'][fn]=check_manifest(json.loads((H/fn).read_text()))
out['immutable_terminal_outputs']={}
for n in ['eth1_split_imu_off60_cpv85_20261101_attempt1/report_completion_1',
          'eth1_cpt7_control_off60_cpv86_20261102']:
    p=H.parent/n/'complete.json';j=json.loads(p.read_text())
    out['immutable_terminal_outputs'][n]={'complete_sha256':sha(p),
        'outputs_checked':check_manifest(j['output_sha256'])}
cp=H.parent/'checkpoints/20261023_hesai_four_sequences'
m=json.loads((cp/'manifest.json').read_text())
for n,r in m['files'].items():
    for p in [cp/n,Path(r['original'])]:
        assert p.stat().st_size==r['bytes'] and sha(p)==r['sha256'],p
out['hesai_checkpoint']={'cases':len(m['cases']),'entries':len(m['files']),
                          'copies_and_originals_pass':True}
for p in H.glob('*.py'):ast.parse(p.read_text())
out['passed']=True
out['output_sha256']={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()}
with (H/'verification.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='output_sha256'},indent=2))
