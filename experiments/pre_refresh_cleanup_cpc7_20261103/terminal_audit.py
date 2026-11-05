"""Supplemental original CP89/CP91 terminal hash checks; no artifact changes."""
from pathlib import Path
import datetime,hashlib,json,sys
H=Path(__file__).resolve().parent;E=H.parent
label=sys.argv[1];assert label in ('before','after')
out={}
for name in ('eth1_alpha_bias_rw_full_off_cpv89_20261102','eth1_split_imu_persistent_on_cpv91_20261102_attempt1'):
    c=E/name/'complete.json';doc=json.loads(c.read_text())
    for path,expected in doc['output_sha256'].items():
        with open(path,'rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==expected,path
    with c.open('rb') as f:h=hashlib.file_digest(f,'sha256').hexdigest()
    out[name]={'output_hashes_passed':len(doc['output_sha256']),'complete_sha256':h}
with (H/f'terminal_{label}.json').open('x') as f:json.dump({'at':datetime.datetime.now().astimezone().isoformat(),'passed':True,'experiments':out},f,indent=2)
if label=='after':assert out==json.loads((H/'terminal_before.json').read_text())['experiments']
print(json.dumps(out,indent=2))
