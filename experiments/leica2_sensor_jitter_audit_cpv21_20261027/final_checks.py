"""Read-only source/checkpoint verification; writes only a new CP-V21 receipt."""
import hashlib
import json
import subprocess
from pathlib import Path
from datetime import datetime, timezone
import numpy as np

H=Path(__file__).resolve().parent
assert not (H/'final_checks.json').exists()
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins={}
for name in ('results.json','frame_time_checks.json'):
    r=json.loads((H/name).read_text());assert r['inputs_unchanged']
    for p,s in r['input_sha256'].items():
        assert p not in pins or pins[p]==s
        pins[p]=s
bad=[p for p,s in pins.items() if sha(p)!=s]
assert not bad,bad
d=np.load(H/'timeseries.npz')
same=np.array_equal(d['hesai_frontier_age_s'][:,1],d['velodyne_frontier_age_s'][:,1],equal_nan=True)
assert same
checkpoint=subprocess.run(['python3','/home/yeranis/repos/gnssFGO/experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py','--originals'],capture_output=True,text=True,check=True)
artifacts={p.name:sha(p) for p in H.iterdir() if p.is_file()}
out={'utc':datetime.now(timezone.utc).isoformat(),'status':'PASS','pinned_input_count':len(pins),
     'inputs_unchanged':True,'kimera_endpoint_frontier_at_every_publication_identical':bool(same),
     'hesai_checkpoint_verification':checkpoint.stdout.strip(),'artifact_sha256':artifacts,
     'scope':'Saved-data diagnosis only. No source or receiver run, build, tuning or fix.'}
with (H/'final_checks.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='artifact_sha256'},indent=2))
