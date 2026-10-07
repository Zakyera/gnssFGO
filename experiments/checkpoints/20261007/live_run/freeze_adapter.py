import hashlib
import json
from pathlib import Path
from prepare import ROOT, REPO

old = json.loads((ROOT/'protected_manifest.json').read_text())
allowed = {str(REPO/'tools/live_receiver_protocol.py'), str(REPO/'tools/grandtour_live_receiver_ros1.py')}
new = {}
for name, expected in old.items():
    new[name] = hashlib.sha256(Path(name).read_bytes()).hexdigest()
    assert name in allowed or new[name] == expected, name
with (ROOT/'protected_manifest_v2.json').open('x') as f:
    json.dump(new,f,indent=2)
print('Only the online pose schema adapter changed; estimator binary, model and profiles unchanged.')
