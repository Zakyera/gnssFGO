"""Keep summary evidence small for a future checkpoint; raw details stay local."""
import json
from pathlib import Path

here=Path(__file__).resolve().parent
out=here/'summary.json'
assert not out.exists()
data={}
for seq in ['eth1','spx3','eig2']:
    x=json.loads((here/f'{seq}_results.json').read_text())
    x.pop('records')
    for direction in x['injections'].values():
        direction.pop('records')
    data[seq]=x
with out.open('x') as stream:
    json.dump(data,stream,indent=2,allow_nan=False)
