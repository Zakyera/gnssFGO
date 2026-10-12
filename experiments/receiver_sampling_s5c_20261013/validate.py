"""Reuse exact S5b comparison semantics, not its changed executable."""
import sys
import json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'receiver_subphase_s5b_20261013'))
from compare import pair, RETAINED
ROOT=Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
tag=sys.argv[1]
assert tag in ['prefix','prefix_deep','prefix_deep8','reference','sample','sample20','replay_sample','replay_sample20','replay_sample20_deep8']
output=ROOT/(tag+'_validation.json');assert not output.exists()
if tag=='prefix':a,b=ROOT/'prefix_reference',ROOT/'prefix_sample'
elif tag in ['prefix_deep','prefix_deep8']:a,b=ROOT/'prefix_reference',ROOT/tag
elif tag=='reference':a,b=RETAINED,ROOT/'reference'
else:a,b=ROOT/'reference',ROOT/tag
result=pair(a,b)
with output.open('x') as f:json.dump(result,f,indent=2)
print(tag,'PASS' if result['passed'] else 'FAIL')
for n,c in result['checks'].items():
    if not c['equal']:print(n,c)
if not result['passed']:raise SystemExit(1)
