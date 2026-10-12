"""Keep failed live equality distinct from a matched offline diagnostic."""
from pathlib import Path
import json
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'receiver_subphase_s5b_20261013'))
from compare import pair, ROOT as OLD
ROOT=Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
assert not json.loads((ROOT/'reference_validation.json').read_text())['passed']
result=pair(OLD/'full_off',ROOT/'reference')
assert result['passed'], 'No internally matching offline baseline established'
result['scope']='Unmodified current offline replay exactly equals prior timers-disabled replay; neither is byte-identical to retained live output. Original full-reference gate remains FAILED.'
output=ROOT/'offline_baseline_identity.json';assert not output.exists()
with output.open('x') as f:json.dump(result,f,indent=2)
print(result['scope'])
