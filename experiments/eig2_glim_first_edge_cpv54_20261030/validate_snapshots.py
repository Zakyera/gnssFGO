"""Completeness gate; offline native equation analysis follows separately."""
import csv,json,sys
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
mode=sys.argv[1];assert mode in ('off','on');S=P/f'full_{mode}_source'
paths=list((S/'checkpoints').glob('update_*/*/checkpoint.bin'))
expected={(k,s) for k in range(6,26) for s in ('before_ordinary','after_ordinary','after_refresh')}
assert {(int(p.parent.parent.name.split('_')[1]),p.parent.name) for p in paths}==expected and len(paths)==60
for p in paths:
    assert p.stat().st_size>1000 and (p.parent/'complete.txt').is_file()
    assert not (p.parent/'roundtrip.txt').exists(),'test-only live factor evaluations enabled'
with (S/'objective.csv').open() as f:rows=list(csv.reader(f))
assert all(float(r[2])==0 for r in rows[:26]),'removal before recorded boundary'
v=dict(passed=True,mode=mode,snapshots=len(paths),bytes=sum(p.stat().st_size for p in paths),
       updates=[6,25],native_roundtrip_during_capture=True,offline_equation_analysis_pending=True)
with (H/(mode+'_snapshots.json')).open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v,indent=2))
