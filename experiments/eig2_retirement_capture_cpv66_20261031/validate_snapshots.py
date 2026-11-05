"""Completeness gate; offline native equation analysis follows separately."""
import csv,json,sys
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
mode=sys.argv[1];assert mode in ('off','on');S=P/f'full_{mode}_source'
paths=list((S/'checkpoints').glob('update_*/*/checkpoint.bin'))
with (S/'objective.csv').open() as f:native=[int(float(r[0])) for r in csv.reader(f)]
assert native==list(range(len(native))) and max(native)>=295,'native late window missing'
updates=[k for k in native if 28<=k<=35 or 200<=k<=298]
expected={(k,s) for k in updates for s in ('before_ordinary','after_ordinary','after_refresh','after_marginalization')}
assert {(int(p.parent.parent.name.split('_')[1]),p.parent.name) for p in paths}==expected and len(paths)==4*len(updates)
for p in paths:
    assert p.stat().st_size>1000 and (p.parent/'complete.txt').is_file()
    assert not (p.parent/'roundtrip.txt').exists(),'test-only live factor evaluations enabled'
events={}
for k in updates:
    event=S/'checkpoints'/f'update_{k}'/'event.txt'
    with event.open() as f:events[k]=list(csv.reader(f))
    assert any(r[0]=='added' for r in events[k]),('empty native update',k)
    for r in events[k]:
        assert r[0] in ('added','removed','marginal')
        if r[0]=='removed':assert r[-1]=='1'
removed=sum(r[0]=='removed' for rows in events.values() for r in rows)
assert removed>0 if mode=='on' else removed==0
v=dict(passed=True,mode=mode,snapshots=len(paths),bytes=sum(p.stat().st_size for p in paths),
       updates=updates,removed_factors=removed,native_roundtrip_during_capture=True,offline_equation_analysis_pending=True)
with (H/(mode+'_snapshots.json')).open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v,indent=2))
