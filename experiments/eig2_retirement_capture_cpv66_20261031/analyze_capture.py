"""Saved-only 60s publication decomposition and exact branch-target selection."""
import csv, importlib.util, json, sys
from pathlib import Path
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'prepared'
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
a=module('cpv50_parser',H.parent/'eig2_hesai_onset_cpv50_20261030/analyze.py')
b=module('cpv65_geometry',H.parent/'eig2_retirement_test_cpv65_20261031/screen.py')
assert json.loads((H/'capture_complete.json').read_text())['passed']
rows=[];omitted=[];ledgers={};targets={}
for mode in ('off','on'):
    S=P/f'full_{mode}_source'; data=a.parse_source(S/'estimators.log')
    logs=[p for p in (S/'ros_logs').glob('*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log') if p.parent.name!='latest']
    assert len(logs)==1
    native=a.parse_stdout(logs[0]); assert not native['failures']
    with a.pin(S/'kimera_odometry.csv').open() as f:stamps=[int(r['field.header.stamp']) for r in csv.DictReader(f)]
    actions={}
    for r in native['actions']:actions.setdefault(r['k'],[]).append(r)
    previous=None;ledger=[]
    for k,life in sorted(native['life'].items()):
        added=[];removed=[]
        for r in actions.get(k,[]):
            if r['action'] not in ('active_window_slot_tracked','active_window_remove_scheduled'):continue
            detail=dict(s.split('=',1) for s in r['details'])
            i,j=(int(s[1:]) for s in r['edge'].split('->'))
            item=dict(slot=int(detail['slot']),from_id=i,to_id=j)
            (added if r['action']=='active_window_slot_tracked' else removed).append(item)
        assert len(removed)==life['retiring_factors']
        ledger.append(dict(k=k,added=added,removed=removed,life=life))
        old=data['windows'].get(k-1,{});new=data['windows'].get(k,{})
        if k-1 not in old or k-1 not in new or k not in new:
            omitted.append(dict(mode=mode,k=k));previous=None;continue
        d=b.decomposition(old[k-1],new[k-1],new[k])
        row=dict(mode=mode,k=k,capture_time_s=(stamps[k]-a.EPOCH)*1e-9,
                 removals=len(removed),insertions=len(added),**d)
        for label,key in [('correction','correction_world_m'),('published','published_world_m'),('within_solution','within_solution_world_m')]:
            row[label+'_direction_cosine']=b.cosine(previous[key],row[key]) if previous else None
        rows.append(row);previous=row
    ledgers[mode]=ledger
    targets[mode]={r['k']:r for r in ledger}
on=[r for r in rows if r['mode']=='on']
late=[r for r in on if 200<=r['k']<=298]
assert late
# Freeze selection before viewing any counterfactual result: two controls,
# first six retirements, largest three late previous-state revisions.
selected=list(range(28,36))+[r['k'] for r in sorted(late,key=lambda r:(-r['correction_m'],r['k']))[:3]]
assert len(selected)==len(set(selected))
events=[]
for k in selected:
    d=P/f'full_on_source/checkpoints/update_{k}'
    native_event=list(csv.reader(a.pin(d/'event.txt').open()))
    logged=targets['on'][k]
    removal=[(int(r[1]),int(r[2])&((1<<56)-1),int(r[3])&((1<<56)-1)) for r in native_event if r[0]=='removed']
    assert sorted(removal)==sorted((r['slot'],r['from_id'],r['to_id']) for r in logged['removed'])
    new_slots={int(r[1]) for r in native_event if r[0]=='added'}
    assert all(r['slot'] in new_slots for r in logged['added'])
    for phase in ('before_ordinary','after_ordinary','after_refresh','after_marginalization'):a.pin(d/phase/'checkpoint.bin')
    events.append(dict(**logged,screen=next((r for r in on if r['k']==k),None)))
summary={mode:b.summarize([r for r in rows if r['mode']==mode]) for mode in ('off','on')}
result=dict(passed=True,scope='Saved first-publication decomposition; not ATE/RPE or causal verdict',summary=summary,
            rows=rows,omissions=omitted,ledger=ledgers,selected_events=events,input_sha256=a.PINS)
for p,h in a.PINS.items():assert a.sha(p)==h,p
with (H/'capture_analysis.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
with (H/'causal_targets.txt').open('x') as f:
    for event in events:
        # One event per line; native slots plus endpoint IDs, not all pose factors.
        f.write(str(event['k'])+'\n')
        for kind in ('removed','added'):
            f.write(kind+' '+str(len(event[kind]))+'\n')
            for r in event[kind]:f.write(f"{r['slot']} {r['from_id']} {r['to_id']}\n")
print(json.dumps(dict(summary=summary,selected=selected,largest_late=max(late,key=lambda r:r['correction_m'])),indent=2))
