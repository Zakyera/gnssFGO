"""Retained wall timers, exact cache counts, and sampling overhead accounting."""
import csv
import json
from pathlib import Path
import sys
from analyze import stats, ORIGIN
from symbolize import ROOT

tag=sys.argv[1];output=ROOT/(tag+'_timings.json');assert not output.exists()
if tag=='reference':
    assert json.loads((ROOT/'offline_baseline_identity.json').read_text())['passed']
else:assert json.loads((ROOT/(tag+'_validation.json')).read_text())['passed']
def read(name):
    with (ROOT/tag/name).open() as f:return list(csv.DictReader(f))
pubs=read('publication_convergence.csv');steps=read('publication_steps.csv')
memo=read('e2_memo.csv')
recovery={int(r['index']) for r in steps if r['phase']=='support' and r['recovery']=='1'}
case=json.loads((ROOT/(tag+'.json')).read_text())
result=dict(run=case,cohorts={},total_publication_seconds=sum(float(r['seconds']) for r in pubs))
for label,lo,hi in [('all',0,1e9),('0-40s',0,40),('20-40s',20,40),('60-80s',60,80),('120-140s',120,140),('140s-end',140,1e9)]:
    rows=[r for r in pubs if r['phase']=='support' and lo<=(int(r['physical_ns'])-ORIGIN)/1e9<hi]
    local=dict(publication_ms=stats([float(r['seconds'])*1000 for r in rows]),
        attempts=stats([int(r['attempted']) for r in rows]),
        recovery_publications=sum(int(r['index']) in recovery for r in rows),
        nonrecovery_ms=stats([float(r['seconds'])*1000 for r in rows if int(r['index']) not in recovery]),
        over_1000ms=sum(float(r['seconds'])>1 for r in rows),cache={})
    for kind in ['0','1']:
        selected=[r for r in memo if r['phase']=='support' and r['kind']==kind]
        previous={key:0 for key in ['calls','hits','misses','failures']};total=previous.copy()
        for r in selected:
            current={key:int(r[key]) for key in previous}
            if lo<=(int(r['physical_ns'])-ORIGIN)/1e9<hi:
                for key in total:total[key]+=current[key]-previous[key]
            previous=current
        local['cache'][kind]=total
    result['cohorts'][label]=local
if 'sample' in tag:
    reference=json.loads((ROOT/'reference.json').read_text())
    result['wall_overhead_fraction']=case['seconds']/reference['seconds']-1
    result['total_cpu_overhead_fraction']=(case['user_seconds']+case['system_seconds'])/(reference['user_seconds']+reference['system_seconds'])-1
    end=json.loads((ROOT/(tag+'_profile')/'end.json').read_text())
    result['main_thread_cpu_over_total_cpu']=end['thread_cpu_ns']/1e9/(case['user_seconds']+case['system_seconds'])
with output.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
