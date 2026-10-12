"""Read-only supplement for the historical mode's two removal log paths.

The frozen validator expects Kimera's commit label, but BPSAM erases its
tracking entry when scheduling the real removeFactorIndices update. The later
commit method consequently has nothing left to erase/log. This checks the
actual scheduled-slot identities and counts without changing any estimator.
"""
from pathlib import Path
from collections import Counter
import json
import re
import sys
from lifecycle_contract import check_kimera

ROOT = Path(__file__).resolve().parent/'prepared'
OUT = ROOT
tag=sys.argv[1]
assert tag=='profile40_on'
SOURCE = ROOT/(tag+'_source')
logs = [p for p in (SOURCE/'ros_logs').glob('*/*stdout.log') if p.parent.name != 'latest']
klog, = [p for p in logs if 'kimera_vio' in p.name]
glog, = [p for p in logs if 'glim_rosnode' in p.name]
kresult=check_kimera(klog.open())
tracked=kresult['tracked'];removed=kresult['removed_by_update']
active={str(i):None for i in range(kresult['remaining'])}
rollbacks=kresult['rollbacks'];removals=kresult['removals']
guards=Counter()
for line in klog.open():
    for token in ['drop_before_stage','rollback_and_retry_without_staged_cbs',
                  'rollback_remove_active_and_retry','skip_duplicate_receiver_edge']:
        if token in line:guards[token]+=1
gactive = set()
gpending = set()
gqueued = grem = 0
gremovals = []
for ln, line in enumerate(glog.open(), 1):
    m = re.search(r'GLIM_CBS_ODOM_INJECT_ROW,[^,]+,g(\d+)->g(\d+),.*active_window_temporary_graph_queued', line)
    if m:
        edge = tuple(map(int, m.groups()))
        assert edge not in gactive and edge not in gpending, (ln, edge)
        gpending.add(edge)
        gqueued += 1
    m = re.search(r'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,slot_tracked,x(\d+),x(\d+),slot=(\d+),total_active=(\d+)', line)
    if m:
        a,b,slot,total=m.groups()
        edge=(int(a),int(b))
        gpending.remove(edge)
        gactive.add(edge)
        assert int(total)==len(gactive), (ln,total,len(gactive))
    m = re.search(r'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,active_window_remove_committed,x(\d+),x(\d+),active_updates=(\d+),active_age_sec=([^,]+),total_active=(\d+)', line)
    if m:
        a,b,n,age,total = m.groups()
        gactive.remove((int(a),int(b)))
        assert int(total) == len(gactive), (ln, total, len(gactive))
        grem += 1
        gremovals.append(dict(line=ln, i=int(a), j=int(b), updates=int(n), age_s=float(age)))
assert gqueued and grem and gqueued-grem == len(gactive)
assert not gpending
assert (SOURCE/'complete.marker').exists()
result = dict(passed=True,
    kimera=dict(tracked=tracked, removed_by_update=removed, remaining=len(active),
                guards=dict(guards), rollbacks=rollbacks, removals=removals),
    glim=dict(queued=gqueued, removed=grem, remaining=len(gactive), removals=gremovals),
    caveat='Verifies logged factor slot lifecycle and successful completion. Not a matrix-level proof of zero indirect CBS information in marginal priors.')
with (OUT/(tag+'_lifecycle_validation.json')).open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps({k:{a:b for a,b in v.items() if a!='removals'} if isinstance(v,dict) else v for k,v in result.items()}, indent=2))
