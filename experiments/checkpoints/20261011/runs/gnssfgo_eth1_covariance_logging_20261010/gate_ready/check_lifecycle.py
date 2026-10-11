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

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT/'gate_on_source'
logs = [p for p in (SOURCE/'ros_logs').glob('*/*stdout.log') if p.parent.name != 'latest']
klog, = [p for p in logs if 'kimera_vio' in p.name]
glog, = [p for p in logs if 'glim_rosnode' in p.name]
active = {}
tracked = removed = 0
guards = Counter()
removals = []
for ln, line in enumerate(klog.open(), 1):
    for token in ['drop_before_stage', 'rollback_and_retry_without_staged_cbs',
                  'rollback_remove_active_and_retry', 'skip_duplicate_receiver_edge']:
        if token in line:
            guards[token] += 1
    m = re.search(r'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,k,g,(x\d+->x\d+),(active_window_slot_tracked|active_window_remove_scheduled),slot=(\d+),(.*)', line)
    if not m:
        continue
    edge, action, slot, tail = m.groups()
    total = int(re.search(r'total_active=(\d+)', tail)[1])
    if action == 'active_window_slot_tracked':
        assert edge not in active, (ln, edge)
        active[edge] = int(slot)
        tracked += 1
    else:
        assert active.pop(edge) == int(slot), (ln, edge, slot)
        assert 'reason=update_remove_factor,slot_valid=true' in tail
        removals.append(dict(line=ln, edge=edge, slot=int(slot)))
        removed += 1
    assert total == len(active), (ln, total, len(active))
assert tracked and removed and tracked-removed == len(active)
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
                guards=dict(guards), removals=removals),
    glim=dict(queued=gqueued, removed=grem, remaining=len(gactive), removals=gremovals),
    caveat='Verifies logged factor slot lifecycle and successful completion. Not a matrix-level proof of zero indirect CBS information in marginal priors.')
with (ROOT/'lifecycle_supplement.json').open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps({k:{a:b for a,b in v.items() if a!='removals'} if isinstance(v,dict) else v for k,v in result.items()}, indent=2))
