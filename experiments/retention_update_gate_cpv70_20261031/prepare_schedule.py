"""Generate only delivery metadata from already-frozen CP69 inputs, no ROS replay."""
import hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name; R=K/'replay'
old=W/'runs/retention_compatibility_cpv69_20261031/replay_pathcheck'
for name in ('frozen_inputs.json','aligned_inputs.bag'):
    assert (old/name).read_bytes()==(R/name).read_bytes()
rows=json.loads((R/'frozen_inputs.json').read_text())['rows']
assert len(rows)==292 and len({tuple(r['sender']) for r in rows})==292
assert [r['update'] for r in rows]==sorted(r['update'] for r in rows)
with (R/'recorded_updates.txt').open('x') as f:
    for r in rows: f.write(' '.join(map(str,[r['update']]+r['sender']))+'\n')
with (H/'schedule.json').open('x') as f:
    json.dump(dict(count=len(rows),first_update=rows[0]['update'],last_update=rows[-1]['update'],
        same_input_bytes=True,source=str(old),schedule_sha256=hashlib.sha256((R/'recorded_updates.txt').read_bytes()).hexdigest()),f,indent=2)
