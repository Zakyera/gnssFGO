"""Compact read-only view; no full-log loading while the benchmark is running."""
import json
from prepare import ROOT
control=ROOT/'capacity160_on_control'
for name in ['complete.json','failed.txt']:
    p=control/name
    if p.exists():print(name,p.read_text().strip())
status=ROOT/'capacity160_on_receiver/transport_status.json'
if status.exists():
    r=json.loads(status.read_text())
    print(json.dumps(dict(sim_elapsed_s=(r['last_imu_ns']-1727775008003475000)/1e9,
        published=r['published_poses'],belief_versions=r['beliefs'],queue=r['queued'],
        peak_queue=r['max_queue'],output_age_s=r['output_age_seconds'],sources=r['sources'])))
else:print('Transport not ready')
