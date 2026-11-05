"""Read-only compact progress, with explicit failures. No estimator/score calls."""
import csv,json
from pathlib import Path
H=Path(__file__).resolve().parent
C=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_off_refresh_full_cpv39_20261029/capture')
out={}
for root,name in [(H,'controller_failed.json'),(H,'container_restoration.json'),(C,'failed.txt'),(C,'capture_complete.json'),(C,'status.json')]:
    p=root/name
    if p.exists():out[name]=json.loads(p.read_text()) if p.suffix=='.json' else p.read_text()
if (C/'observer_status.json').exists():
    d=json.loads((C/'observer_status.json').read_text());out['topics']={}
    for t,r in d['topics'].items():
        out['topics'][t]=dict(rows=r['count'],last_sensor_s=(r['last_ns']-1730638816563546034)*1e-9,max_gap_s=r['max_gap_ns']*1e-9)
print(json.dumps(out,indent=2))
