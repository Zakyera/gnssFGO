"""Compact read-only view; no full-log loading while the benchmark is running."""
import json
import re
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
source=ROOT/'capacity160_on_source'
logs=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
if len(logs)==1:
    with logs[0].open('rb') as f:
        f.seek(max(0,logs[0].stat().st_size-512*1024));tail=f.read().decode(errors='replace')
    lines=tail.splitlines()
    for line in reversed(lines):
        if 'SHARED_NATIVE_PROVIDER_QUERY ' not in line:continue
        text=re.sub(r'\x1b\[[0-9;]*m','',line).split('SHARED_NATIVE_PROVIDER_QUERY ',1)[1]
        print('latest_provider_query',text.split(' matrix=',1)[0]);break
    for line in reversed(lines):
        if 'GLIM_ODOM_IMU_TIMING_ROW,' not in line:continue
        f=re.split('[\x1b\r\n]',line.split('GLIM_ODOM_IMU_TIMING_ROW,',1)[1])[0].split(',')
        if len(f)==25:print('latest_scan',json.dumps(dict(frame=int(f[1]),ms=float(f[23]),active_frames=int(f[5]))))
        break
