"""Read-only snapshot, including source lag, not just the receiver queue."""
import csv
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent/'prepared'
s=json.loads((ROOT/'live160_on_receiver/transport_status.json').read_text())
source=ROOT/'live160_on_source'
paths=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
failures=0;stamp=None;cost=None
for p in [source/'estimators.log',*paths]:
    for line in p.open():
        failures+='MEAN_ALIGNMENT_UNAVAILABLE' in line
        if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
            row=line.split('GLIM_ODOM_IMU_TIMING_ROW,')[1].strip().split(',')
            if row[-1]=='processed': stamp=float(row[0]);cost=float(row[-2])
result=dict(sensor_seconds=round((s['last_imu_ns']-1727775008003475000)*1e-9,2),
    fused_poses=s['published_poses'],queue=s['queued'],output_age_s=s['output_age_seconds'],
    versions=s['beliefs'],alignment_failures=failures,
    latest_glim_pose_age_s=None if stamp is None else round(s['last_imu_ns']*1e-9-stamp,3),
    latest_glim_scan_wall_ms=cost,
    failed=(ROOT/'live160_on_control/failed.txt').exists(),
    complete=(ROOT/'live160_on_control/complete.json').exists())
print(json.dumps(result))
