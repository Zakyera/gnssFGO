"""Save raw-record gap evidence. No estimator invocation or data alteration."""
from pathlib import Path
from rosbags.rosbag1 import Reader
import json
import numpy as np

H=Path(__file__).resolve().parent
D=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/ARC-3')
epoch=1731934116840035611
out={}
for kind,topic in [('anymal_velodyne','/anymal/velodyne/points'),('cpt7_imu','/boxi/cpt7/imu')]:
    p=D/('2024-11-18-13-48-19_'+kind+'.bag')
    with Reader(p) as r:
        c=next(c for c in r.connections if c.topic==topic)
        a=np.array([v.time for v in r.indexes[c.id]],dtype=np.int64)
    g=np.diff(a)
    out[kind]=dict(path=str(p),count=len(a),first_ns=int(a[0]),last_ns=int(a[-1]),
        max_gap_s=float(g.max()/1e9),median_gap_s=float(np.median(g)/1e9),
        regressions=int((g<=0).sum()),gaps_over_150ms=int((g>150_000_000).sum()),
        gaps_over_1s=[dict(before_ns=int(a[i]),after_ns=int(a[i+1]),
            relative_to_selected_epoch_s=float((a[i]-epoch)/1e9),gap_s=float(g[i]/1e9))
            for i in np.flatnonzero(g>1_000_000_000)])
out['interpretation']='Recorded input absence, not evidence of estimator runtime delay. No capture attempted.'
with (H/'lidar_imu_diagnosis.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
