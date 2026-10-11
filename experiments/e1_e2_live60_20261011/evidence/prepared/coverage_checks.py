"""Duration-aware completion gate, using measurement stamps, not ROS receipt.

Checks are frozen before capture and intentionally independent of accuracy.
Startup gaps are reported explicitly, never filled or scored as measured poses.
"""
import csv
import json
import sys
from pathlib import Path
import numpy as np

def check(t, p, duration, first_imu, last_imu, role):
    t=np.asarray(t); p=np.asarray(p)
    assert len(t)>10 and p.shape==(len(t),3), (role,'missing poses')
    assert np.isfinite(t).all() and np.isfinite(p).all(), role
    gaps=np.diff(t)
    assert (gaps>0).all(), (role,'unordered stamps')
    assert last_imu-first_imu>=duration-.3, 'IMU stream truncated'
    span=float(t[-1]-t[0]); tail=float(last_imu-t[-1]); start=float(t[0]-first_imu)
    assert span>=duration-2, (role,'truncated coverage',span,duration)
    assert -.1<=tail<=.5 and -.1<=start<=1.5, (role,'start/tail',start,tail)
    # Kimera has a retained ~3.2 s startup gap. Report it and restrict large
    # holes after startup; do not impose a fictitious continuous startup path.
    later=gaps[t[:-1]>=first_imu+5]
    assert not len(later) or later.max()<=1, (role,'post-startup gap',later.max())
    return dict(rows=len(t),start_s=float(t[0]),end_s=float(t[-1]),span_s=span,
        maximum_pose_gap_s=float(gaps.max()),post_startup_max_gap_s=float(later.max()) if len(later) else None,
        start_after_first_imu_s=start,tail_behind_last_imu_s=tail,
        path_length_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()))

def load(path, kimera=False):
    if kimera:
        with path.open() as f: rows=list(csv.DictReader(f))
        return np.array([int(r['field.header.stamp'])*1e-9 for r in rows]),np.array(
            [[float(r['field.pose.pose.position.'+c]) for c in 'xyz'] for r in rows])
    a=np.loadtxt(path,ndmin=2)
    return a[:,0],a[:,1:4]

def main():
    source=Path(sys.argv[1]); duration=float(sys.argv[2]); journal=Path(sys.argv[3])
    imu=[]
    with journal.open() as f:
        for line in f:
            v=line.strip().split(',')
            if v[0]=='I': imu.append(int(v[2])*1e-9)
    assert len(imu)>100 and np.all(np.diff(imu)>0)
    result={'passed':True,'requested_s':duration,'imu_samples':len(imu),'imu_span_s':imu[-1]-imu[0]}
    for role,filename in [('native_glim','odom_imu.txt'),('native_kimera','kimera_odometry.csv')]:
        result[role]=check(*load(source/filename,role=='native_kimera'),duration,imu[0],imu[-1],role)
    with (source/'source_coverage.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result))

if __name__=='__main__':main()
