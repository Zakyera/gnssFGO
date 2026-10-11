"""Read-only native-stream tail/coverage check, in addition to receiver checks."""
from pathlib import Path
import csv
import json
import sys
import numpy as np

root=Path(__file__).resolve().parent
tag=sys.argv[1]
source=root/(tag+'_source')
live=root/(tag+'_receiver')
assert (source/'complete.marker').exists() and (live/'complete.marker').exists()
imus=[]
for line in (live/'events.csv').open():
    fields=line.rstrip().split(',')
    if fields[0]=='I': imus.append(int(fields[2])*1e-9)
glim=np.loadtxt(source/'odom_imu.txt')
with (source/'kimera_odometry.csv').open() as f:
    kimera=np.array([[float(row[k]) for k in ['field.header.stamp',
      'field.pose.pose.position.x','field.pose.pose.position.y','field.pose.pose.position.z',
      'field.pose.pose.orientation.x','field.pose.pose.orientation.y','field.pose.pose.orientation.z',
      'field.pose.pose.orientation.w']] for row in csv.DictReader(f)])
kimera[:,0]*=1e-9
result=dict(tag=tag,imu_start=imus[0],imu_end=imus[-1],imu_count=len(imus),sources={})
for name,a in [('native_glim',glim),('native_kimera_ros_odometry',kimera)]:
    assert np.isfinite(a).all() and a.shape[1]==8
    assert len(a)>100 and np.all(np.diff(a[:,0])>0)
    span=float(a[-1,0]-a[0,0]); tail=float(imus[-1]-a[-1,0])
    assert span>28 and -.5<tail<1, (name,span,tail)
    result['sources'][name]=dict(rows=len(a),start=float(a[0,0]),end=float(a[-1,0]),
        duration=span,tail_gap_s=tail,path_m=float(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1).sum()))
result['passed']=True
with (root/(tag+'_coverage.json')).open('x') as f: json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
