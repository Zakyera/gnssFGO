"""Bounded runtime sensor/time provenance gate; no reprocessing or optimizer."""
from pathlib import Path
import csv,json,sys
import numpy as np
P=Path(__file__).resolve().parent
mode=sys.argv[1];assert mode in ('off','on')
S=P/f'full_{mode}_source'
assert (P/f'full_{mode}_control/complete.json').exists()
assert json.loads((S/'velodyne_startup.json').read_text())['passed']
pre=json.loads((P/'input_audit.json').read_text())
clouds=pre['clouds'];stamps=np.array([c['normalized_stamp_s'] for c in clouds])
poses=np.loadtxt(S/'odom_imu.txt',ndmin=2)
errors=[];indices=[]
for t in poses[:,0]:
 i=int(np.argmin(abs(stamps-t)));error=float(abs(stamps[i]-t))
 assert error<=5e-6,('published timestamp is not normalized scan',t,error)
 assert clouds[i]['captured_imu_bracketed'],('published scan lacks captured IMU bracket',i)
 errors.append(error);indices.append(i)
assert len(set(indices))==len(indices) and indices==sorted(indices)
assert indices[0]>0 and indices[-1]>=len(clouds)-3
paths=list(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'));assert len(paths)==1
text=paths[0].read_text()
assert 'add an offset to the frame timestamp to make per-point ones positive!!' in text
for bad in ('use pseudo per-point timestamps','point timestamp rewind detected',
            'negative per-point timestamp is found after time conversion',
            'large per-point timestamp is found after time conversion',
            'inconsistent # of points and # of timestamps'):
 assert bad not in text,bad
with (S/'glim_odometry.csv').open() as f:rows=list(csv.DictReader(f))
assert rows and all(r['field.child_frame_id']=='cpt7_imu' for r in rows)
meta=(S/'run_metadata.txt').read_text()
assert 'glim_sensor_pair=velodyne+cpt7_imu' in meta
report=dict(passed=True,mode=mode,raw_scans=len(clouds),published=len(poses),
 normalized_stamp_max_error_s=max(errors),first_raw_scan_index_published=indices[0],
 last_raw_scan_index_published=indices[-1],all_published_scan_intervals_imu_bracketed=True,
 signed_times_native_autoconf_observed=True,body_frame='cpt7_imu',
 limit='Verifies existing timing route and available IMU bounds, not a new independent deskew-equation audit.')
with (S/'velodyne_validation.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
