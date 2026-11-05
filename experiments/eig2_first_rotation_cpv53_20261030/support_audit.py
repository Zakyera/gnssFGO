"""Supplemental source provenance and first-edge ordering; no solver."""
import csv,hashlib,json,re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name
P=W/'runs/eig2_hesai_insertion_cpv51_20261030/prepared'
S=P/'full_on_source'
pins={}
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
 p=Path(p).resolve();pins[str(p)]=sha(p);return p
def save(n,d):
 with (H/n).open('x') as f:json.dump(d,f,indent=2,allow_nan=False)
a=json.loads(pin(H/'analysis.json').read_text())
assert a['exports']['off']['times']==a['exports']['on']['times']
with pin(K/'extracted/pim.csv').open() as f:rows=list(csv.DictReader(f))
r=next(r for r in rows if r['from']=='4' and r['to']=='5')
initial=np.array([r['initialR'+str(i)] for i in range(9)],float).reshape(3,3)
reported=np.array(a['rotations']['Kimera_before_insertion']['matrix'])
parity=float(np.degrees(Rotation.from_matrix(initial.T@reported).magnitude()))
assert parity<1e-10

logs=[p for p in (S/'ros_logs').glob('*/glim_rosnode-1-stdout.log') if p.parent.name!='latest']
assert len(logs)==1
lines=pin(logs[0]).read_text().splitlines()
export_line=a['exports']['on']['line']
events=[]
for i,line in enumerate(lines[:export_line],1):
 if 'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,' not in line:continue
 row=line.split('GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,',1)[1]
 row=re.sub(r'\x1b\[[0-9;]*m','',row).strip()
 if row.startswith(('inserted,','slot_tracked,','removed,')):
  times=re.search(r'\[INFO\] \[([0-9.]+), ([0-9.]+)\]',line)
  events.append(dict(line=i,row=row,wall_s=times[1],sensor_clock_s=times[2]))
assert [e['row'] for e in events if e['row'].startswith('inserted,')]==['inserted,x4,x6,total_active=1']
assert any(e['row']=='slot_tracked,x4,x6,slot=45,total_active=1' for e in events)
assert not any(e['row'].startswith('removed,') for e in events)
export=re.search(r'\[INFO\] \[([0-9.]+), ([0-9.]+)\]',lines[export_line-1])
timeline=dict(log=str(logs[0]),prior_events=events,
              export=dict(line=export_line,wall_s=export[1],sensor_clock_s=export[2]),
              conclusion='One K2G factor x4->x6 was already present before GLIM5->7 export; not an untouched ON LIO estimate. OFF same-edge comparison is separate, not a causal removal experiment.')

protected=json.loads(pin(H/'protected_manifest.json').read_text())
sources=[W/'src/glim_ros1/src/glim_ros/cbs_bridge.cpp',
 W/'src/glim_ros1/src/glim_ros/glim_ros.cpp',
 W/'src/glim/src/glim/odometry/odometry_estimation_imu.cpp',
 W/'src/Kimera-VIO-ROS/src/KimeraVioRos.cpp',
 W/'src/Kimera-VIO/include/kimera-vio/imu-frontend/CbsNativeImuMeanBridge.h',
 W/'src/Kimera-VIO/src/imu-frontend/ImuFrontend.cpp',
 W/'src/Kimera-VIO/src/dataprovider/DataProviderModule.cpp',
 W/'src/Kimera-VIO/src/utils/ThreadsafeImuBuffer.cpp',
 W/'runs/eig2_hesai_insertion_cpv51_20261030/VioBackend.cpp']
provenance=[]
for p in sources:
 pin(p);old=protected.get(str(p));actual=pins[str(p)]
 if old is not None:assert old==actual,p
 provenance.append(dict(path=str(p),sha256=actual,protected_hash_match=(old==actual if old else None),
  use='Pinned source corroboration' if old else 'Current-source explanation only; historical source hash not established by this manifest. Runtime numeric/frame checks are separate.'))
for p in [S/'grandtour_launch_used.launch',S/'resolved_kimera_params.yaml',S/'resolved_glim_params.yaml',
          S/'kimera_profile/ImuParams.yaml',S/'glim_config/config_ros.json',S/'glim_config/config_sensors.json',
          S/'effective_mode_check.json']:
 pin(p)
saved=(S/'grandtour_launch_used.launch').read_text()
assert '<arg name="glim_input_imu_topic" default="/boxi/cpt7/imu"/>' in saved
assert '<arg name="kimera_input_imu_topic" default="/boxi/cpt7/imu"/>' in saved
frame_note=('Payload header map is configured world label. Both body configurations cpt7_imu, '
 'saved PIM body rotation identity. Incoming raw rotation equals native MAIN endpoint relative '
 'rotation; actual inserted rotation equals reconstructed native bridge. This audits this edge, '
 'not all calibration or binary source semantics.')
save('support_audit.json',dict(passed=True,initial_rotation_parity_deg=parity,timeline=timeline,
 source_provenance=provenance,frame_note=frame_note,input_sha256=pins,
 limitation='Not all current explanatory source files have historical source-hash provenance; no blanket source/binary identity claim. GLIM contemporaneous bias/PIM was not saved.'))
print(json.dumps(dict(passed=True,initial_rotation_parity_deg=parity,timeline=timeline,
                     protected_sources=sum(x['protected_hash_match'] is True for x in provenance),
                     explanatory_unpinned_sources=sum(x['protected_hash_match'] is None for x in provenance)),indent=2))
