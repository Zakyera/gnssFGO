"""Additional read-only sensor checks before playback, inside Noetic."""
from pathlib import Path
import hashlib,json,re,subprocess,sys
source=Path(sys.argv[1]);root=Path(__file__).resolve().parent
def load(p):return json.loads(re.sub(r'/\*.*?\*/|//[^\n]*','',p.read_text(),flags=re.S))
cfg=load(source/'glim_config/config_ros.json')['glim_ros']
sensor=load(source/'glim_config/config_sensors.json')['sensors']
assert cfg['points_topic']=='/boxi/hesai/points'
assert cfg['lidar_frame_id']=='hesai_lidar'
assert cfg['imu_topic']=='/boxi/cpt7/imu' and cfg['imu_frame_id']==cfg['base_frame_id']=='cpt7_imu'
assert cfg['imu_time_offset']==cfg['points_time_offset']==0
assert sensor['autoconf_perpoint_times'] and not sensor['autoconf_prefer_frame_time']
assert sensor['perpoint_relative_time'] and sensor['perpoint_time_scale']==1
assert not sensor['global_shutter_lidar']
want=[-.044611849471,.302238142205,-.012539945511,-.709330179038,.704851604038,.003921407290,-.004419949690]
assert sensor['T_lidar_imu']==want
for folder in ('glim_config','kimera_profile'):
 expected=root/'input_profiles'/folder
 for p in expected.rglob('*'):
  if p.is_file():assert p.read_bytes()==(source/folder/p.relative_to(expected)).read_bytes(),p
bus=subprocess.check_output(['rosnode','info','/glim_rosnode'],text=True)
subscriptions=bus.split('Subscriptions:',1)[1].split('Services:',1)[0]
assert '/boxi/hesai/points' in subscriptions and '/boxi/cpt7/imu' in subscriptions
assert '/anymal/velodyne/points' not in subscriptions and 'livox' not in subscriptions.lower()
result=dict(passed=True,checked_before_playback=True,topic=cfg['points_topic'],frame=cfg['lidar_frame_id'],
 body_frame='cpt7_imu',T_lidar_imu=want,deskew=True,native_hesai_timing_profile_unchanged=True,
 source_profiles_exact=True,subscriptions=subscriptions)
with (source/'hesai_startup.json').open('x') as f:json.dump(result,f,indent=2)
