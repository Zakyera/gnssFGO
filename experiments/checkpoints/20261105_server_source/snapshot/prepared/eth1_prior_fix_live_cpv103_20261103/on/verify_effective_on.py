#!/usr/bin/env python3
"""Run by the PRIVATE source runner after startup, BEFORE any bag playback."""
import json
import os
from pathlib import Path
import sys
import xmlrpc.client
import yaml
import rosgraph

root = Path(__file__).resolve().parent
source = Path(sys.argv[1])
assert os.environ['CBSMS_BAG_DURATION_SEC'] == '346.32'
assert os.environ['CBSMS_BAG_RATE'] == '0.20'
assert os.environ['CBSMS_MISSION']=='2024-10-01-11-29-55'
assert os.environ['CBSMS_BAG_START_SEC']=='0'
enabled = os.environ['CBSMS_CBS_ENABLE'] == 'true'
assert enabled, 'CP-V91 requires BOTH source receivers enabled'
assert os.environ['CBSMS_KIMERA_MODE'] == 'stereo'
assert os.environ['CBSMS_CBS_PROTOCOL'] == 'classic'
assert os.environ['CBSMS_CBS_LIVE_REVISION_ENABLE'] == 'false'
assert os.environ.get('KIMERA_IMU_SAMPLE_SUPPORT', 'current') == 'current'
assert os.environ['KIMERA_CBS_REVISION_AUDIT'] == '1'
glim = yaml.safe_load((source/'resolved_glim_params.yaml').read_text())
kimera = yaml.safe_load((source/'resolved_kimera_params.yaml').read_text())
expected = dict(glim_cbs_experimental_local_gpu_batching=True, glim_cbs_mode='inject_persistent', glim_cbs_live_revision_enable=False,
    glim_cbs_active_factor_diagnostics_enable=False,
    glim_cbs_active_factor_diagnostics_stride=20,
    glim_cbs_active_factor_diagnostics_max_updates=40,
    glim_cbs_experimental_native_imu_mean_alignment=True,
    glim_cbs_receiver_local_competition_audit_max_samples=40,
    glim_cbs_receiver_local_competition_audit_stride=20,
    glim_cbs_odom_covariance_mode='schur_relative_between', glim_cbs_outgoing_marginal_source='direct',
    glim_cbs_k2g_odom_factor_covariance_scale=1.0, glim_cbs_health_aware_enable=False,
    glim_cbs_receive_external_beliefs=enabled, glim_cbs_publish_beliefs=True,
    glim_cbs_odom_max_horizon_pairs_per_update=1, glim_cbs_odom_horizon_sec=.2,
    glim_factor_graph_inspector_enable=False, glim_cbs_export_endpoint_audit_enable=True)
for key, value in expected.items():
    assert glim[key] == value, (key, glim.get(key), value)
assert glim['glim_cbs_receive_external_beliefs'] is True
import subprocess
glim_bus = subprocess.check_output(['rosnode', 'info', '/glim_rosnode'], text=True)
subscriptions = glim_bus.split('Subscriptions:', 1)[1].split('Publications:', 1)[0]
assert '/boxi/cpt7/imu' in subscriptions and '/boxi/alphasense/imu' not in subscriptions
assert kimera['cbs_belief_receive_enable'] is enabled
assert kimera['cbs_belief_publish_enable'] is True
assert kimera['rerun_factor_graph_inspector_enable'] is False
kimera_bus = subprocess.check_output(['rosnode', 'info', '/kimera_vio_ros/kimera_vio_ros_node'], text=True)
kimera_subscriptions = kimera_bus.split('Subscriptions:', 1)[1].split('Publications:', 1)[0]
assert '/boxi/alphasense/imu' in kimera_subscriptions and '/boxi/cpt7/imu' not in kimera_subscriptions
assert kimera['base_link_frame_id']=='alphasense_imu'
assert kimera['cbs_external_pose_frame_id']=='cpt7_imu'
import tf
import rospy
rospy.init_node('cpv91_frame_preflight', anonymous=True)
listener=tf.TransformListener()
import time
deadline=time.monotonic()+10
while not listener.canTransform('alphasense_imu','cpt7_imu',rospy.Time(0)):
    assert time.monotonic()<deadline, 'Static body transform not available before playback'
    time.sleep(.05)
translation, quaternion=listener.lookupTransform('alphasense_imu','cpt7_imu',rospy.Time(0))
import numpy as np
matrix=tf.transformations.quaternion_matrix(quaternion);matrix[:3,3]=translation
audit=json.loads((root/'input_audit.json').read_text())
assert np.max(abs(matrix-np.asarray(audit['T_alphasense_cpt7'])))<1e-9
master = rosgraph.Master('/active_window40_preflight')
uri = master.lookupNode('/kimera_vio_ros/kimera_vio_ros_node')
code, _, pid = xmlrpc.client.ServerProxy(uri).getPid('/active_window40_preflight')
assert code == 1
argv = Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
argv = [a.decode() for a in argv if a]
flags = dict(a[2:].split('=',1) for a in argv if a.startswith('--') and '=' in a)
expected_flags = dict(cbs_odom_factor_mode='persistent',
    cbs_experimental_native_imu_mean_alignment='true',
    cbs_covariance_exchange_audit_max_samples='40',
    cbs_covariance_exchange_audit_stride='20',
    cbs_use_temporary_cbs_linear_factors='false', cbs_live_revision_enable='false',
    cbs_odom_covariance_mode='schur_relative_between', cbs_l2k_odom_factor_covariance_scale='1.0',
    cbs_health_aware_enable='false', cbs_use_local_smoother_for_belief_covariance='true',
    cbs_use_persistent_bpsam_sidecar_for_belief_covariance='false')
for key, value in expected_flags.items():
    assert flags.get(key) == value, (key, flags.get(key), value)
loaded = Path('/proc/%d/maps' % pid).read_text()
assert str(root/'private_build/libkimera_vio.so') in loaded
assert '/workspace/cbs_gtsam4.3/build/kimera_vio/libkimera_vio.so' not in loaded
assert '/workspace/cbs_gtsam4.3/devel/lib/libkimera_vio.so' not in loaded
glim_uri=master.lookupNode('/glim_rosnode')
glim_pid=xmlrpc.client.ServerProxy(glim_uri).getPid('/active_window40_preflight')[2]
glim_loaded=Path('/proc/%d/maps' % glim_pid).read_text()
candidate = Path("/workspace/cbs_gtsam4.3/runs/shared_linearization_s4_provider_20261012/release_2")
from map_guard import require_glim_libraries
verified_dso_paths = require_glim_libraries(glim_loaded, candidate)
assert str(root/'private_build') not in glim_loaded
assert str(candidate) not in loaded and 'libe1b_profile' not in loaded
assert str(root/'private_build/libcbs.so') in loaded
def env(pid):
    return dict(x.decode().split('=',1) for x in Path('/proc/%d/environ'%pid).read_bytes().split(b'\0') if b'=' in x)
assert env(pid)['FL4Q_ENABLE']=='0'
assert env(pid)['CPK9_PRIOR_REFRESH']=='1'
archive_dir=Path(env(pid)['CPV102_PRIOR_FAILURE_DIR']).resolve()
assert archive_dir==source/'prior_failures' and archive_dir.is_dir() and os.access(archive_dir,os.W_OK)
assert 'CPV102_PRIOR_FAILURE_DIR' not in env(glim_pid)
assert not list(archive_dir.iterdir())
assert env(pid)['CPV40_REFRESH']=='1' and env(pid)['CPV38_OFF_REFRESH']=='0'
assert 'CPV40_REFRESH' not in env(glim_pid)
assert env(pid)['FL4M_RETIREMENT']=='remove'
assert env(pid)['CPV76_PERSISTENT_COMPAT']=='1' and env(pid).get('CPV69_RETAIN_COMPAT','0')=='0'
assert 'CPV76_PERSISTENT_COMPAT' not in env(glim_pid)
assert 'KIMERA_IMU_SAMPLE_SUPPORT' not in env(pid)
assert not any(env(pid).get(k,'0') not in ('','0') for k in ('CPV70_RECORDED_UPDATE_SCHEDULE','CPV73_REPLAY_READY_TIMEOUT_MS'))
assert Path(env(pid)['FL4Q_LOG']).resolve()==source/'acceptance.csv'
assert Path(env(pid)['FL4M_AUDIT']).resolve()==source/'objective.csv'
assert 'FL4Q_ENABLE' not in env(glim_pid)
assert 'CPK9_PRIOR_REFRESH' not in env(glim_pid)
assert env(glim_pid)['CBS_E1B_PROFILE_PATH'] == str(source/'e1b_profile.jsonl')
assert 'CBS_E1B_PROFILE_PATH' not in env(pid)
assert env(glim_pid)['CBS_E1B_LOCAL_UPDATE_GPU_BATCHING']=='1'
assert env(glim_pid)['CPK16_MAIN_GPU_BATCHING']=='1'
assert 'CPK16_MAIN_GPU_BATCHING' not in env(pid)
assert 'LD_PRELOAD' not in env(glim_pid)
assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'
assert env(glim_pid)['CPV82_GLIM_PERSISTENT_COMPAT']=='1'
assert 'CPV82_GLIM_PERSISTENT_COMPAT' not in env(pid)
assert env(glim_pid)['GLIM_REVISION_COVARIANCE_POLICY']=='native_local'
assert 'GLIM_REVISION_COVARIANCE_POLICY' not in env(pid)
import hashlib
profile_checks={}
for original in (root/'input_profiles/kimera_profile').rglob('*'):
    if original.is_file():
        relative=original.relative_to(root/'input_profiles/kimera_profile')
        actual=source/'kimera_profile'/relative
        assert actual.read_bytes()==original.read_bytes(), str(relative)
        profile_checks[str(relative)]=hashlib.sha256(actual.read_bytes()).hexdigest()
imu_text=(source/'kimera_profile/ImuParams.yaml').read_text().replace('%YAML:1.0','')
imu_params=yaml.safe_load(imu_text)
assert imu_params['gyroscope_random_walk']==0.000266
assert imu_params['accelerometer_random_walk']==0.0043
assert imu_params['rate_hz']==200
assert imu_params['gyroscope_noise_density']==0.00020202932517334086
assert imu_params['accelerometer_noise_density']==0.0012593921763625943
assert hashlib.sha256(Path('/workspace/cbs_gtsam4.3/runs/glim_persistent_compat_cpv82_20261101/release_4/libglim_ros.so').read_bytes()).hexdigest()=='aecc729f4bf51548950b8cb997c3dcd474e30cabe23785c4011e6b22f7dbe918'
assert hashlib.sha256((root/'private_build/libkimera_vio.so').read_bytes()).hexdigest()=='985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09'
assert hashlib.sha256(Path('/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019/libglim_ros.so').read_bytes()).hexdigest()=='42df73c172232a612da965bf43ead8ec454caf08434b84e2da5de74d26cdb79f'
assert hashlib.sha256(Path('/workspace/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/release_1/libgtsam_points.so.1.2.1').read_bytes()).hexdigest()=='6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca'
assert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)
assert 'GLIM_SHARED_NATIVE_PROVIDER' not in env(pid)
assert 'CBS_E1B_LOCAL_UPDATE_GPU_BATCHING' not in env(pid)
assert not any('SNAPSHOT' in k or 'REAL_INPUT' in k or 'GLIM_E1_REAL_AUDIT_ROOT' == k for k in env(glim_pid))
assert 'e1b4a_real_input' not in glim_loaded
with (source/'effective_mode_check.json').open('x') as f:
    json.dump(dict(passed=True, checked_before_playback=True, playback_rate=0.20, requested_s=346.32, glim=expected,
                   actual_profile_sha256=profile_checks,actual_imu_parameters=imu_params,
                   kimera_body='alphasense_imu',glim_body='cpt7_imu',wire_body='cpt7_imu',
                   T_alphasense_cpt7=matrix.tolist(),kimera_receive=True,glim_receive=True,
                   kimera=expected_flags, kimera_native_only_outgoing_information=False, mixed_prior_feedback_user_accepted=True, kimera_candidate_env={k:env(pid)[k] for k in ('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT','CPV40_REFRESH','CPV38_OFF_REFRESH','CPV76_PERSISTENT_COMPAT')}, prior_failure_archive=str(archive_dir), private_kimera_sha256='985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09', kimera_pid=pid, kimera_argv=argv,
                   kimera_maps=loaded,glim_maps=glim_loaded,glim_persistent_compat=env(glim_pid)['CPV82_GLIM_PERSISTENT_COMPAT'],glim_revision_covariance='native_local',cached_provider_sha256='6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca'), f, indent=2)
print('CP-V91 split IMU topics/body transform and current private builds verified; BOTH CBS receivers ON, persistent.')
