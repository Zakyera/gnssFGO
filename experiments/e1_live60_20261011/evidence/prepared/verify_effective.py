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
assert os.environ['CBSMS_BAG_DURATION_SEC'] == '60.0'
enabled = os.environ['CBSMS_CBS_ENABLE'] == 'true'
assert os.environ['CBSMS_CBS_PROTOCOL'] == 'classic'
assert os.environ['CBSMS_CBS_LIVE_REVISION_ENABLE'] == 'false'
assert os.environ.get('KIMERA_IMU_SAMPLE_SUPPORT', 'current') == 'current'
assert os.environ['KIMERA_CBS_REVISION_AUDIT'] == '1'
glim = yaml.safe_load((source/'resolved_glim_params.yaml').read_text())
kimera = yaml.safe_load((source/'resolved_kimera_params.yaml').read_text())
expected = dict(glim_cbs_experimental_local_gpu_batching=True, glim_cbs_mode='active_window_temporary' if enabled else 'observe_only', glim_cbs_live_revision_enable=False,
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
assert kimera['cbs_belief_receive_enable'] is enabled
assert kimera['cbs_belief_publish_enable'] is True
assert kimera['rerun_factor_graph_inspector_enable'] is False
master = rosgraph.Master('/active_window40_preflight')
uri = master.lookupNode('/kimera_vio_ros/kimera_vio_ros_node')
code, _, pid = xmlrpc.client.ServerProxy(uri).getPid('/active_window40_preflight')
assert code == 1
argv = Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
argv = [a.decode() for a in argv if a]
flags = dict(a[2:].split('=',1) for a in argv if a.startswith('--') and '=' in a)
expected_flags = dict(cbs_odom_factor_mode='active_window_temporary',
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
assert '/workspace/cbs_gtsam4.3/build/kimera_vio/libkimera_vio.so' in loaded
assert '/workspace/cbs_gtsam4.3/devel/lib/libkimera_vio.so' not in loaded
glim_uri=master.lookupNode('/glim_rosnode')
glim_pid=xmlrpc.client.ServerProxy(glim_uri).getPid('/active_window40_preflight')[2]
glim_loaded=Path('/proc/%d/maps' % glim_pid).read_text()
assert str(root/'private_build/libglim_ros.so') in glim_loaded
assert '/workspace/cbs_gtsam4.3/devel/lib/libglim_ros.so' not in glim_loaded
assert '/workspace/cbs_gtsam4.3/devel/libglim.so' in glim_loaded
assert str(root/'private_build/libgtsam_points_cuda.so.1.2.1') in glim_loaded
assert str(root/'private_build/libcbs.so') in glim_loaded
with (source/'effective_mode_check.json').open('x') as f:
    json.dump(dict(passed=True, checked_before_playback=True, glim=expected,
                   kimera=expected_flags, kimera_pid=pid, kimera_argv=argv,
                   kimera_maps=loaded,glim_maps=glim_loaded), f, indent=2)
print('Active-window mode and unchanged Gaussian source settings verified before playback.')
