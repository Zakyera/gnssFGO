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
assert os.environ['CBSMS_BAG_DURATION_SEC'] == '40.0'
assert os.environ['CBSMS_CBS_ENABLE'] == 'false'
assert os.environ['CBSMS_CBS_PROTOCOL'] == 'classic'
assert os.environ['CBSMS_CBS_LIVE_REVISION_ENABLE'] == 'false'
assert os.environ.get('KIMERA_IMU_SAMPLE_SUPPORT', 'current') == 'current'
assert os.environ['KIMERA_CBS_REVISION_AUDIT'] == '1'
glim = yaml.safe_load((source/'resolved_glim_params.yaml').read_text())
kimera = yaml.safe_load((source/'resolved_kimera_params.yaml').read_text())
expected = dict(glim_cbs_mode='observe_only', glim_cbs_live_revision_enable=False,
    glim_cbs_odom_covariance_mode='schur_relative_between', glim_cbs_outgoing_marginal_source='direct',
    glim_cbs_k2g_odom_factor_covariance_scale=1.0, glim_cbs_health_aware_enable=False,
    glim_cbs_receive_external_beliefs=False, glim_cbs_publish_beliefs=True,
    glim_cbs_odom_max_horizon_pairs_per_update=1, glim_cbs_odom_horizon_sec=.2,
    glim_factor_graph_inspector_enable=False, glim_cbs_export_endpoint_audit_enable=True)
for key, value in expected.items():
    assert glim[key] == value, (key, glim.get(key), value)
assert kimera['cbs_belief_receive_enable'] is False
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
    cbs_use_temporary_cbs_linear_factors='false', cbs_live_revision_enable='false',
    cbs_odom_covariance_mode='schur_relative_between', cbs_l2k_odom_factor_covariance_scale='1.0',
    cbs_health_aware_enable='false', cbs_use_local_smoother_for_belief_covariance='true',
    cbs_use_persistent_bpsam_sidecar_for_belief_covariance='false')
for key, value in expected_flags.items():
    assert flags.get(key) == value, (key, flags.get(key), value)
with (source/'effective_mode_check.json').open('x') as f:
    json.dump(dict(passed=True, checked_before_playback=True, glim=expected,
                   kimera=expected_flags, kimera_pid=pid, kimera_argv=argv), f, indent=2)
print('CBS-OFF receiving disabled and unchanged Gaussian source settings verified before playback.')

