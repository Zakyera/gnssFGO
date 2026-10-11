"""Constants for a single prepared experiment, not production defaults."""
from pathlib import Path
import hashlib

def file_sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

ROOT = Path(__file__).resolve().parent
REPO = Path('/home/yeranis/repos/gnssFGO')
BASE = ROOT.parents[1] / 'gnssfgo_eth1_live_wnoa_20261007'
OLD = BASE / 'bounded120_on_source'
REFERENCE = ROOT.parents[1] / 'gnssfgo_eth1_native_mean_alignment_20261010/gpu_restored'
PROFILE = ROOT.parents[1] / 'grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_cbs_off_full_346s_20260907'
BINARY = Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/replay')
ENV = dict(LD_LIBRARY_PATH='/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib',
           OMP_NUM_THREADS='1', WNOJ_STRICT_PUBLICATION='1', WNOJ_INCREMENTAL_PROFILE='nominal')

def inside(path):
    return '/workspace/' + str(Path(path).relative_to('/home/yeranis/repos/V4RL'))

def launch_arguments():
    return [
        'glim_config_path:=' + inside(PROFILE / 'glim_config'),
        'glim_dump_path:=' + inside(ROOT / 'pilot_on_source/glim_dump'),
        'kimera_params_folder:=' + inside(PROFILE / 'kimera_profile'),
        'kimera_mode:=stereo_shared_cpt7', 'kimera_body_frame_id:=cpt7_imu',
        'kimera_input_imu_topic:=/boxi/cpt7/imu', 'cbs_enable:=true',
        'glim_cbs_mode:=active_window_temporary', 'cbs_live_revision_enable:=false',
        'glim_publish_cbs_beliefs:=true', 'kimera_publish_cbs_beliefs:=true',
        'glim_receive_cbs_beliefs:=true', 'kimera_receive_cbs_beliefs:=true',
        'kimera_cbs_backend_enable:=true', 'enable_ekf:=false',
        'glim_factor_graph_inspector_enable:=false',
        'kimera_factor_graph_inspector_enable:=false',
        'cbs_odom_max_horizon_pairs_per_update:=1', 'cbs_odom_publish_stride:=1',
        'cbs_odom_horizon_sec:=0.20', 'cbs_belief_receive_start_delay_sec:=0.0',
        'glim_export_endpoint_audit_enable:=true', 'diagnostic_node_output:=log',
        'glim_receiver_local_competition_audit_max_samples:=40',
        'glim_receiver_local_competition_audit_stride:=20',
        'kimera_receiver_local_competition_audit_max_samples:=40',
        'kimera_receiver_local_competition_audit_stride:=20',
        'rerun_recording_id:=pilot_on_source',
        'kimera_log_output_path:=' + inside(ROOT / 'pilot_on_source/kimera_logs')]
