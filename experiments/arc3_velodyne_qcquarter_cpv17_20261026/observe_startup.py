"""Save read-only postlaunch identity/progress observation; not a completed-run gate."""
import datetime,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_arc3_velodyne_qcquarter_cpv17_20261026/prepared')
assert not (H/'startup_observation.json').exists()
status=json.loads((H/'arc3/status.json').read_text());assert status['state']=='running'
S=P/'full_off_source';R=P/'full_off_receiver'
effective=json.loads((S/'effective_mode_check.json').read_text())
sensor=json.loads((S/'velodyne_startup.json').read_text())
assert effective['passed'] and sensor['passed']
assert effective['glim']['glim_cbs_receive_external_beliefs'] is False
assert effective['glim']['glim_cbs_experimental_local_gpu_batching'] is True
assert effective['kimera_candidate_env']==dict(CPK9_PRIOR_REFRESH='1',FL4Q_ENABLE='0',FL4M_RETIREMENT='remove')
pid=int((R/'receiver.pid').read_text())
exe=subprocess.check_output(['docker','exec','ubuntu-jammy-humble-nvidia','readlink',f'/proc/{pid}/exe'],text=True).strip()
assert exe=='/home/yeranis/results/gnssfgo_leica2_motion_prior_quarter_cpv15_20261026/live_trace'
digest=subprocess.check_output(['docker','exec','ubuntu-jammy-humble-nvidia','sha256sum',f'/proc/{pid}/exe'],text=True).split()[0]
assert digest=='9da4f4d0a8ab76498ff3a282afd3e06af5c6a337f20ce99c2b0e07cdc2aa2ed1'
transport=json.loads((R/'transport_status.json').read_text())
assert min(transport['sources'].values())>0 and transport['published_poses']>0
assert transport['max_queue']<1000 and transport['duplicates']==0
assert not any((R/f).exists() for f in ('failed.txt','transport_failed.txt'))
assert not (P/'full_off_control/failed.txt').exists()
result=dict(observed=datetime.datetime.now().astimezone().isoformat(),startup_passed=True,
    completed_run=False,status=status,receiver_pid=pid,actual_receiver_exe=exe,actual_receiver_sha256=digest,
    effective_parameters_passed=True,velodyne_sensor_profile_passed=True,transport=transport,
    qc_multiplier_original=.25,first_published_scoring_planned=True)
with (H/'startup_observation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
