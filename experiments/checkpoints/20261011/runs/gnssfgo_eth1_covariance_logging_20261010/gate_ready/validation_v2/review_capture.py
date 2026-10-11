"""Independent completion review after the inherited 100-pose shell guard.

Never creates/replaces source complete.marker or controller complete.json.
All original failure evidence remains intact. No source/receiver is launched.
"""
from pathlib import Path
import csv
import hashlib
import json
import re
import rosbag

ROOT=Path(__file__).resolve().parent.parent
source=ROOT/'gate_on_source'; control=ROOT/'gate_on_control'; live=ROOT/'gate_on_receiver'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
protected=json.loads((ROOT/'protected_validation.json').read_text())
assert protected['passed'] and protected['manifest_sha256']==sha(ROOT/'protected_manifest.json')
assert not (source/'complete.marker').exists() and not (control/'complete.json').exists()
assert (control/'failed.txt').read_text().strip()=='source failed'
assert (live/'complete.marker').exists()
coverage=json.loads((ROOT/'coverage_validation.json').read_text())
assert coverage['passed']
assert coverage['trajectories']['native_kimera']['span_s']>=9.0
assert 'Done.' in (source/'rosbag.log').read_text()
assert json.loads((source/'effective_mode_check.json').read_text())['passed']
with (source/'glim_odometry.csv').open() as f:grows=list(csv.DictReader(f))
with (source/'kimera_odometry.csv').open() as f:krows=list(csv.DictReader(f))
error=f'Insufficient estimator output: GLIM={len(grows)}, Kimera={len(krows)}.'
assert error in (control/'source.log').read_text() and len(grows)<100
used=(source/'experiment_runner_used.sh').read_text()
assert 'min_glim_rows=100' in used and 'if [[ "${duration_sec}" == 8.0 ]]' in used
assert '"${duration_sec}" == 10.0' in used
logs=[p for p in (source/'ros_logs').glob('*/*stdout.log') if p.parent.name!='latest']
texts=[p.read_text() for p in logs]+[(source/'estimators.log').read_text()]
text='\n'.join(texts)
for token in ['terminate called','process has died','MEAN_ALIGNMENT_UNAVAILABLE',
              'failed to build external odometry factor']:
    assert token not in text,token
assert 'shutting down processing monitor complete' in texts[-1]
g2k=sum('CBS_BPSAM_ODOM_ADD_ROW_G2K' in l and 'sender_odom_queued' in l for l in text.splitlines())
k2g=sum('GLIM_CBS_ODOM_INJECT_ROW' in l and 'active_window_temporary_graph_queued' in l for l in text.splitlines())
with rosbag.Bag(str(source/'recordings/cbs.bag')) as bag:
    wire=bag.get_message_count('/kimera/cbs/odom_belief_in')
targets=[int(m[1]) for m in re.finditer(r'CBS_ODOM_FACTOR_COVARIANCE_ROW,G2K,k,g,x\d+,x(\d+),[^\n]*active_window_temporary_odom_applied',text)]
assert wire>0 and g2k>0 and k2g>0 and targets
tail=int(krows[-1]['field.header.seq'])-max(targets)
assert tail<=15
status=json.loads((live/'transport_status.json').read_text())
assert status['queued']==0 and status['imu']>0 and all(status['sources'].values())
summary=json.loads((live/'receiver/summary.json').read_text())
assert summary['pending_at_end']==0
metadata=dict(cbs_protocol='classic',cbs_covariance='schur_relative_between',presentation_enable='false',
    glim_receive_cbs_beliefs='true',kimera_receive_cbs_beliefs='true',sensor_window_sec='10.0',
    bag_rate='0.10',glim_cbs_mode='active_window_temporary',kimera_cbs_odom_factor_mode='active_window_temporary')
env=(ROOT/'source.env').read_text()
for field,value in [('CBSMS_CBS_PROTOCOL','classic'),('CBSMS_BAG_RATE','0.10'),
                    ('CBSMS_PRESENTATION_ENABLE','false')]:
    assert f'export {field}={value}' in env
out=dict(captured_data_complete=True,original_harness_status='FAILED_POSTCHECK_ROW_THRESHOLD',
    original_failure_preserved=True,no_estimator_rerun=True,
    evidence=['rosbag Done', 'normal source shutdown', 'native timestamp/tail coverage',
              'nonzero bidirectional source CBS', 'receiver drained with zero pending',
              'protected preflight hashes and effective parameters unchanged'],
    glim_rows=len(grows),kimera_rows=len(krows),g2k_wire_messages=wire,g2k_queued=g2k,k2g_queued=k2g,
    g2k_tail_gap=tail,checked_metadata_reconstructed_from_frozen_config=metadata,
    limitation='Reconstructed metadata is not the absent source run_metadata.txt. '
               'Full numerical/covariance/lifecycle validation remains separate.')
with (ROOT/'capture_review.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
