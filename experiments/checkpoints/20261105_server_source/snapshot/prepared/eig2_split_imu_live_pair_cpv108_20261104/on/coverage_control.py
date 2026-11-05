"""Exact saved input/initialization/EOF reconciliation, not a guessed frame count."""
import csv,json,re,sys
from pathlib import Path
import numpy as np
import coverage_checks as legacy
S=Path(sys.argv[1]);P=Path(__file__).resolve().parent
a=json.loads((P/'input_audit.json').read_text());wanted=json.loads((P/'expected_headers.json').read_text());imu={}
duration=a['duration_s'];assert duration==355.02
for topic,file in [('/boxi/cpt7/imu','cpt7_headers.csv'),('/boxi/alphasense/imu','alphasense_headers.csv')]:
    with (S/file).open() as f:rows=list(csv.DictReader(f))
    key=next(k for k in rows[0] if k.endswith('.stamp'));ts=[int(r[key]) for r in rows]
    assert ts==wanted[topic] and np.all(np.diff(ts)>0)
    imu[topic]=dict(rows=len(ts),first_ns=ts[0],last_ns=ts[-1],all_raw_stamps_equal=True)
with (S/'kimera_odometry.csv').open() as f:rows=list(csv.DictReader(f))
assert {r['field.child_frame_id'] for r in rows}=={'alphasense_imu'}
kt=np.asarray([int(r['field.header.stamp']) for r in rows],np.int64)
assert [int(r['field.header.seq']) for r in rows]==list(range(len(rows)))
cam=np.asarray(a['stereo_pair_stamps_ns'],np.int64)
assert list(cam)==sorted(set(wanted['/boxi/alphasense/front_left/image_raw/compressed'])&set(wanted['/boxi/alphasense/front_right/image_raw/compressed']))
assert np.isin(kt,cam).all();init_index=int(np.flatnonzero(cam==kt[0])[0]);assert init_index<=15
lastimu=imu['/boxi/alphasense/imu']['last_ns'];processable=(cam>=kt[0])&(cam<=lastimu)
text=(S/'estimators.log').read_text();front=[int(x) for x in re.findall(r'VioFrontend \[ms\]\s+(\d+)',text)]
left=[int(x) for x in re.findall(r'data_provider_left_frame_queue Size \[#\]\s+(\d+)',text)]
right=[int(x) for x in re.findall(r'data_provider_right_frame_queue Size \[#\]\s+(\d+)',text)]
assert max(left)==max(right)==len(cam)==a['stereo_pairs']
assert max(front)==int(processable.sum()),(max(front),int(processable.sum()))
assert int(re.search(r'Initialize Pipeline: timestamp = (\d+)',text)[1])==kt[0]
updates=set(map(int,re.findall(r'KIMERA_BACKEND_SPINONCE_TIMING_ROW,(\d+),',text)))
assert sorted(updates)==list(range(len(rows)))
logpaths={p.resolve() for p in [S/'estimators.log',*S.glob('ros_logs/*/*stdout.log')]}
alltext='\n'.join(p.read_text() for p in sorted(logpaths))
assert not re.search(r'IndeterminantLinearSystem|terminate called|Assertion.*failed|local_update_failure|\[FATAL\]|CPV40.*FAIL',alltext)
assert 'persistent_odom_applied' in alltext and 'injected_persistent' in alltext and 'sender_odom_queued' in alltext
assert 'injected_active_window' not in alltext
injections={tag:len(re.findall(pattern,alltext)) for tag,pattern in {
    'G2K_queued':r'CBS_BPSAM_ODOM_ADD_ROW_G2K[^\n]*sender_odom_queued',
    'G2K_persistent':r'CBS_ODOM_FACTOR_COVARIANCE_ROW,G2K[^\n]*persistent_odom_applied',
    'K2G_persistent':r'GLIM_CBS_ODOM_INJECT_ROW[^\n]*injected_persistent'}.items()}
assert all(injections.values()),injections
result=dict(passed=True,requested_s=duration,input_receipts=imu,both_receivers_on=True,external_injections=injections,
    raw_unmatched_camera_headers=a['raw_unmatched_camera_headers'],
    frontend=dict(received_pairs=len(cam),preinitialization=init_index,processed=max(front),terminal_unsupported=int((cam>lastimu).sum()),
    terminal_camera_after_last_imu_ms=float((cam[-1]-lastimu)*1e-6)),backend_rows=len(rows),
    limits='Native counters and exact timestamps, not an instrumented per-frame branch trace')
assert result['frontend']['received_pairs']==init_index+max(front)+int((cam>lastimu).sum())
for role,file in [('native_glim','odom_imu.txt'),('native_kimera','kimera_odometry.csv')]:
    v=imu['/boxi/alphasense/imu' if role=='native_kimera' else '/boxi/cpt7/imu'];result[role]=legacy.check(*legacy.load(S/file,role=='native_kimera'),duration,v['first_ns']*1e-9,v['last_ns']*1e-9,role)
assert 'Done.' in (S/'rosbag.log').read_text()
with (S/'source_coverage.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
