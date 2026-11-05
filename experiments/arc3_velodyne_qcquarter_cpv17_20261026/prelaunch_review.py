"""Read-only harness/scoring tests, one new review receipt; no sensor capture."""
import ast,hashlib,importlib.util,json,shutil,socket,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_arc3_velodyne_qcquarter_cpv17_20261026/prepared')
V=Path('/home/yeranis/results/gnssfgo_arc3_velodyne_qcquarter_cpv17_20261026')
assert not (H/'prelaunch_review.json').exists() and not V.exists()
assert not (H/'arc3/status.json').exists() and not any(P.glob('full_*_control'))
spec=importlib.util.spec_from_file_location('evaluation',P/'evaluate_fl4f.py')
e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e);a=e.a;n=e.np
tests=a.tests();assert tests['passed']
t=n.arange(501,dtype=n.int64)*a.STEP
p=n.c_[n.arange(501)*.02,n.sin(n.arange(501)*.01),n.zeros(501)]
r=e.b.R.from_euler('z',(n.arange(501)*.001)[:,None]).as_matrix()
curve=(t,p,r);valid=n.ones(len(t),bool);valid[200:215]=False
score=e.score(curve,curve,t,valid)
assert score['ATE_RMSE_m']<1e-12 and score['translation_RPE_1s_RMSE_m']<1e-12
assert score['rpe_pairs']==466
assert e.b.length(p,valid)<float(n.linalg.norm(n.diff(p,axis=0),axis=1).sum())
sys.path.insert(0,str(P));import input_coverage as ic
pre=json.loads((P/'input_audit.json').read_text());it=ic.self_test()
inventory=ic.check_available([c['normalized_stamp_s'] for c in pre['clouds'][1:]],pre)
manifest=json.loads((P/'protected_manifest.json').read_text())
for path,digest in manifest.items():
    with Path(path).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==digest,path
for script in [*P.glob('*.py'),*(H/'arc3').glob('*.py')]:ast.parse(script.read_text())
configs=[json.loads((P/f'full_{m}.json').read_text()) for m in ('off','on')]
for c in configs:
    assert c['receiver_binary']==str(P/'receiver_cpk19.sh')
    assert c['receiver_fixed_lag_seconds']==7 and c['support_period_ns']==100000000
    for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
    for source in c['sources']:source.pop('session')
assert configs[0]==configs[1]
wrapper=(P/'receiver_cpk19.sh').read_text()
assert '/gnssfgo_leica2_motion_prior_quarter_cpv15_20261026/live_trace' in wrapper
assert '--absorbed-revision-policy seal_at_absorption' in wrapper
for mode in ('off','on'):
    runner=(P/('source_runner.sh' if mode=='off' else 'source_runner_on.sh')).read_text()
    assert '726.38' in runner and 'min_glim_rows=7008' in runner
    assert 'rostopic echo -p /glim/cbs/odometry' in runner
containers=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).split()
assert containers==['exited','exited']
with socket.socket() as s:s.bind(('127.0.0.1',11488))
free=shutil.disk_usage(P).free;assert free>22*1024**3
checkpoint=subprocess.run(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],capture_output=True,text=True,check=True)
review=dict(passed=True,geometry_tests=tests,identity_gap_score=score,input_negative_tests=it,
    raw_inventory_test=inventory,protected_files=len(manifest),containers=containers,
    qc_multiplier_original=.25,receiver_model_delta='CP-V15 verified one-line Qc*0.25, unchanged initial priors',
    primary_products=['GLIM first-published CSV','Kimera native ROS','Fusion ONLINE'],
    free_bytes=free,pair_forecast_bytes=14*1024**3,reserve_bytes=8*1024**3,
    checkpoint_stdout=checkpoint.stdout,window=pre['window'],
    setup_notes=['First preparation command denied socket by sandbox before any outputs; authorized elevated preflight then passed',
                 'Synthetic test driver corrected single-axis scipy angle shape; production geometry unchanged',
                 'Admission v1 terminal IMU bracket failure preserved; v2 adds10ms boundary, no source retry'])
with (H/'prelaunch_review.json').open('x') as f:json.dump(review,f,indent=2,default=a.serial)
print(json.dumps(review,default=a.serial))
