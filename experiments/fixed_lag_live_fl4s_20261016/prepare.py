"""Copy the validated live harness; substitute only isolated Kimera candidate."""
from pathlib import Path
import ast, hashlib, json, shutil, subprocess, difflib, importlib.util
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
O=W/'runs/gnssfgo_fixed_lag_live30_off_on_fl4f2_20261014/prepared'
Q=W/'runs/fixed_lag_current_chart_fl4q_20261016'
F=W/'runs/gnssfgo_fixed_lag_live30_off_on_fl4s_20261016/prepared'
R=W/'runs/gnssfgo_fixed_lag_live30_off_on_fl4s2_20261016/prepared'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
old=json.loads((O/'protected_manifest.json').read_text())
protected=json.loads((Q/'protected_before.json').read_text())
q=json.loads((Q/'pre_capture_receipt.json').read_text())
failed_preflight=json.loads((F/'protected_manifest.json').read_text())
assert not (F/'pilot30_off_source').exists()
assert not (F/'pilot30_off_receiver').exists()
assert not (F/'pilot30_off_control/source.log').exists()
assert not (F/'pilot30_off_control/receiver.log').exists()
assert 'FL4a policy requires the pinned fixed-lag wrapper' in (F/'pilot30_off_control/preflight.log').read_text()
for p,h in {**old,**protected,**q['files'],**failed_preflight}.items():assert sha(p)==h,p
assert sha(Q/'private_build/libkimera_vio.so')==q['library_sha']
for n in ['test_passed.json','sanitizer_passed.json']:
    assert json.loads((Q/n).read_text())['passed']
assert not R.parent.exists()
R.mkdir(parents=True)
shutil.copytree(O/'private_build',R/'private_build',symlinks=True)
shutil.copy2(Q/'private_build/libkimera_vio.so',R/'private_build/libkimera_vio.so')
files=['experiment.py','coverage_checks.py','validate_case.py','evaluate_fl4f.py',
 'compare_belief_cov_fl4f.py','preflight_on_fl4f.py','map_guard.py','source.env',
 'receiver_fl4e.sh','run_case.py','run_case_on.py','verify_effective.py',
 'verify_effective_on.py','source_runner.sh','source_runner_on.sh',
 'active_window.launch','active_window_on.launch','glim_profile.launch',
 'glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch',
 'pilot30_off.json','pilot30_on.json']
for n in files:
    s=(O/n).read_text().replace(O.parent.name,R.parent.name).replace('11434','11441')
    s=s.replace('gnssfgo_fixed_lag_live_fl4f_20261014','gnssfgo_fixed_lag_live_fl4s_20261016')
    s=s.replace('eth1_fl4f2_live30_','eth1_fl4s_live30_').replace('_20261014"','_20261016"') if n.endswith('.json') else s
    if n.startswith('verify_effective'):
        s=once(s,"assert '/workspace/cbs_gtsam4.3/build/kimera_vio/libkimera_vio.so' in loaded",
          "assert str(root/'private_build/libkimera_vio.so') in loaded\nassert '/workspace/cbs_gtsam4.3/build/kimera_vio/libkimera_vio.so' not in loaded")
        target="assert env(glim_pid)['CBS_E1B_PROFILE_PATH']"
        check="""assert env(pid)['FL4Q_ENABLE']=='1'
assert env(pid)['FL4M_RETIREMENT']=='remove'
assert Path(env(pid)['FL4Q_LOG']).resolve()==source/'acceptance.csv'
assert Path(env(pid)['FL4M_AUDIT']).resolve()==source/'objective.csv'
assert 'FL4Q_ENABLE' not in env(glim_pid)
assert env(glim_pid)['CBS_E1B_PROFILE_PATH']"""
        s=once(s,target,check)
    if n=='kimera_vio_ros.launch':
        target='    <param name="log_gt_data"'
        envs='''    <env name="FL4Q_ENABLE" value="1"/>
    <env name="FL4Q_LOG" value="$(env ROS_LOG_DIR)/../acceptance.csv"/>
    <env name="FL4M_RETIREMENT" value="remove"/>
    <env name="FL4M_AUDIT" value="$(env ROS_LOG_DIR)/../objective.csv"/>
'''
        s=once(s,target,envs+target)
    if n in ['run_case.py','run_case_on.py']:
        s=once(s,"'SHARED_NATIVE_PROVIDER_FAILURE']", "'SHARED_NATIVE_PROVIDER_FAILURE','FL4Q','active_window_rollback','active_window_retry_local_update_','Smoother recovery','Persistent BPSAM recovery']")
    (R/n).write_text(s)
    shutil.copymode(O/n,R/n)
shutil.copytree(O/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
# Preserve the exact executable/path guard, registering only this fresh copy.
guard=R/'transport_tools/live_receiver_protocol.py'
guard.write_text(once(guard.read_text(),str(O/'receiver_fl4e.sh'),str(R/'receiver_fl4e.sh')))
spec=importlib.util.spec_from_file_location('fl4s_protocol_preflight',guard)
protocol=importlib.util.module_from_spec(spec)
exec(compile(guard.read_text(),str(guard),'exec'),protocol.__dict__)
for mode in ('off','on'):
    protocol.validate_config(json.loads((R/f'pilot30_{mode}.json').read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
shutil.copy2(H/'validate_candidate.py',R/'validate_candidate.py')
for mode in ('off','on'):
    c=json.loads((R/f'pilot30_{mode}.json').read_text())
    assert c['receiver_model']=='source_wnoa' and c['receiver_fixed_lag_seconds']==7
    assert c['receiver_publication_policy']=='fl4a_fixed_lag_incremental'
    assert c['support_period_ns']==100000000 and c['startup_samples']==59
    assert all(s['session']==f'eth1_fl4s_live30_{mode}_20261016' for s in c['sources'])
off=json.loads((R/'pilot30_off.json').read_text());on=json.loads((R/'pilot30_on.json').read_text())
for c in [off,on]:
    c.pop('sequence')
    for s in c['sources']:s.pop('session')
assert off==on
for p in R.rglob('*'):
    if p.suffix=='.py':ast.parse(p.read_text())
    if p.suffix=='.launch':ET.parse(p)
    if p.suffix in ['.sh','.env']:subprocess.run(['bash','-n',str(p)],check=True)
diff=[]
for n in files:
    diff.extend(difflib.unified_diff((O/n).read_text().splitlines(True),(R/n).read_text().splitlines(True),fromfile=n+' (FL4f)',tofile=n+' (FL4s)'))
diff.extend(difflib.unified_diff((O/'transport_tools/live_receiver_protocol.py').read_text().splitlines(True),guard.read_text().splitlines(True),fromfile='protocol (FL4f)',tofile='protocol (FL4s2)'))
(H/'harness_v2.diff').write_text(''.join(diff))
manifest={**old,**protected,**q['files'],**failed_preflight}
manifest.update({str(p):sha(p) for p in (F/'pilot30_off_control').rglob('*') if p.is_file()})
manifest.update({str(p):sha(p) for p in R.rglob('*') if p.is_file()})
with (R/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
(R/'preflight').mkdir()
evidence=dict(passed=True,protected=len(manifest),kimera_sha=q['library_sha'],
    receiver_sha=sha('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live'),
    unchanged_glim_libraries=True,unchanged_profiles=True,matched_receiver_configs=True,
    transport_config_validated=True,preplayback_failure_preserved=str(F),
    native_mean_alignment_enabled=True,source_cases='OFF then classic bidirectional temporary ON')
with (R/'preflight/validation.json').open('x') as f:json.dump(evidence,f,indent=2)
print(json.dumps(evidence,indent=2))
