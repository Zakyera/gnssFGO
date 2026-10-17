"""Mechanical duration extension of validated FL4s; no estimator/model edits."""
from pathlib import Path
import ast, difflib, hashlib, json, shutil, subprocess, types
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
O=W/'runs/gnssfgo_fixed_lag_live30_off_on_fl4s2_20261016/prepared'
R=W/'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
old=json.loads((O/'protected_manifest.json').read_text())
for p,h in old.items():assert sha(p)==h,p
close=json.loads((H.parent/'fixed_lag_live_fl4s_20261016/closeout.json').read_text())
assert close['passed']
for p,h in close['validation_hashes'].items():assert sha(p)==h,p
assert sha(H.parent/'fixed_lag_live_fl4s_20261016/REPORT.txt')==close['report_sha256']
assert sha(O/'private_build/libkimera_vio.so')=='6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
exe=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')
assert sha(exe)=='53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'
assert not R.parent.exists()
assert shutil.disk_usage(W).free>8*1024**3
R.mkdir(parents=True)
shutil.copytree(O/'private_build',R/'private_build',symlinks=True)
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'evaluate_fl4f.py','compare_belief_cov_fl4f.py','preflight_on_fl4f.py','map_guard.py',
 'source.env','receiver_fl4e.sh','run_case.py','run_case_on.py','verify_effective.py',
 'verify_effective_on.py','source_runner.sh','source_runner_on.sh','active_window.launch',
 'active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','pilot30_off.json','pilot30_on.json']
def common(s):
    return (s.replace(O.parent.name,R.parent.name).replace('11441','11442')
        .replace('gnssfgo_fixed_lag_live_fl4s_20261016','gnssfgo_fixed_lag_live_fl4t_20261016')
        .replace('eth1_fl4s_live30_','eth1_fl4t_live120_').replace('pilot30','live120'))
diff=[]
for n in files:
    s=common((O/n).read_text())
    if n=='source.env':s=once(s,':-30.0',':-120.0')
    if n.startswith('source_runner'):
        s=once(s,'"${duration_sec}" == 30.0','"${duration_sec}" == 120.0')
        s=s.replace('Prepared 30-second','Prepared 120-second')
        s=once(s,'min_glim_rows=270','min_glim_rows=1170')
        s=once(s,'min_kimera_rows=120','min_kimera_rows=570')
    if n.startswith('verify_effective'):
        s=once(s,"['CBSMS_BAG_DURATION_SEC'] == '30.0'","['CBSMS_BAG_DURATION_SEC'] == '120.0'")
    if n in ['run_case.py','run_case_on.py']:
        s=once(s,'import shlex','import shlex\nimport shutil')
        s=once(s,'    duration=30','    duration=120')
        s=once(s,'    own.mkdir(exist_ok=False)',"    assert shutil.disk_usage(ROOT).free>8*1024**3, 'disk preflight: need 8GiB free'\n    own.mkdir(exist_ok=False)")
        s=once(s,"        while jobs['source'].poll() is None:","        while jobs['source'].poll() is None:\n            assert shutil.disk_usage(ROOT).free>5*1024**3, 'disk reserve below 5GiB'")
    if n=='validate_candidate.py':
        s=once(s,'assert len(a)==len(obj)==len(poses)==134','assert len(a)==len(obj)==len(poses) and len(a)>=570')
        s=once(s,'np.arange(134)','np.arange(len(a))')
    if n=='validate_case.py':
        s=once(s,"source_cov['requested_s']==30","source_cov['requested_s']==120")
        s=once(s,"['rows']>=270 and source_cov['native_kimera']['rows']>=120","['rows']>=1170 and source_cov['native_kimera']['rows']>=570")
        s=once(s,'*1e-9>=28.','*1e-9>=118.')
        s=once(s,'timeout=300','timeout=1200')
        s=once(s,"assert action['unsupported_at_end']==summary['unsupported_end']","assert action['unsupported_at_end']==summary['unsupported_end']==0")
    if n=='preflight_on_fl4f.py':
        s=once(s,"assert json.loads((RESULT/'validation.json').read_text())['passed']","assert json.loads((P/'live120_off_source/candidate_validation.json').read_text())['passed']\nassert json.loads((RESULT/'validation.json').read_text())['passed']")
    if n=='compare_belief_cov_fl4f.py':s=once(s,'assert len(off)==421','assert len(off)>=1700')
    if n=='evaluate_fl4f.py':s=once(s,'assert len(times)>200','assert len(times)>1100')
    if n.endswith('.json'):s=s.replace('first30s','first120s').replace('FL4f fixed-lag','FL4t fixed-lag')
    target=R/n.replace('pilot30','live120')
    target.write_text(s);shutil.copymode(O/n,target)
    diff.extend(difflib.unified_diff((O/n).read_text().splitlines(True),s.splitlines(True),fromfile=n+' FL4s',tofile=target.name+' FL4t'))
shutil.copytree(O/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):
    s=p.read_text();new=common(s)
    p.write_text(new)
    if s!=new:diff.extend(difflib.unified_diff(s.splitlines(True),new.splitlines(True),fromfile=p.name+' FL4s',tofile=p.name+' FL4t'))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
protocol=types.ModuleType('preflight_protocol')
guard=R/'transport_tools/live_receiver_protocol.py'
exec(compile(guard.read_text(),str(guard),'exec'),protocol.__dict__)
for mode in ('off','on'):
    c=json.loads((R/f'live120_{mode}.json').read_text());protocol.validate_config(c)
    assert all(s['session']==f'eth1_fl4t_live120_{mode}_20261016' for s in c['sources'])
    oldcfg=json.loads((O/f'pilot30_{mode}.json').read_text())
    for cfg in (c,oldcfg):
        for key in ('sequence','ros_master_uri','receiver_binary'):cfg.pop(key)
        for source in cfg['sources']:source.pop('session')
    assert c==oldcfg
assert sha(R/'private_build/libkimera_vio.so')==sha(O/'private_build/libkimera_vio.so')
assert sha(R/'receiver_fl4e.sh')==sha(O/'receiver_fl4e.sh')
for p in R.rglob('*'):
    if p.suffix=='.py':ast.parse(p.read_text())
    if p.suffix=='.launch':ET.parse(p)
    if p.suffix in ['.sh','.env']:subprocess.run(['bash','-n',str(p)],check=True)
(H/'harness.diff').write_text(''.join(diff))
manifest={**old,**close['validation_hashes']}
# Preserve preceding gate's preparation/evaluation and all primary outputs.
for p in O.iterdir():
    if p.is_file() and p.name!='controller.lock':manifest[str(p)]=sha(p)
for folder in [O/'evaluation_full',H.parent/'fixed_lag_live_fl4s_20261016']:
    for p in folder.rglob('*'):
        if p.is_file():manifest[str(p)]=sha(p)
for mode in ('off','on'):
    for n in ['odom_imu.txt','kimera_odometry.csv','acceptance.csv','objective.csv']:
        p=O/f'pilot30_{mode}_source'/n;manifest[str(p)]=sha(p)
    for n in ['online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum']:
        p=O/f'pilot30_{mode}_receiver/receiver'/n;manifest[str(p)]=sha(p)
manifest.update({str(p):sha(p) for p in R.rglob('*') if p.is_file()})
with (R/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
(R/'preflight').mkdir()
result=dict(passed=True,protected=len(manifest),duration=120,rate=.1,
    kimera_sha=sha(R/'private_build/libkimera_vio.so'),receiver_sha=sha(exe),
    same_sensor_profiles=True,same_receiver_model_config=True,
    protocol_configs_validated=True,free_bytes=shutil.disk_usage(W).free)
with (R/'preflight/validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
