"""Fresh standalone ON copy of frozen FL4t; no estimator or model modification."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess,types
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
R=W/'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
manifest=json.loads((T/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
assert sha(T/'private_build/libkimera_vio.so')=='6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
exe=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')
assert sha(exe)=='53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'
assert not R.parent.exists()
assert shutil.disk_usage(W).free>8*1024**3
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
def remap(s):
    return (s.replace(T.parent.name,R.parent.name).replace('11442','11444')
        .replace('gnssfgo_fixed_lag_live_fl4t_20261016','gnssfgo_fixed_lag_live_fl4v_20261017')
        .replace('eth1_fl4t_live120_on_20261016','eth1_fl4v_live120_on_20261017')
        .replace('eth1_fl4t_live120_off_20261016','eth1_fl4v_live120_off_20261017'))
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
    'evaluate_fl4f.py','map_guard.py','source.env','receiver_fl4e.sh','run_case_on.py',
    'verify_effective_on.py','source_runner_on.sh','active_window.launch','active_window_on.launch',
    'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
    'kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch',
    'live120_off.json','live120_on.json']
diff=[]
for n in files:
    old=(T/n).read_text();s=remap(old)
    if n=='evaluate_fl4f.py':
        s=s.replace("('off','on')","('on',)")
        s=s.replace('Run after both live cases validate','Run after the standalone ON case validates')
    if n=='live120_on.json':s=s.replace('FL4t fixed-lag WNOA matched live pilot','FL4v fixed-lag WNOA standalone ON')
    (R/n).write_text(s);shutil.copymode(T/n,R/n)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=n+' FL4t',tofile=n+' FL4v'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):
    old=p.read_text();s=remap(old);p.write_text(s)
    if s!=old:diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=p.name+' FL4t',tofile=p.name+' FL4v'))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
protocol=types.ModuleType('preflight_protocol');guard=R/'transport_tools/live_receiver_protocol.py'
exec(compile(guard.read_text(),str(guard),'exec'),protocol.__dict__)
c=json.loads((R/'live120_on.json').read_text());protocol.validate_config(c)
old=json.loads((T/'live120_on.json').read_text())
for cfg in (c,old):
    for k in ('sequence','ros_master_uri','receiver_binary'):cfg.pop(k)
    for s in cfg['sources']:s.pop('session')
assert c==old
assert sha(R/'receiver_fl4e.sh')==sha(T/'receiver_fl4e.sh')
for p in (R/'private_build').rglob('*'):
    if p.is_file():assert sha(p)==sha(T/'private_build'/p.relative_to(R/'private_build'))
for p in R.rglob('*'):
    if p.suffix=='.py':ast.parse(p.read_text())
    if p.suffix=='.launch':ET.parse(p)
    if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for p in T.iterdir():
    if p.is_file() and p.name!='controller.lock':manifest[str(p)]=sha(p)
for folder in [T/'live120_off_source',H.parent/'fixed_lag_live_fl4t_20261016',H.parent/'fixed_lag_stopping_fl4u_20261017']:
    for p in folder.iterdir():
        if p.is_file():manifest[str(p)]=sha(p)
manifest.update({str(p):sha(p) for p in R.rglob('*') if p.is_file()})
manifest[str(H/'PROTOCOL.txt')]=sha(H/'PROTOCOL.txt')
manifest[str(H/'prepare.py')]=sha(H/'prepare.py')
with (R/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
(R/'preflight').mkdir()
result=dict(passed=True,protected=len(manifest),duration=120,rate=.1,standalone_on=True,
    kimera_sha=sha(R/'private_build/libkimera_vio.so'),receiver_sha=sha(exe),
    same_receiver_model_config=True,protocol_config_valid=True,free_bytes=shutil.disk_usage(W).free)
with (R/'preflight/validation.json').open('x') as f:json.dump(result,f,indent=2)
with (R/'preflight_on.json').open('x') as f:
    json.dump(dict(passed=True,admission='explicit user-authorized standalone ON despite failed OFF; not a matched120s comparison',
        protected_unchanged=len(manifest),receiver_config='same FL4t model; fresh identities and port',
        source_settings='same frozen FL4t ON launch and binaries',off_gate_not_passed=True),f,indent=2)
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
print(json.dumps(result,indent=2))
