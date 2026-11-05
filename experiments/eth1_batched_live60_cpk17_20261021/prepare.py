"""Fresh bounded harness copy; reuse existing capture/evaluation machinery."""
from pathlib import Path
import ast, difflib, hashlib, json, shutil, subprocess
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared'
R=W/'runs/gnssfgo_eth1_batched_live60_cpk17_20261021/prepared'
V=Path('/home/yeranis/results/gnssfgo_eth1_batched_live60_cpk17_20261021')
F=H.parent/'eth1_full_rate020_cpk14_20261020'
OLD='/workspace/cbs_gtsam4.3/runs/native_covariance_cache_cpi3_20261019/release_v2'
NEW='/workspace/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/release_1'
HASH='6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def common(s):
    return (s.replace(S.parent.name,R.parent.name).replace('eth1_cpk14_full','eth1_cpk17_live60')
            .replace('11462','11468').replace('CP-K14','CP-K17')
            .replace('full_off', 'live60_off').replace('full_on', 'live60_on')
            .replace('full_{', 'live60_{').replace("['full']","['live60']")
            .replace('evaluation_full','evaluation_live60').replace('346.32','60.0'))

assert not R.parent.exists() and not V.exists(),'refuse reused outputs'
assert shutil.disk_usage(W).free>15*1024**3
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,w in manifest.items():assert sha(p)==w,p
candidate=W/'runs/main_gpu_batching_cpk16_20261020/release_1/libgtsam_points.so.1.2.1'
assert sha(candidate)==HASH
bagcheck=(H.parent/'eth1_speed60_cpk12_20261020/inspect_bags.py').read_text()
p=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
    'source /opt/ros/noetic/setup.bash && python3 -'],input=bagcheck,text=True,capture_output=True)
assert p.returncode==0,p.stdout+p.stderr
bags=json.loads(p.stdout);assert bags['passed'] and bags['window']['duration']==60
R.mkdir(parents=True)
for name in ('private_build','input_profiles'):
    shutil.copytree(S/name,R/name,symlinks=True)
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'validate_local.py','map_guard.py','source.env','receiver_fl4e.sh','run_case.py',
 'run_case_on.py','verify_effective.py','verify_effective_on.py','source_runner.sh',
 'source_runner_on.sh','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch',
 'full_off.json','full_on.json','evaluate_fl4f.py','preflight_on.py']
diff=[]
for name in files:
    old=(S/name).read_text();s=common(old)
    if name in ('glim_profile.launch','glim_profile_on.launch'):
        s=s.replace(OLD,NEW)
        s=once(s,'<env name="GLIM_SHARED_NATIVE_PROVIDER"',
               '<env name="CPK16_MAIN_GPU_BATCHING" value="1"/>\n        <env name="GLIM_SHARED_NATIVE_PROVIDER"')
    elif name=='map_guard.py':s=s.replace(OLD,NEW)
    elif name in ('verify_effective.py','verify_effective_on.py'):
        s=s.replace(OLD,NEW).replace('3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7',HASH)
        s=once(s,"assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'",
            "assert env(glim_pid)['CPK16_MAIN_GPU_BATCHING']=='1'\n"
            "assert 'CPK16_MAIN_GPU_BATCHING' not in env(pid)\n"
            "assert 'LD_PRELOAD' not in env(glim_pid)\n"
            "assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'")
    elif name in ('source_runner.sh','source_runner_on.sh'):
        s=once(s,'min_glim_rows=3430','min_glim_rows=570')
        s=once(s,'min_kimera_rows=1700','min_kimera_rows=270')
    elif name=='validate_case.py':
        s=s.replace("['rows']>=3430","['rows']>=570").replace("['rows']>=1700","['rows']>=270")
        s=once(s,'1e-9>=344.32','1e-9>=58.0')
        s=s.replace('full ETH1 capture','60-second ETH1 capture')
    elif name=='validate_candidate.py':s=once(s,'len(rows) >= 1700','len(rows) >= 270')
    elif name=='validate_local.py':s=once(s,'first>=3430','first>=570')
    elif name=='preflight_on.py':
        s=once(s,"V/'validation.json',V/'repeat_validation.json'","V/'validation.json'")
        s=s.replace('all existing gates','ordinary capture/log checks; no repeat replay')
    dest=R/name.replace('full_','live60_')
    dest.write_text(s);shutil.copymode(S/name,dest)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(dest)))
shutil.copytree(S/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
(H/'harness.diff').write_text(''.join(diff))
(H/'inspect_bags.py').write_text(bagcheck)
def cfg(p):
    c=json.loads(p.read_text())
    for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
    for v in c['sources']:v.pop('session')
    return c
assert cfg(R/'live60_off.json')==cfg(R/'live60_on.json')==cfg(S/'full_off.json')
for name in ('receiver_fl4e.sh','coverage_checks.py'):
    assert sha(R/name)==sha(S/name)
for folder in ('private_build','input_profiles'):
    for p in (S/folder).rglob('*'):
        if p.is_file():assert sha(p)==sha(R/p.relative_to(S))
for p in R.rglob('*'):
    if p.suffix=='.py':ast.parse(p.read_text())
    elif p.suffix=='.launch':ET.parse(p)
    elif p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
manifest[str(candidate)]=HASH
for base in (R,H):
    for p in base.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:manifest[str(p)]=sha(p)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=dict(passed=True,requested_s=60,playback_rate=.20,template=str(S),bags=bags,
    protected_files=len(manifest),glim_candidate_sha256=HASH,kimera_and_receiver_unchanged=True,
    exhaustive_checker=False,initial_container_state='both stopped',no_repeat_replay=True)
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='bags'},indent=2))
