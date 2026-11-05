"""Mechanical full-window extension of the completed K17 harness; no estimator edits."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,socket,subprocess
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eth1_batched_live60_cpk17_20261021/prepared'
R=W/'runs/gnssfgo_eth1_batched_full_cpk18_20261021/prepared'
V=Path('/home/yeranis/results/gnssfgo_eth1_batched_full_cpk18_20261021')
H17=H.parent/'eth1_batched_live60_cpk17_20261021'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def common(s):
    return (s.replace(S.parent.name,R.parent.name)
        .replace('eth1_cpk17_live60','eth1_cpk18_full')
        .replace('11468','11469').replace('CP-K17','CP-K18')
        .replace('live60_off','full_off').replace('live60_on','full_on')
        .replace('live60_{','full_{').replace("['live60']","['full']")
        .replace('evaluation_live60','evaluation_full'))

assert not R.parent.exists() and not V.exists(),'refuse existing experiment outputs'
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as sock:sock.bind(('127.0.0.1',11469))
assert json.loads((H17/'results.json').read_text())['passed']
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,w in manifest.items():assert sha(p)==w,p
for mode in ('off','on'):
    assert (S/f'live60_{mode}_control/complete.json').exists()
    for p in (S/f'live60_{mode}_source/local_provenance.json',
              S/f'live60_{mode}_source/candidate_validation.json',
              Path('/home/yeranis/results')/S.parent.name/mode/'validation.json'):
        assert json.loads(p.read_text())['passed'],p
bagcheck=(H.parent/'eth1_full_rate020_cpk14_20261020/inspect_bags.py').read_text()
p=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
    'source /opt/ros/noetic/setup.bash && python3 -'],input=bagcheck,text=True,capture_output=True)
assert p.returncode==0,p.stdout+p.stderr
bags=json.loads(p.stdout);assert bags['passed'] and bags['window']['duration']==346.32
R.mkdir(parents=True)
for name in ('private_build','input_profiles'):
    shutil.copytree(S/name,R/name,symlinks=True)
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'validate_local.py','map_guard.py','source.env','receiver_fl4e.sh','run_case.py',
 'run_case_on.py','verify_effective.py','verify_effective_on.py','source_runner.sh',
 'source_runner_on.sh','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch',
 'live60_off.json','live60_on.json','evaluate_fl4f.py','preflight_on.py']
diff=[]
for name in files:
    old=(S/name).read_text();s=common(old)
    if name in ('run_case.py','run_case_on.py'):
        s=once(s,'duration=60.0','duration=346.32')
    elif name in ('source_runner.sh','source_runner_on.sh'):
        s=once(s,'"${duration_sec}" == 60.0','"${duration_sec}" == 346.32')
        s=once(s,'full60.0-second','full346.32-second')
        s=once(s,'min_glim_rows=570','min_glim_rows=3430')
        s=once(s,'min_kimera_rows=270','min_kimera_rows=1700')
    elif name in ('verify_effective.py','verify_effective_on.py'):
        s=once(s,"CBSMS_BAG_DURATION_SEC'] == '60.0'","CBSMS_BAG_DURATION_SEC'] == '346.32'")
        s=once(s,'requested_s=60.0','requested_s=346.32')
    elif name=='source.env':s=once(s,'CBSMS_BAG_DURATION_SEC:-60.0','CBSMS_BAG_DURATION_SEC:-346.32')
    elif name=='validate_case.py':
        s=once(s,"['requested_s']==60.0","['requested_s']==346.32")
        s=once(s,"['rows']>=570","['rows']>=3430")
        s=once(s,"['rows']>=270","['rows']>=1700")
        s=once(s,'1e-9>=58.0','1e-9>=344.32')
        s=once(s,'60-second ETH1 capture','full346.32-second ETH1 capture')
    elif name=='validate_candidate.py':s=once(s,'len(rows) >= 270','len(rows) >= 1700')
    elif name=='validate_local.py':s=once(s,'first>=570','first>=3430')
    elif name=='preflight_on.py':s=once(s,'same60.0s OFF','same346.32s OFF')
    elif name in ('live60_off.json','live60_on.json'):
        s=once(s,'bag0-60.0s','bag0-346.32s')
        s=s.replace('_20261019"','_20261021"')
    dest=R/({'live60_off.json':'full_off.json','live60_on.json':'full_on.json'}.get(name,name))
    dest.write_text(s);shutil.copymode(S/name,dest)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(dest)))
shutil.copytree(S/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
(H/'harness.diff').write_text(''.join(diff))
(H/'inspect_bags.py').write_text(bagcheck)
summary=common((H17/'summarize.py').read_text())
summary=once(summary,"('last20s',40)","('last20s',326.32)")
summary=summary.replace('One60s experimental pair, not a full batching-preservation or full-sequence certificate',
    'One full-sequence experimental pair, not an exhaustive batching-preservation certificate')
summary=summary.replace('batched live60s','batched full346.32s')
summary=summary.replace('no default promotion/full-sequence claim','no default promotion or exhaustive preservation claim')
summary=once(summary,"(H/'REPORT.txt').open('x')","(H/'RESULTS_REPORT.txt').open('x')")
(H/'summarize.py').write_text(summary)
def cfg(p):
    c=json.loads(p.read_text())
    for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
    for v in c['sources']:v.pop('session')
    return c
assert cfg(R/'full_off.json')==cfg(R/'full_on.json')==cfg(S/'live60_off.json')
for name in ('receiver_fl4e.sh','coverage_checks.py'):
    assert sha(R/name)==sha(S/name)
for folder in ('private_build','input_profiles'):
    for p in (S/folder).rglob('*'):
        if p.is_file():assert sha(p)==sha(R/p.relative_to(S)),p
for p in [*R.rglob('*'),H/'summarize.py']:
    if p.suffix=='.py':ast.parse(p.read_text())
    elif p.suffix=='.launch':ET.parse(p)
    elif p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for base in (R,H):
    for p in base.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.name!='REPORT.txt':manifest[str(p)]=sha(p)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=dict(passed=True,requested_s=346.32,playback_rate=.20,template=str(S),bags=bags,
    protected_files=len(manifest),binaries_and_model_and_profiles_unchanged=True,
    exhaustive_checker=False,initial_container_state='both stopped',no_repeat_replay=True)
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='bags'},indent=2))
