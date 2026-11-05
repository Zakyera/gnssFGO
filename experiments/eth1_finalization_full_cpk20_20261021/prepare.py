"""Fresh K18-shaped capture with K19 finalization; no estimator builds/edits."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,socket,subprocess
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eth1_batched_full_cpk18_20261021/prepared'
R=W/'runs/gnssfgo_eth1_finalization_full_cpk20_20261021/prepared'
V=Path('/home/yeranis/results/gnssfgo_eth1_finalization_full_cpk20_20261021')
H18=H.parent/'eth1_batched_full_cpk18_20261021'
H19=H.parent/'receiver_finalization_cpk19_20261021'
EXE=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')
EXPECTED='4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def common(s):
    return (s.replace(S.parent.name,R.parent.name).replace('eth1_cpk18_full','eth1_cpk20_full')
        .replace('11469','11470').replace('CP-K18','CP-K20').replace('receiver_fl4e.sh','receiver_cpk19.sh'))

assert not R.parent.exists() and not V.exists(),'refuse existing outputs'
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as sock:sock.bind(('127.0.0.1',11470))
assert sha(EXE)==EXPECTED
assert json.loads((H19/'closeout.json').read_text())['passed']
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,w in manifest.items():assert sha(p)==w,p
for name,w in json.loads((H19/'closeout.json').read_text())['files'].items():
    assert sha(H19/name)==w,name
    manifest[str(H19/name)]=w
manifest[str(EXE)]=EXPECTED
bagcheck=(H18/'inspect_bags.py').read_text()
p=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
    'source /opt/ros/noetic/setup.bash && python3 -'],input=bagcheck,text=True,capture_output=True)
assert p.returncode==0,p.stdout+p.stderr
bags=json.loads(p.stdout);assert bags['passed'] and bags['window']['duration']==346.32
R.mkdir(parents=True)
for name in ('private_build','input_profiles'):
    shutil.copytree(S/name,R/name,symlinks=True)
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'validate_local.py','map_guard.py','source.env','run_case.py','run_case_on.py',
 'verify_effective.py','verify_effective_on.py','source_runner.sh','source_runner_on.sh',
 'active_window.launch','active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','full_off.json','full_on.json','evaluate_fl4f.py','preflight_on.py']
diff=[]
for name in files:
    old=(S/name).read_text();s=common(old)
    if name=='validate_case.py':
        s=once(s,'from pathlib import Path','from pathlib import Path\nfrom expiry_audit import ExpiryAudit')
        s=once(s,"EXE=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')",f'EXE=Path({str(EXE)!r})')
        s=once(s,"    assert summary['complete'] and summary['lag_seconds']==7",
            "    assert summary['complete'] and summary['lag_seconds']==7\n"
            "    assert summary['absorbed_revision_policy']=='seal_at_absorption'\n"
            f"    assert sha(EXE)=={EXPECTED!r}\n"
            "    assert cfg['receiver_binary']==str(P/'receiver_cpk19.sh')")
        s=once(s,'    action=collections.Counter()','    audit=ExpiryAudit()\n    action=collections.Counter()')
        s=once(s,"            kind=r[0]", "            audit.consume(r,int(directory.name))\n"
            "            if r[:2]==['belief','sealed_reference']:continue\n            kind=r[0]")
        s=once(s,"    assert action['duplicate']==0 and action['stale']==0",
            "    assert action['duplicate']==0 and action['stale']==0\n"
            "    assert summary['expired']==sum(action[x] for x in ('expired_revision','expired_duplicate','expired_stale'))\n"
            "    assert summary['received']==sum(summary[x] for x in ('inserted','replaced','excluded','unsupported_end','duplicate','stale','expired'))\n"
            "    expiry=audit.finish(summary)")
        s=once(s,'summary=summary,actions=dict(action),','summary=summary,actions=dict(action),expiry=expiry,')
        s=once(s,"    with (RESULT/'validation.json').open('x') as f:json.dump(result,f,indent=2)",
            "    with (RESULT/'expired_details.json').open('x') as f:json.dump(audit.details,f,indent=2)\n"
            "    with (RESULT/'validation.json').open('x') as f:json.dump(result,f,indent=2)")
        begin=s.index('\ndef repeat():');end=s.index("\nif __name__=='__main__':",begin)
        s=s[:begin]+s[end:]
        s=once(s,"    print(json.dumps(check() if phase=='check' else repeat() if phase=='repeat' else None,indent=2))",
            "    assert phase=='check', 'CP-K20 does not authorize a saved-journal solver repeat'\n"
            "    print(json.dumps(check(),indent=2))")
    dest=R/name;dest.write_text(s);shutil.copymode(S/name,dest)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(dest)))
shutil.copy2(H19/'receiver_cpk19.sh',R/'receiver_cpk19.sh')
(R/'receiver_cpk19.sh').chmod(0o755)
shutil.copy2(H/'expiry_audit.py',R/'expiry_audit.py')
shutil.copytree(S/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
(H/'harness.diff').write_text(''.join(diff))
summary=common((H18/'summarize.py').read_text())
summary=summary.replace('batched full346.32s','batched finalized-policy full346.32s')
summary=once(summary," 'Kimera variable-history/7s receiver-retention issue remains an independent longer-run risk.',",
    " 'Known post-absorption revisions are explicitly expired under CP-K19; counts and',\n"
    " 'forgone mean/covariance changes are included in checks.receiver.expiry above.',\n"
    " 'These changes are not GT errors; the late corrections were not incorporated.',")
(H/'summarize.py').write_text(summary)
def cfg(p):
    c=json.loads(p.read_text())
    for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
    for v in c['sources']:v.pop('session')
    return c
assert cfg(R/'full_off.json')==cfg(R/'full_on.json')==cfg(S/'full_off.json')
assert sha(R/'coverage_checks.py')==sha(S/'coverage_checks.py')
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
    protected_files=len(manifest),source_binaries_model_profiles_unchanged=True,
    receiver_sha256=EXPECTED,absorbed_revision_policy='seal_at_absorption',
    exhaustive_checker=False,initial_container_state='both stopped',no_repeat_replay=True)
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='bags'},indent=2))
