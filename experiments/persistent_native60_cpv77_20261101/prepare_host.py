"""Verify originals and prepare byte-exact wire bag; no estimator launch."""
import ast,datetime,hashlib,json,shutil,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name;R=K/'replay'
S=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared/full_on_source'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
assert not (H/'prepared.json').exists() and shutil.disk_usage(W).free>8*1024**3
pins=json.loads((E/'persistent_compatibility_cpv76_20261101/protected_before.json').read_text())
pins.update(json.loads((E/'retention_readiness_cpv73_20261031/complete.json').read_text())['output_sha256'])
for p,v in pins.items():assert sha(p)==v,p
v=json.loads((E/'persistent_compatibility_cpv76_20261101/verification_fixture1.json').read_text());assert v['passed']
for name,digest in v['libraries'].items():assert sha(K/'private_build'/name)==digest
for p in (R/'kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(S/'kimera_profile'/p.relative_to(R/'kimera_profile'))
for root in (R,H):
    for p in root.glob('*.py'):ast.parse(p.read_text())
for p in R.glob('*.launch'):ET.parse(p)
audit=json.loads((S.parent/'input_audit.json').read_text())
for b in audit['bag_files'].values():
    assert sha(b['path'])==b['sha256'];pins[b['path']]=b['sha256']
for p in (S/'estimators.log',S/'recordings/cbs.bag',S/'kimera_odometry.csv',S/'effective_mode_check.json'):
    pins[str(p)]=sha(p)
before=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert before=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',before
save(H/'preparation_start.json',dict(time=datetime.datetime.now().astimezone().isoformat(),containers=before))
try:
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
    cmd=['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash; source /workspace/cbs_gtsam4.3/devel/setup.bash; PYTHONDONTWRITEBYTECODE=1 python3 /workspace/cbs_gtsam4.3/runs/'+H.name+'/replay/prepare_inputs.py']
    with (H/'prepare_inputs.log').open('x') as f:
        p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT);save(H/'prepare_inputs_process.json',dict(pid=p.pid,args=cmd));rc=p.wait()
    assert rc==0,('input preparation',rc)
finally:
    top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
    assert all(l.split()[1] in ('tail','zsh') for l in top.splitlines()[1:] if l.strip()),top
    subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
    after=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    save(H/'preparation_cleanup.json',dict(containers=after,restored=after==before));assert after==before
raw=json.loads((R/'raw_inputs.json').read_text());assert raw['selected_beliefs']>500
for root in (K,H):
    for p in root.rglob('*'):
        if p.is_file() and p.name not in ('REPORT.txt','prepare_host.log'):
            pins[str(p)]=sha(p)
save(H/'protected_before.json',pins)
save(H/'prepared.json',dict(passed=True,protected_files=len(pins),binaries=v['libraries'],
    input_messages=raw['selected_messages'],input_beliefs=raw['selected_beliefs'],free_bytes=shutil.disk_usage(W).free,
    scope='Kimera-only native-matching temporary/general persistent 60s'))
print('PREPARED',len(pins),'pins, complete raw beliefs',raw['selected_beliefs'],flush=True)
