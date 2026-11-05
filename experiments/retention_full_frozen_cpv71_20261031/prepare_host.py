"""Read-only predecessor checks and a fresh full-test pin manifest; no replay."""
import ast, hashlib, json, shutil, subprocess, xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent; E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3'); K=W/'runs'/H.name; R=K/'replay'
H70=E/'retention_update_gate_cpv70_20261031'; K70=W/'runs'/H70.name
S=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared/full_on_source'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
assert not (H/'prepared.json').exists()
assert shutil.disk_usage(W).free>8*1024**3
pins={}
def merge(d):
    for p,v in d.items():
        assert p not in pins or pins[p]==v,('conflicting historical pin',p)
        pins[p]=v
old=json.loads((H70/'complete.json').read_text());assert old['passed'] and old['phase2_comparison_passed']
merge(json.loads((H70/'replay_attempt/protected_before.json').read_text()))
merge(old['output_sha256']);pins[str(H70/'complete.json')]=sha(H70/'complete.json')
full=json.loads((E/'eig2_hesai_refresh_full_cpv49_20261030/complete.json').read_text());assert full['passed']
merge(full['output_sha256'])
for p,v in pins.items():assert sha(p)==v,p
for p in (R/'kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(S/'kimera_profile'/p.relative_to(R/'kimera_profile'))
binary={}
for name in ('libkimera_vio.so','libcbs.so'):
    v=sha(K/'private_build'/name);assert v==sha(K70/'private_build'/name)
    binary[name]=v
assert binary['libkimera_vio.so']=='6e620d0f24aed055ee8f9f2f638af1b59881ff09e23ebe561614162a93d634ce'
for mode in ('release','san','backend'):
    assert json.loads((K70/(mode+'_initial/passed.json')).read_text())['passed']
save(K/'binary_reuse.json',dict(passed=True,reused_from=str(K70),sha256=binary,no_build=True))
for root in (H,R):
    for p in root.glob('*.py'):ast.parse(p.read_text())
for p in R.glob('*.launch'):ET.parse(p)
audit=json.loads((S.parent/'input_audit.json').read_text())
for v in audit['bag_files'].values():
    assert sha(v['path'])==v['sha256'];pins[v['path']]=v['sha256']
for p in (S/'estimators.log',S/'recordings/cbs.bag',S/'kimera_odometry.csv',S/'effective_mode_check.json'):
    pins[str(p)]=sha(p)
for root in (K,H):
    for p in root.rglob('*'):
        if p.is_file() and p.name not in ('REPORT.txt','prepare_host.log'):
            pins[str(p)]=sha(p)
save(H/'protected_before.json',pins)
save(H/'prepared.json',dict(passed=True,protected_files=len(pins),binaries=binary,
    storage_free_bytes=shutil.disk_usage(W).free,forecast_additional_reserve_bytes=2*1024**3,
    scope='full355.02s conditional remove/retain; no new live feedback',source=str(S)))
print('PREPARED',len(pins),'protected files; exact CP70 binaries reused',flush=True)
