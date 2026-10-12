"""Verify frozen inputs/libraries and source checkouts; never modifies them."""
import hashlib
import json
from pathlib import Path
import subprocess

WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_local_update_batching_20261012'
def host(path): return Path(path.replace('/workspace/cbs_gtsam4.3',str(WS)))
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for part in iter(lambda:f.read(1048576),b''):h.update(part)
    return h.hexdigest()
protected=json.loads((WS/'runs/gnssfgo_e1b_profile40_mapguard_20261012/prepared/protected_manifest.json').read_text())
baseline_count=len(protected)
protected.update(json.loads((ROOT/'protected_preparation.json').read_text()))
build_protected={}
for file in ROOT.glob('*/*_protected.json'):
    build_protected.update(json.loads(file.read_text()))
for path,digest in protected.items():assert sha(host(path))==digest,path
script='import hashlib,json,sys;from pathlib import Path;items=json.loads(sys.argv[1]);bad=[p for p,h in items.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h];assert not bad,bad;print(len(items))'
subprocess.run(['docker','exec','cbsms_ws_gpu','python3','-c',script,json.dumps(build_protected)],check=True)
repos=json.loads((ROOT/'source_git.json').read_text())
for repo,item in repos.items():
    assert subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()==item['head']
    assert not subprocess.check_output(['git','-C',repo,'status','--porcelain'],text=True).strip(),repo
maps=(ROOT/'no_data_startup/maps.txt').read_text()
actual={str(host(line.split(maxsplit=5)[5])) for line in maps.splitlines() if len(line.split(maxsplit=5))==6 and line.split(maxsplit=5)[5].startswith('/')}
names=['libglim_ros.so','libglim.so','libcbs.so','libe1b_profile.so','libgtsam.so.4.3a1','libgtsam_points_cuda.so.1.2.1']
resolved={}
for name in names:
    target=ROOT/'release_2'/name
    while target.is_symlink():
        link=target.readlink();target=host(str(link)) if link.is_absolute() else target.parent/link
    expected=str(target.resolve(strict=True))
    assert {p for p in actual if Path(p).name==name}=={expected},name
    resolved[name]=expected
assert json.loads((ROOT/'no_data_startup/result.json').read_text())['passed']
assert json.loads((ROOT/'no_data_startup/ready.json').read_text())['source_measurements']==0
summary=dict(passed=True,baseline_hashes=baseline_count,host_protected_hashes=len(protected),build_protected_hashes=len(build_protected),source_repos=len(repos),mapped_libraries=resolved)
with (ROOT/'integrity.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
print(json.dumps(summary,indent=2))
