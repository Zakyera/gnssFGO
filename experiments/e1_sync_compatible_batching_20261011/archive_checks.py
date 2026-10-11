#!/usr/bin/env python3
"""Archive bounded E1 evidence and verify checkpoint/prior-candidate integrity."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN = WS/'runs/glim_sync_compatible_batching_e1_20261011'
PREVIOUS = HERE.parent/'e1_local_gpu_batching_20261011/validation_manifest.json'
MANIFEST = WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/protected_manifest.json'

def sha(path):
    path=Path(path)
    if path.is_symlink():
        target=path.readlink()
        if str(target).startswith('/workspace/'):
            path=Path('/home/yeranis/repos/V4RL')/str(target).removeprefix('/workspace/')
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()

protected=json.loads(MANIFEST.read_text())
previous=json.loads(PREVIOUS.read_text())
changes={}
for path,expected in protected.items():
    actual=sha(path)
    if actual!=expected: changes[path]={'baseline':expected,'candidate':actual}
# This continuation must not change any additional protected entry.
assert changes==previous['intentional_source_changes'],changes
prior_checks={}
for path,expected in previous['binaries'].items():
    actual=sha(path); assert actual==expected,(path,expected,actual)
    prior_checks[path]=actual
result={'protected_total':len(protected),'unchanged_count':len(protected)-len(changes),
        'intentional_source_changes':changes,'additional_protected_changes':[],
        'prior_candidate_and_baseline_binary_checks':prior_checks,
        'binaries':{},'repositories':{},'all_local_log_hashes':{}}
for sub, names in {'private':['libgtsam_points_cuda.so.1.2.1','libcbs.so','libglim_ros.so',
                              'test_attempt2','test_attempt3','probe_attempt2','sanitize_attempt1'],
                   'ubsan':['libgtsam_points_cuda.so.1.2.1','ubsan_attempt1']}.items():
    for name in names: result['binaries'][str(RUN/sub/name)]=sha(RUN/sub/name)
for name in ['libgtsam_points.so','libgtsam_points_cuda.so']:
    result['binaries'][str(WS/'devel/lib'/name)]=sha(WS/'devel/lib'/name)
for repo in [WS/'src/cbs',WS/'src/glim_ros1',WS/'src/gtsam_points']:
    result['repositories'][repo.name]={
        'head':subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        'status':subprocess.check_output(['git','-C',str(repo),'status','--porcelain'],text=True)}
out=HERE/'validation_logs'; out.mkdir(exist_ok=False)
for sub in ['private','ubsan']:
    (out/sub).mkdir()
    for p in sorted((RUN/sub).iterdir()):
        if p.suffix not in ['.log','.xml','.json']: continue
        result['all_local_log_hashes'][str(p)]=sha(p)
        # Dense native probe matrices stay local; their hashes/comparison and
        # library-resolution logs are retained, not thousands of matrix rows.
        if p.name.startswith('native_') and p.suffix=='.log': continue
        shutil.copy2(p,out/sub/p.name)
result['archived_log_hashes']={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}
with (HERE/'validation_manifest.json').open('x') as f: json.dump(result,f,indent=2)
print(json.dumps({k:result[k] for k in ['protected_total','unchanged_count','additional_protected_changes','repositories']},indent=2))
