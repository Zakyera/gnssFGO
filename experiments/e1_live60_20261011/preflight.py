#!/usr/bin/env python3
"""Freeze exact private inputs and check launch equivalence before capture."""
import ast
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import yaml

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared'
ROOT=WS/'runs/gnssfgo_e1_live60_20261011/prepared'
PRIOR=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
OUT=ROOT/'preflight'
OUT.mkdir(exist_ok=False)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def inside(p):return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
def docker(cmd,name):
    r=subprocess.run(['docker','exec','cbsms_ws_gpu','bash','-lc',cmd],capture_output=True,text=True)
    with (OUT/name).open('x') as f:f.write(r.stdout+r.stderr)
    assert r.returncode==0,(name,r.stderr[-2000:])
    return r.stdout
prior=json.loads((HERE.parent/'e1_sync_compatible_batching_20261011/validation_manifest.json').read_text())
old=json.loads((BASE/'protected_manifest.json').read_text())
changed={p:{'baseline':v,'candidate':sha(p)} for p,v in old.items() if sha(p)!=v}
assert changed==prior['intentional_source_changes']
for p,h in prior['prior_candidate_and_baseline_binary_checks'].items():assert sha(p)==h,p
assert json.loads((WS/'runs/glim_e1_real_graph_20261011/prepared/validation.json').read_text())['passed']
for name,p in [('libglim_ros.so',PRIOR/'libglim_ros.so'),
               ('libgtsam_points_cuda.so.1.2.1',PRIOR/'libgtsam_points_cuda.so.1.2.1'),
               ('libcbs.so',WS/'runs/glim_local_gpu_batching_e1_20261011/private/libcbs.so')]:
    assert sha(ROOT/'private_build'/name)==sha(p)
assert sha(ROOT/'private_build/libglim_ros.so')==prior['binaries'][str(PRIOR/'libglim_ros.so')]
assert shutil.disk_usage(ROOT).free>15*1024**3
for p in [*ROOT.glob('*.py'),*ROOT.parent.glob('*.py'),*HERE.glob('*.py')]:ast.parse(p.read_text())
for p in [ROOT/'source_runner.sh',ROOT/'source.env']:subprocess.run(['bash','-n',str(p)],check=True)
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
docker('nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv','gpu.log')
docker("python3 -c 'import socket; s=socket.socket(); assert s.connect_ex((\"127.0.0.1\",11403))!=0'",'port.log')
probe="""from pathlib import Path
hits=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit(): continue
 try:
  a=(p/'cmdline').read_bytes().split(b'\\0')
  if not a or a[0].endswith(b'python3'):continue
  if any(x in a[0] for x in [b'glim_rosnode',b'kimera_vio_ros_node',b'/replay']):hits.append((p.name,a[0].decode()))
 except OSError:pass
print(hits)
assert not hits
"""
docker('python3 -c '+shlex.quote(probe),'no_sources.log')
r=subprocess.run(['pgrep','-af','^(/home/yeranis/results/.*/replay|python3 .*/launch_grandtour_live_receiver.py)'],capture_output=True,text=True)
assert r.returncode==1,r.stdout
libenv='export LD_LIBRARY_PATH='+inside(ROOT/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib:$LD_LIBRARY_PATH && '
ldd=docker(setup+libenv+'ldd -r '+inside(ROOT/'private_build/libglim_ros.so'),'ldd.log')
assert 'undefined symbol' not in ldd and 'not found' not in ldd
for name in ['libcbs.so','libgtsam_points_cuda.so.1']:assert inside(ROOT/'private_build'/name) in ldd
docker(setup+libenv+inside(PRIOR/'test_attempt3')+' --gtest_output=xml:'+inside(OUT/'regression.xml'),'regression.log')
assert '[  PASSED  ] 14 tests.' in (OUT/'regression.log').read_text()
sys.path.insert(0,str(ROOT))
from experiment import launch_arguments,BINARY
args=launch_arguments()
params=docker(setup+shlex.join(['roslaunch','--dump-params',inside(ROOT/'active_window.launch'),*args]),'resolved_params.yaml')
kimera=docker(setup+shlex.join(['roslaunch','--args=/kimera_vio_ros/kimera_vio_ros_node',inside(ROOT/'active_window.launch'),*args]),'kimera_args.txt')
a=yaml.safe_load((BASE/'preflight/resolved_params.yaml').read_text());b=yaml.safe_load(params)
differences={k:[a.get(k),b.get(k)] for k in set(a)|set(b) if a.get(k)!=b.get(k)}
for k,(x,y) in differences.items():
    if k=='/glim_rosnode/glim_cbs_experimental_local_gpu_batching':assert x is None and y is True
    else:assert isinstance(x,str) and x.replace(inside(BASE),inside(ROOT))==y,(k,x,y)
assert (BASE/'preflight/kimera_args.txt').read_text().replace(inside(BASE),inside(ROOT))==kimera
cfg=json.loads((ROOT/'live60_on.json').read_text());base_cfg=json.loads((BASE/'full160_on.json').read_text())
assert {k:v for k,v in cfg.items() if k not in ['sequence','ros_master_uri','sources']}=={k:v for k,v in base_cfg.items() if k not in ['sequence','ros_master_uri','sources']}
for x,y in zip(cfg['sources'],base_cfg['sources']):assert {k:v for k,v in x.items() if k!='session'}=={k:v for k,v in y.items() if k!='session'}
assert sha(BINARY)=='62124b496ad6f854079218862acb149892aa452d4af908c50bc96067e49c9dbb'
sys.path.insert(0,'/home/yeranis/repos/gnssFGO/tools')
from live_receiver_protocol import validate_config
validate_config(cfg)
manifest={p:sha(p) for p in old}
for p in [*ROOT.glob('*'),*ROOT.parent.glob('*.py'),ROOT.parent/'PROTOCOL.txt',*HERE.glob('*'),
          *ROOT.joinpath('private_build').glob('*')]:
    if p.is_file():manifest[str(p)]=sha(p)
for p in PRIOR.glob('lib*'):
    if p.is_file():manifest[str(p)]=sha(p)
with (ROOT/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
result=dict(passed=True,duration_s=60,rate=.1,port=11403,protected_entries=len(manifest),
    original_protected_unchanged=len(old)-len(changed),intentional_prior_changes=changed,
    parameter_differences=differences,receiver_sha256=sha(BINARY),
    private_binaries={p.name:sha(p) for p in (ROOT/'private_build').glob('*') if p.is_file()},
    raw_bag_provenance=json.loads((WS/'runs/glim_e1_real_graph_20261011/prepared/preflight.json').read_text())['bags_metadata_not_full_file_hashes'])
for p,v in result['raw_bag_provenance'].items():
    s=Path(p).stat();assert s.st_size==v['bytes'] and s.st_mtime_ns==v['mtime_ns']
with (OUT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print('E1_LIVE60_PREFLIGHT_PASS: exact candidate, receiver, source flags, private libraries,14 tests')
