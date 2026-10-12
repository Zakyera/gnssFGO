"""Verify the retained E1 harness and E2 binary before a single live capture."""
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
BASE=WS/'runs/gnssfgo_e1_e2_live160_20261011/prepared'
ROOT=WS/'runs/gnssfgo_e1b_profile40_20261012/prepared'
LIB=WS/'runs/e1b_local_profile_20261012/release_2'
AUDIT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
OUT=ROOT/'preflight'
OUT.mkdir(exist_ok=False)
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def inside(p):return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
def docker(cmd,name,container='cbsms_ws_gpu'):
    r=subprocess.run(['docker','exec',container,'bash','-lc',cmd],capture_output=True,text=True)
    with (OUT/name).open('x') as f:f.write(r.stdout+r.stderr)
    assert r.returncode==0,(name,r.stderr[-2000:])
    return r.stdout
old=json.loads((AUDIT/'protected.json').read_text())
old.update(json.loads((BASE/'protected_manifest.json').read_text()))
evidence=json.loads((AUDIT/'preservation.json').read_text())
old.update(evidence['candidate']);old.update(evidence['e1_executables'])
for name,h in json.loads((LIB/'complete.json').read_text())['artifacts'].items(): old[str(LIB/name)]=h
pinned=json.loads((HERE.parent/'e1b_local_profile_20261012/validation.json').read_text())['source_git_unchanged']
for repo,v in pinned.items():
    assert subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()==v['head']
    assert not subprocess.check_output(['git','-C',repo,'status','--porcelain'],text=True).strip()
for p,h in old.items():assert sha(p)==h,p
assert sha(AUDIT/'candidate/replay')=='9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
for p in (ROOT/'private_build').glob('*'):assert sha(p)==sha(BASE/'private_build'/p.name)
assert shutil.disk_usage(ROOT).free>15*1024**3
for p in [*ROOT.glob('*.py'),*ROOT.parent.glob('*.py'),*HERE.glob('*.py')]:ast.parse(p.read_text())
for p in [*ROOT.glob('*.sh'),ROOT/'source.env']:subprocess.run(['bash','-n',str(p)],check=True)
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
docker('nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv','gpu.log')
docker("python3 -c 'import socket; s=socket.socket(); assert s.connect_ex((\"127.0.0.1\",11406))!=0'",'port.log')
probe="""from pathlib import Path
hits=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  a=(p/'cmdline').read_bytes().split(b'\\0')
  if a and any(a[0].endswith(x) for x in [b'/glim_rosnode',b'/kimera_vio_ros_node',b'/replay']):hits.append((p.name,a[0].decode()))
 except OSError:pass
print(hits)
assert not hits
"""
for container in ['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']:
    docker('python3 -c '+shlex.quote(probe),'no_owned_'+container+'.log',container)
r=subprocess.run(['pgrep','-af','^python3 .*(run_case|launch_grandtour_live_receiver)'],capture_output=True,text=True)
assert r.returncode==1,r.stdout
libenv='export LD_LIBRARY_PATH='+inside(LIB)+':/workspace/cbs_gtsam4.3/devel/lib:$LD_LIBRARY_PATH && '
ldd=docker(setup+libenv+'ldd -r '+inside(LIB/'libglim_ros.so'),'ldd.log')
assert 'undefined symbol' not in ldd and 'not found' not in ldd
for name in ['libcbs.so','libgtsam_points_cuda.so.1']:assert inside(LIB/name) in ldd
test=WS/'runs/e1_correctness_audit_20261011/release_test'
docker(setup+libenv+inside(test)+' --gtest_output=xml:'+inside(OUT/'regression.xml'),'regression.log')
assert '[  PASSED  ] 18 tests.' in (OUT/'regression.log').read_text()
sys.path.insert(0,str(ROOT))
from experiment import launch_arguments,BINARY,ENV
params=docker(setup+shlex.join(['roslaunch','--dump-params',inside(ROOT/'active_window.launch'),*launch_arguments()]),'resolved_params.yaml')
kimera=docker(setup+shlex.join(['roslaunch','--args=/kimera_vio_ros/kimera_vio_ros_node',inside(ROOT/'active_window.launch'),*launch_arguments()]),'kimera_args.txt')
a=yaml.safe_load((BASE/'preflight/resolved_params.yaml').read_text());b=yaml.safe_load(params)
diff={k:[a.get(k),b.get(k)] for k in set(a)|set(b) if a.get(k)!=b.get(k)}
for k,(x,y) in diff.items():assert isinstance(x,str) and x.replace(inside(BASE),inside(ROOT))==y,(k,x,y)
assert (BASE/'preflight/kimera_args.txt').read_text().replace(inside(BASE),inside(ROOT))==kimera
cfg=json.loads((ROOT/'profile40_on.json').read_text());oldcfg=json.loads((BASE/'live160_on.json').read_text())
ignore=['sequence','ros_master_uri','sources','receiver_binary']
assert {k:v for k,v in cfg.items() if k not in ignore}=={k:v for k,v in oldcfg.items() if k not in ignore}
for x,y in zip(cfg['sources'],oldcfg['sources']):assert {k:v for k,v in x.items() if k!='session'}=={k:v for k,v in y.items() if k!='session'}
assert ENV['E2_EXACT_LINEARIZATION_CACHE']=='1'
assert (ROOT/'receiver_e2.sh').read_bytes()==(BASE/'receiver_e2.sh').read_bytes()
sys.path.insert(0,'/home/yeranis/repos/gnssFGO/tools')
from live_receiver_protocol import validate_config
validate_config(cfg)
bags=json.loads((BASE/'preflight/validation.json').read_text())['raw_bag_provenance']
for p,v in bags.items():
    s=Path(p).stat();assert s.st_size==v['bytes'] and s.st_mtime_ns==v['mtime_ns']
manifest=dict(old)
for p in [*ROOT.glob('*'),*ROOT.parent.glob('*.py'),ROOT.parent/'PROTOCOL.txt',*HERE.glob('*'),*ROOT.joinpath('private_build').glob('*'),
          Path('/home/yeranis/repos/gnssFGO/tools/launch_grandtour_live_receiver.py'),Path('/home/yeranis/repos/gnssFGO/tools/live_receiver_protocol.py')]:
    if p.is_file():manifest[str(p)]=sha(p)
with (ROOT/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
result=dict(passed=True,protected_entries=len(manifest),parameter_path_differences=diff,raw_bag_provenance=bags,
            receiver_binary=str(BINARY),receiver_sha256=sha(BINARY),wrapper_sha256=sha(ROOT/'receiver_e2.sh'),
            source_library_hashes={p.name:sha(p) for p in (ROOT/'private_build').glob('*')},
            prior_e2_evidence=str(AUDIT/'audit_summary.json'),new_e1_tests=18)
with (OUT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
print('E1B_PROFILE40_PREFLIGHT_PASS',len(manifest),'protected hashes; 18 tests; unchanged source settings')
