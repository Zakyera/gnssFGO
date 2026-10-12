"""Read-only admission gates for the single authorized shadow capture."""
from pathlib import Path
import ast
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import yaml
from prepare import WS,RUN,ROOT,BUILD,OLD,inside,HERE,sha

out=ROOT/(sys.argv[1] if len(sys.argv)>1 else 'preflight');out.mkdir(exist_ok=False)
assert not (ROOT/'preflight/validation.json').exists()
assert json.loads((RUN/'no_data_startup/result.json').read_text())['passed']
assert json.loads((RUN/'no_data_startup/ready.json').read_text())['source_measurements']==0
assert json.loads((HERE/'synthetic_oracle.json').read_text())['passed']
assert json.loads((HERE/'mixed_oracle.json').read_text())['passed']
assert json.loads((BUILD/'tests/result.json').read_text())['returncode']==0
assert json.loads((BUILD.parent/'asan_1/tests/result.json').read_text())['returncode']==0
assert json.loads((BUILD/'nonregression/result.json').read_text())['disabled_main_byte_identical']
assert shutil.disk_usage(ROOT).free>15*1024**3
protected=json.loads((OLD/'prepared/protected_manifest.json').read_text())
for p,h in protected.items():assert sha(Path(p))==h,p
for p in [*ROOT.glob('*.py'),*RUN.glob('*.py')]:ast.parse(p.read_text())
for p in [*ROOT.glob('*.sh'),ROOT/'source.env']:subprocess.run(['bash','-n',str(p)],check=True)
def docker(cmd,name,container='cbsms_ws_gpu'):
    r=subprocess.run(['docker','exec',container,'bash','-lc',cmd],capture_output=True,text=True)
    (out/name).write_text(r.stdout+r.stderr);assert r.returncode==0,(name,r.stderr[-2000:]);return r.stdout
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
    docker('python3 -c '+shlex.quote(probe),'processes_'+container+'.log',container)
r=subprocess.run(['pgrep','-af','^python3 .*(run_case|launch_grandtour_live_receiver)'],capture_output=True,text=True)
assert r.returncode==1,r.stdout
docker("python3 -c 'import socket; s=socket.socket(); assert s.connect_ex((\"127.0.0.1\",11420))!=0'",'port.log')
docker('nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv','gpu.log')
docker('python3 '+inside(ROOT/'map_guard.py')+' '+inside(BUILD)+' '+inside(RUN/'no_data_startup/maps.txt'),'map_guard.log')
sys.path.insert(0,str(ROOT));from experiment import launch_arguments,BINARY,ENV
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
params=docker(setup+shlex.join(['roslaunch','--dump-params',inside(ROOT/'active_window.launch'),*launch_arguments()]),'resolved_params.yaml')
kimera=docker(setup+shlex.join(['roslaunch','--args=/kimera_vio_ros/kimera_vio_ros_node',inside(ROOT/'active_window.launch'),*launch_arguments()]),'kimera_args.txt')
baseline=(OLD/'prepared/preflight/resolved_params.yaml').read_text().replace(OLD.name,RUN.name)
assert yaml.safe_load(params)==yaml.safe_load(baseline),'source params changed'
assert kimera==(OLD/'prepared/preflight/kimera_args.txt').read_text().replace(OLD.name,RUN.name)
assert (ROOT/'receiver_e2.sh').read_bytes()==(OLD/'prepared/receiver_e2.sh').read_bytes()
cfg=json.loads((ROOT/'profile40_on.json').read_text());prev=json.loads((OLD/'prepared/profile40_on.json').read_text())
prev['receiver_binary']=prev['receiver_binary'].replace(OLD.name,RUN.name)
ignore={'sequence','ros_master_uri','sources'}
assert {k:v for k,v in cfg.items() if k not in ignore}=={k:v for k,v in prev.items() if k not in ignore}
for x,y in zip(cfg['sources'],prev['sources']):assert {k:v for k,v in x.items() if k!='session'}=={k:v for k,v in y.items() if k!='session'}
assert ENV['E2_EXACT_LINEARIZATION_CACHE']=='1'
assert sha(BINARY)=='9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
bags=json.loads((OLD/'prepared/preflight/validation.json').read_text())['raw_bag_provenance']
for p,v in bags.items():
    stat=Path(p).stat();assert stat.st_size==v['bytes'] and stat.st_mtime_ns==v['mtime_ns']
# Every source DSO selected by the new map guard is pinned, resolving container
# absolute symlinks inside the container. No global library replacement.
library_json=docker('python3 -c '+shlex.quote("from pathlib import Path; import hashlib,json; p=Path('"+inside(BUILD)+"'); print(json.dumps({str(q.resolve()):hashlib.sha256(q.read_bytes()).hexdigest() for q in p.glob('lib*.so*')}))"),'libraries.json')
for p,h in json.loads(library_json).items():
    host=WS/p.removeprefix('/workspace/cbs_gtsam4.3/');assert sha(host)==h;protected[str(host)]=h
for p in [*ROOT.glob('*'),*RUN.glob('*.py'),RUN/'PROTOCOL.md',*HERE.glob('*.py'),HERE/'Recording.h',
          *ROOT.joinpath('private_build').glob('*')]:
    if p.is_file():protected[str(p)]=sha(p)
(ROOT/'protected_manifest.json').write_text(json.dumps(protected,indent=2))
(ROOT/'preflight/validation.json').write_text(json.dumps(dict(passed=True,evidence=str(out),protected_entries=len(protected),
  source_parameters_equal_to_B0=True,kimera_flags_equal_to_B0=True,receiver_model_equal_to_B0=True,
  raw_bag_provenance=bags,private_cpu_sha256=sha(BUILD/'libgtsam_points.so.1.2.1'),
  private_bridge_sha256=sha(BUILD/'libglim_ros.so'),no_data_startup=str(RUN/'no_data_startup')),indent=2))
print('S3_CAPTURE_PREFLIGHT_PASS',len(protected))
