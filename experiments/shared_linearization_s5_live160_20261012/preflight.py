"""Frozen-build admission and isolated harness checks before one capacity run."""
import ast
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import yaml
from prepare import HERE,WS,OLD,RUN,ROOT,BUILD,PRIOR,inside,sha,transform

out=ROOT/'preflight'
out.mkdir(exist_ok=False)
assert not (ROOT/'capacity160_on_control').exists()
assert json.loads((OLD/'validation_logs/complete.json').read_text())['passed']
assert json.loads((OLD/'no_data_startup/result.json').read_text())['passed']
assert json.loads((BUILD/'tests/result.json').read_text())['returncode']==0
assert json.loads((BUILD.parent/'asan_nopie/tests/result.json').read_text())['returncode']==0
assert json.loads((BUILD/'nonregression/result.json').read_text())['disabled_main_byte_identical']
assert sha(BUILD/'libglim_ros.so')=='0114b435785ae85f58a69f1156abd1a2a320dc7138ca0b2dd4bfd56715d4ebea'
assert sha(BUILD/'libgtsam_points.so.1.2.1')=='42fb1e8a931ebc86a06ae41a2eaf6ff39b9efbe5a1a4fd9737ee06edb2274228'
assert shutil.disk_usage(ROOT).free>15*1024**3
protected=json.loads((OLD/'prepared/protected_manifest.json').read_text())
for p,h in protected.items():assert sha(p)==h,p
repos=json.loads((PRIOR/'before_integrity.json').read_text())['source_repositories']
for p,v in repos.items():
    for cmd,expected in [(['rev-parse','HEAD'],v['head']),(['status','--porcelain'],v['status'])]:
        got=subprocess.check_output(['git','-C',p,*cmd],text=True).strip()
        assert got==expected,(p,cmd,got)
for p in [*ROOT.glob('*.py'),*RUN.glob('*.py'),*HERE.glob('*.py')]:ast.parse(p.read_text())
for p in [*ROOT.glob('*.sh'),ROOT/'source.env']:subprocess.run(['bash','-n',str(p)],check=True)

def docker(cmd,name,container='cbsms_ws_gpu'):
    r=subprocess.run(['docker','exec',container,'bash','-lc',cmd],capture_output=True,text=True)
    (out/name).write_text(r.stdout+r.stderr)
    assert r.returncode==0,(name,r.stderr[-2000:])
    return r.stdout

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
r=subprocess.run(['pgrep','-af','^python3 .*(run_case|launch_grandtour_live_receiver|validate_numerical_case)'],capture_output=True,text=True)
assert r.returncode==1,r.stdout
docker("python3 -c 'import socket; s=socket.socket(); assert s.connect_ex((\"127.0.0.1\",11424))!=0'",'port.log')
docker('nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv','gpu.log')
docker('python3 '+inside(ROOT/'map_guard.py')+' '+inside(BUILD)+' '+inside(OLD/'no_data_startup/maps.txt'),'retained_startup_map_guard.log')
sys.path.insert(0,str(ROOT))
from experiment import launch_arguments,BINARY,ENV
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
params=docker(setup+shlex.join(['roslaunch','--dump-params',inside(ROOT/'active_window.launch'),*launch_arguments()]),'resolved_params.yaml')
kimera=docker(setup+shlex.join(['roslaunch','--args=/kimera_vio_ros/kimera_vio_ros_node',inside(ROOT/'active_window.launch'),*launch_arguments()]),'kimera_args.txt')
prior_admission=json.loads((OLD/'prepared/preflight/validation.json').read_text())
prior_evidence=Path(prior_admission['evidence'])
assert yaml.safe_load(params)==yaml.safe_load(transform((prior_evidence/'resolved_params.yaml').read_text())),'source params changed'
assert kimera==transform((prior_evidence/'kimera_args.txt').read_text())
assert (ROOT/'receiver_e2.sh').read_bytes()==(OLD/'prepared/receiver_e2.sh').read_bytes()
cfg=json.loads((ROOT/'capacity160_on.json').read_text())
prev=json.loads(transform((OLD/'prepared/profile40_on.json').read_text()))
assert {k:v for k,v in cfg.items() if k!='sequence'}=={k:v for k,v in prev.items() if k!='sequence'}
assert ENV['E2_EXACT_LINEARIZATION_CACHE']=='1'
assert sha(BINARY)=='9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
bags=json.loads((OLD/'prepared/preflight/validation.json').read_text())['raw_bag_provenance']
for p,v in bags.items():
    stat=Path(p).stat();assert stat.st_size==v['bytes'] and stat.st_mtime_ns==v['mtime_ns']
libraries=docker('python3 -c '+shlex.quote("from pathlib import Path; import hashlib,json; p=Path('"+inside(BUILD)+"'); print(json.dumps({str(q.resolve()):hashlib.sha256(q.read_bytes()).hexdigest() for q in p.glob('lib*.so*')}))"),'libraries.json')
for p,h in json.loads(libraries).items():
    host=WS/p.removeprefix('/workspace/cbs_gtsam4.3/')
    assert sha(host)==h;protected[str(host)]=h
# Same compiled fixture, new result directory. No model or build change.
fixture=json.loads((BUILD/'tests/result.json').read_text())
assert sha(BUILD/'test_shadow')==fixture['executable_sha256']
started=time.monotonic()
docker(shlex.join(['env','LD_LIBRARY_PATH='+fixture['LD_LIBRARY_PATH'],inside(BUILD/'test_shadow'),
                  '--gtest_output=xml:'+inside(out/'release_tests.xml')]),'release_tests.log')
xml=ET.parse(out/'release_tests.xml').getroot()
assert int(xml.attrib['tests'])==47 and int(xml.attrib['failures'])==0 and int(xml.attrib['errors'])==0
(out/'release_tests.json').write_text(json.dumps(dict(passed=True,tests=47,seconds=time.monotonic()-started,
    executable_sha256=fixture['executable_sha256']),indent=2))
for p in [*ROOT.glob('*'),*RUN.glob('*.py'),RUN/'PROTOCOL.md',*HERE.glob('*.py'),HERE/'PROTOCOL.md',
          *ROOT.joinpath('private_build').glob('*')]:
    if p.is_file():protected[str(p)]=sha(p)
(ROOT/'protected_manifest.json').write_text(json.dumps(protected,indent=2))
(out/'validation.json').write_text(json.dumps(dict(passed=True,protected_entries=len(protected),
    source_parameters_equal_to_S4d=True,kimera_flags_equal_to_S4d=True,receiver_model_equal_to_S4d=True,
    raw_bag_provenance=bags,source_repositories=repos,release_tests=47,
    prior_sanitizer_evidence=str(BUILD.parent/'asan_nopie/tests/result.json'),
    prior_startup=str(OLD/'no_data_startup'),actual_new_process_admission_required=True,
    private_bridge_sha256=sha(BUILD/'libglim_ros.so'),private_cpu_sha256=sha(BUILD/'libgtsam_points.so.1.2.1')),indent=2))
print('S5_PREFLIGHT_PASS',len(protected))
