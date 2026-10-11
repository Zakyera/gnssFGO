"""Validate and freeze the existing harness at 160 s; no estimator changes."""
import ast
import json
import shlex
import socket
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from experiment import ROOT, REPO, file_sha, inside, launch_arguments

PERF = ROOT.parents[1]/'glim_cbs_diagnostic_budget_20261010'
OFF = ROOT.parents[1]/'gnssfgo_eth1_covariance160_20261010/prepared'
out = ROOT/'preflight'
assert not out.exists()
assert json.loads((PERF/'FINAL_VALIDATION.json').read_text())['passed']
manifest = json.loads((PERF/'prepared/protected_manifest.json').read_text())
for path, sha in manifest.items():
    assert file_sha(path) == sha, ('protected input changed',path)
assert file_sha(ROOT/'private_build/libglim_ros.so') == file_sha(PERF/'prepared/private_build/libglim_ros.so')
off = json.loads((OFF/'full160_off_validation.json').read_text())
assert off['passed'] and off['coverage']['requested_s'] == 160
assert (OFF/'full160_off_control/complete.json').exists()
assert not (OFF/'full160_off_control/failed.txt').exists()
for name, result in off['repeat_files'].items():
    assert result['byte_identical']
    p = OFF/'full160_off_receiver/receiver'/name
    assert file_sha(p) == result['live_sha256']
for p in [OFF/'full160_off_validation.json', OFF/'full160_off_source/run_metadata.txt',
          OFF/'full160_off_source/glim_odom.tum', OFF/'full160_off_source/kimera_odom.tum']:
    if p.exists(): manifest[str(p)] = file_sha(p)
for p in (OFF/'full160_off_source').rglob('*.tum'):
    manifest[str(p)] = file_sha(p)
for name in off['repeat_files']:
    p = OFF/'full160_off_receiver/receiver'/name
    manifest[str(p)] = file_sha(p)
for p in ROOT.glob('*.py'): ast.parse(p.read_text())
for p in ROOT.parent.glob('*.py'): ast.parse(p.read_text())
for p in ['source.env','source_runner.sh']:
    subprocess.run(['bash','-n',str(ROOT/p)],check=True)
guard, = [s for s in (ROOT/'source_runner.sh').read_text().splitlines() if s.startswith('[[ "${cbs_enable}"')]
for enabled in ['true','false']:
    for duration in ['40.0','100.0','160.0','346.0']:
        cmd=f'cbs_enable={enabled}; cbs_direction=bidirectional; duration_sec={duration}; bag_start_sec=0.0; '+guard
        assert (subprocess.run(['bash','-c',cmd],capture_output=True).returncode == 0) == (enabled=='true' and duration=='160.0')
for name in ['budget_test.xml','budget_test_sanitized.xml']:
    suite=ET.parse(PERF/name).getroot()
    assert suite.attrib['tests']=='6' and suite.attrib['failures']=='0'
subprocess.run(['python3',str(PERF/'test_change_scope.py')],check=True)
with socket.socket() as s: assert s.connect_ex(('127.0.0.1',11398)) != 0
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
def resolve(option):
    return subprocess.check_output(['docker','exec','cbsms_ws_gpu','bash','-lc',
        setup+shlex.join(['roslaunch',option,inside(ROOT/'active_window.launch'),*launch_arguments()])],text=True)
params=resolve('--dump-params')
args=resolve('--args=/kimera_vio_ros/kimera_vio_ros_node')
files=resolve('--files')
old=yaml.safe_load((PERF/'prepared/preflight/resolved_params.yaml').read_text())
new=yaml.safe_load(params)
differences={k:[old.get(k),new.get(k)] for k in set(old)|set(new) if old.get(k)!=new.get(k)}
for k,(a,b) in differences.items():
    assert isinstance(a,str) and isinstance(b,str) and a.replace(inside(PERF/'prepared'),inside(ROOT))==b,(k,a,b)
assert (PERF/'prepared/preflight/kimera_args.txt').read_text().replace(inside(PERF/'prepared'),inside(ROOT))==args
config=json.loads((ROOT/'full160_on.json').read_text())
old_config=json.loads((PERF/'prepared/perf100_on.json').read_text())
for k in config:
    if k not in ['sequence','ros_master_uri','sources']: assert config[k]==old_config[k],k
for a,b in zip(config['sources'],old_config['sources']):
    assert {k:v for k,v in a.items() if k!='session'} == {k:v for k,v in b.items() if k!='session'}
assert config['ros_master_uri']=='http://127.0.0.1:11398'
import sys
sys.path.insert(0,str(REPO/'tools'))
from live_receiver_protocol import validate_config
validate_config(config)
for p in [*ROOT.glob('*'), *ROOT.parent.glob('*.py'), ROOT.parent/'PROTOCOL.txt',
          ROOT/'private_build/libglim_ros.so',ROOT/'private_build/build.json']:
    if p.is_file(): manifest[str(p)]=file_sha(p)
out.mkdir()
for name,value in [('resolved_params.yaml',params),('kimera_args.txt',args),('launch_files.txt',files)]:
    (out/name).write_text(value)
result=dict(passed=True,duration_s=160,rate=.1,port=11398,protected_entries=len(manifest),
    parameter_differences=differences,private_library_sha=file_sha(ROOT/'private_build/libglim_ros.so'),
    reused_off=str(OFF/'full160_off_validation.json'),
    off_reuse_reason='Diagnostic function returns immediately in observe_only before changed body, in both versions; mathematical code unchanged. Live scheduling is not claimed identical.')
(out/'validation.json').write_text(json.dumps(result,indent=2))
(ROOT/'protected_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(result,indent=2))
