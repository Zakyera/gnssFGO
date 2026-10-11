"""Freeze one bounded performance retest without touching previous artifacts."""
import ast
import json
import shlex
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from experiment import ROOT, REPO, file_sha, inside, launch_arguments

OLD = ROOT.parents[1]/'gnssfgo_eth1_covariance160_20261010/prepared'
WS = ROOT.parents[2]
out = ROOT/'preflight'
assert not out.exists()
manifest = json.loads((OLD/'protected_manifest.json').read_text())
allowed = {str(WS/'src/glim_ros1'/f) for f in
           ['src/glim_ros/cbs_bridge.cpp','include/glim_ros/cbs_bridge.hpp']}
changed = []
for path, sha in manifest.items():
    current = file_sha(path)
    if current != sha:
        assert path in allowed, ('unexpected protected change',path)
        before = ROOT.parent/'before'/Path(path).name
        assert file_sha(before) == sha
        changed.append(path)
        manifest[path] = current
assert set(changed) == allowed
for f in ['budget_test.xml','budget_test_sanitized.xml']:
    suite=ET.parse(ROOT.parent/f).getroot()
    assert suite.attrib['tests']=='6' and suite.attrib['failures']=='0'
for p in ROOT.glob('*.py'): ast.parse(p.read_text())
for p in ['source.env','source_runner.sh']:
    subprocess.run(['bash','-n',str(ROOT/p)],check=True)
guard, = [s for s in (ROOT/'source_runner.sh').read_text().splitlines()
          if s.startswith('[[ "${cbs_enable}"')]
for enabled in ['true','false']:
    for duration in ['10.0','40.0','100.0','160.0']:
        cmd=f'cbs_enable={enabled}; cbs_direction=bidirectional; duration_sec={duration}; bag_start_sec=0.0; '+guard
        rc=subprocess.run(['bash','-c',cmd],capture_output=True).returncode
        assert (rc==0)==(enabled=='true' and duration=='100.0')
with socket.socket() as s: assert s.connect_ex(('127.0.0.1',11396)) != 0
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
def resolve(option):
    return subprocess.check_output(['docker','exec','cbsms_ws_gpu','bash','-lc',
        setup+shlex.join(['roslaunch',option,inside(ROOT/'active_window.launch'),*launch_arguments()])],text=True)
params=resolve('--dump-params')
args=resolve('--args=/kimera_vio_ros/kimera_vio_ros_node')
files=resolve('--files')
old=yaml.safe_load((OLD/'preflight/resolved_params.yaml').read_text())
new=yaml.safe_load(params)
differences={k:[old.get(k),new.get(k)] for k in set(old)|set(new) if old.get(k)!=new.get(k)}
diag={'/glim_rosnode/glim_cbs_active_factor_diagnostics_enable':False,
      '/glim_rosnode/glim_cbs_active_factor_diagnostics_stride':20,
      '/glim_rosnode/glim_cbs_active_factor_diagnostics_max_updates':40}
for k,(a,b) in differences.items():
    if k in diag: assert a is None and b==diag[k]
    else: assert isinstance(a,str) and isinstance(b,str) and a.replace(str(inside(OLD)),inside(ROOT))==b,(k,a,b)
assert (OLD/'preflight/kimera_args.txt').read_text().replace(inside(OLD),inside(ROOT))==args
config=json.loads((OLD/'full160_on.json').read_text())
config['sequence']='ETH1 first100s; ON diagnostic-cost retest; unchanged source and WNOA mathematics'
config['ros_master_uri']='http://127.0.0.1:11396'
for source in config['sources']: source['session']='eth1_diagnostic_budget_on_20261010'
sys.path.insert(0,str(REPO/'tools'))
from live_receiver_protocol import validate_config
validate_config(config)
with (ROOT/'perf100_on.json').open('x') as f: json.dump(config,f,indent=2)
build=json.loads((ROOT/'private_build/build.json').read_text())
assert file_sha(ROOT/'private_build/libglim_ros.so')==build['sha256']
for p in [ROOT/'private_build/libglim_ros.so',
          WS/'src/glim_ros1/include/glim_ros/cbs_diagnostic_budget.hpp',
          WS/'src/glim_ros1/test/cbs_diagnostic_budget_test.cpp',
          WS/'src/glim_ros1/CMakeLists.txt']:
    manifest[str(p)]=file_sha(p)
for p in ROOT.iterdir():
    if p.is_file(): manifest[str(p)]=file_sha(p)
out.mkdir()
for name,value in [('resolved_params.yaml',params),('kimera_args.txt',args),('launch_files.txt',files)]:
    (out/name).write_text(value)
result=dict(passed=True,duration_s=100,rate=.1,port=11396,protected_originals=len(manifest),
    allowed_source_changes=changed,parameter_differences=differences,private_library_sha=build['sha256'],
    interpretation='engineering throughput retest, not a matched accuracy comparison',
    baseline_off_validation=str(OLD/'full160_off_validation.json'))
(out/'validation.json').write_text(json.dumps(result,indent=2))
(ROOT/'protected_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(result,indent=2))
