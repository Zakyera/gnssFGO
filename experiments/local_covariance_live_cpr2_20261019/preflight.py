"""Read-only checks of prepared launch and split-log validator; emit new evidence."""
from pathlib import Path
import ast,hashlib,json,sys
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
T=W/'runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared'
sys.path.insert(0,str(P/'transport_tools'))
from live_receiver_protocol import parse_belief
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
protected=json.loads((P/'protected_manifest.json').read_text())
for path,h in protected.items():assert sha(path)==h,path
for path in P.glob('*'):
 if path.is_file() and path.suffix in ('.py','.sh','.env','.launch','.json') and path.name not in ('protected_manifest.json',):
  text=path.read_text()
  assert T.parent.name not in text,path
  assert '11447' not in text,path
assert not (P/'run_case_on.py').exists() and not (P/'live30_on.json').exists()
assert 'duration=30' in (P/'run_case.py').read_text()
assert 'min_glim_rows=280' in (P/'source_runner.sh').read_text()
assert 'min_kimera_rows=135' in (P/'source_runner.sh').read_text()
# Exercise the revised validator's actual loop on retained Kimera logs only.
# No complete-capture/provenance claim about the old failed run is made.
tree=ast.parse((P/'validate_local.py').read_text())
loop=next(n for n in tree.body if isinstance(n,ast.For) and isinstance(n.target,ast.Tuple))
loop.iter=ast.parse("[('kimera','kimera_front','*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]",mode='eval').body
namespace={'S':T/'live60_off_source','expected':{},'result':{},'parse_belief':parse_belief}
exec('import collections,re,numpy as np',namespace)
exec(compile(ast.fix_missing_locations(ast.Module(body=[loop],type_ignores=[])),str(P/'validate_local.py'),'exec'),namespace)
k=namespace['result']['kimera'];assert k['versions']>0 and k['local_joint_queries']>0
effective=json.loads((T/'live60_off_source/effective_mode_check.json').read_text())
maps=effective['glim_maps'].replace('/shared_linearization_s4_provider_20261012/release_2/libgtsam_points.so.1.2.1','/native_covariance_cache_cpi3_20261019/release_v2/libgtsam_points.so.1.2.1')
with (H/'map_fixture.txt').open('x') as f:f.write(maps)
result=dict(passed=True,protected_unchanged=len(protected),kimera_validator_retained_data=k,
 scope='prepared OFF-only harness; exact revised split-log parser exercised on retained data; no estimation')
with (H/'preflight.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
