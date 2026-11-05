"""Mechanical reuse of validated CP-R4/R3 harness for matched SPX-3."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_eig2_local_cov60_off_cpr4_20261019/prepared'
O=W/'runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared'
R=W/'runs/gnssfgo_spx3_local_cov60_cpr5_20261019/prepared'
F=H.parent/'fixed_lag_spx3_fl4w_20261017'
S=W/'runs/spx3_full_system_revision_diagnostics_20260927/spx3_off_classic_revisions_nodelog'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
assert not R.parent.exists(),'refuse duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
assert json.loads((H.parent/'local_covariance_off60_cpr4_20261019/final_checks.json').read_text())['passed']
manifest=json.loads((T/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
old_preflight=json.loads((F/'preflight.json').read_text())
for p,h in old_preflight['bag_sha256'].items():assert sha(p)==h,p
bag_check=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
 'source /opt/ros/noetic/setup.bash; python3 -'],input=(F/'inspect_bags.py').read_text(),
 capture_output=True,text=True,check=True)
bags=json.loads(bag_check.stdout);assert bags['passed']
assert bags==old_preflight['bags'],'SPX-3 input metadata changed'
profiles={}
for folder,wanted in old_preflight['profile_sha256'].items():
 profiles[folder]={str(p.relative_to(S/folder)):sha(p) for p in (S/folder).rglob('*') if p.is_file()}
 assert profiles[folder]==wanted,folder
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
for folder in profiles:shutil.copytree(S/folder,R/'input_profiles'/folder)
def common(s):
 return (s.replace(T.parent.name,R.parent.name).replace(O.parent.name,R.parent.name)
  .replace('eig2_cpr4_live60_off_20261019','spx3_cpr5_live60_off_20261019')
  .replace('eig2_cpr3_live60_on_20261019','spx3_cpr5_live60_on_20261019')
  .replace('11450','11451').replace('11449','11451')
  .replace('2024-11-03-13-59-54','2024-11-02-17-43-10')
  .replace('EIG-2','SPX-3').replace('CBSMS_BAG_START_SEC=5.7','CBSMS_BAG_START_SEC=32.0')
  .replace('CBSMS_SEED_CLOCK_SECS=1730638815','CBSMS_SEED_CLOCK_SECS=1730565901')
  .replace('"${bag_start_sec}" == 5.7','"${bag_start_sec}" == 32.0')
  .replace("['CBSMS_BAG_START_SEC']=='5.7'","['CBSMS_BAG_START_SEC']=='32.0'"))
shared=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'map_guard.py','source.env','receiver_fl4e.sh','run_case.py','verify_effective.py',
 'source_runner.sh','active_window.launch','glim_profile.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live60_off.json','validate_local.py','evaluate_fl4f.py']
on=['run_case_on.py','verify_effective_on.py','source_runner_on.sh',
 'active_window_on.launch','glim_profile_on.launch','live60_on.json']
diff=[]
for name in shared+on:
 template=O if name in on else T
 old=(template/name).read_text();s=common(old)
 if name=='validate_case.py':s=once(s,"assert MODE == 'off'","assert MODE in ('off','on')")
 if name=='validate_local.py':s=once(s,"assert mode == 'off'","assert mode in ('off','on')")
 if name=='evaluate_fl4f.py':
  s=once(s,"ON_ROOT = Path('/workspace/cbs_gtsam4.3/runs/"+R.parent.name+"/prepared')","ON_ROOT = ROOT")
  s=s.replace('Run after matching OFF validates; read already-validated ON','Run after both matching cases validate')
 if name.endswith('.json'):
  obj=json.loads(s);obj['sequence']='SPX-3 LOCAL CP-R5 bag32-92s matched '+('ON' if name.endswith('_on.json') else 'OFF')
  s=json.dumps(obj,indent=2)+'\n'
 (R/name).write_text(s);shutil.copymode(template/name,R/name)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(template/name),tofile=name+' CP-R5'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
for name in ('PROTOCOL.txt','preflight_on.py'):shutil.copy2(H/name,R/name)
assert sha(R/'receiver_fl4e.sh')==sha(T/'receiver_fl4e.sh')
def config(p):
 obj=json.loads(p.read_text())
 for k in ('sequence','ros_master_uri','receiver_binary'):obj.pop(k)
 for source in obj['sources']:source.pop('session')
 return obj
assert config(R/'live60_off.json')==config(R/'live60_on.json')==config(T/'live60_off.json')
def functions(code):
 return {x.name:ast.dump(x,include_attributes=False) for x in ast.parse(code).body if isinstance(x,ast.FunctionDef)}
for name in ('sha','stat','prism_pose','path_length','score','score_saved','covariances'):
 assert functions((T/'evaluate_fl4f.py').read_text())[name]==functions((R/'evaluate_fl4f.py').read_text())[name]
for p in R.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 if p.suffix=='.launch':ET.parse(p)
 if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
manifest.update(old_preflight['bag_sha256'])
for folder in profiles:
 for p in (S/folder).rglob('*'):
  if p.is_file():manifest[str(p)]=sha(p)
for p in R.rglob('*'):
 if p.is_file():manifest[str(p)]=sha(p)
for name in ('prepare.py','PROTOCOL.txt','preflight_on.py'):manifest[str(H/name)]=sha(H/name)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=dict(passed=True,templates=[str(T),str(O)],bags=bags,
 bag_sha256=old_preflight['bag_sha256'],profile_sha256=profiles,source_profile_origin=str(S),
 requested_s=60,bag_start_s=32,protected_files=len(manifest),free_bytes=shutil.disk_usage(W).free,
 binaries_identical=True,receiver_model_identical=True,scoring_helpers_unchanged=True,
 initial_container_state='both stopped; started only for this authorized comparison')
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps({k:v for k,v in result.items() if k not in ('bags','profile_sha256')},indent=2))
