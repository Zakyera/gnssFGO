"""Mechanical matched-harness reuse for ETH1, no estimator/model edits."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_spx3_local_cov60_cpr5_20261019/prepared'
R=W/'runs/gnssfgo_eth1_local_cov60_cpr6_20261019/prepared'
F=H.parent/'local_covariance_spx3_cpr5_20261019'
S=W/'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared/live120_on_source'
A=H.parent/'checkpoints/20261017_fl4v/snapshot/profiles'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not R.parent.exists(),'refuse duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
assert json.loads((F/'final_checks.json').read_text())['passed']
manifest=json.loads((T/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
profiles={}
for folder in ('glim_config','kimera_profile'):
 profiles[folder]={str(p.relative_to(S/folder)):sha(p) for p in (S/folder).rglob('*') if p.is_file()}
 archived={str(p.relative_to(A/folder)):sha(p) for p in (A/folder).rglob('*') if p.is_file()}
 assert profiles[folder] and profiles[folder]==archived,folder
check=(H.parent/'fixed_lag_spx3_fl4w_20261017/inspect_bags.py').read_text()
check=check.replace('/GrandTour/SPX-3','/GrandTour').replace('2024-11-02-17-43-10','2024-10-01-11-29-55')
check=check.replace('start=epoch+32;end=start+60','start=epoch;end=start+60')
# At offset0 sensors naturally start at different sub-second times. Validate
# their actual start offsets, not the offset32 assumption of the SPX-3 script.
check=check.replace("out[k]['record_start']<=start", "out[k]['record_start']<=start+1")
# ETH1 camera-info begins 44.604ms before IE-TC, but IE-TC starts before the
# first CPT7 sample and every estimator output. Do not require GT for camera-info.
check=check.replace("gt['first'][topic]<=start", "gt['first'][topic]<=out['cpt7_imu']['stamp_coverage']['first']['/boxi/cpt7/imu']")
check=check.replace('seed_clock_seconds=int(start)-1,bag_offset=32','seed_clock_seconds=1727775007,bag_offset=0')
result=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
 'source /opt/ros/noetic/setup.bash; python3 -'],input=check,capture_output=True,text=True)
assert result.returncode==0,result.stdout+result.stderr
bags=json.loads(result.stdout);assert bags['passed']
baghash={str(W/Path(v['path']).relative_to('/workspace/cbs_gtsam4.3')):manifest[str(W/Path(v['path']).relative_to('/workspace/cbs_gtsam4.3'))]
 for k,v in bags.items() if k in ('hesai','cpt7_imu','alphasense','cpt7_ie_tc')}
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
for folder in profiles:shutil.copytree(S/folder,R/'input_profiles'/folder)
def common(s):
 return (s.replace(T.parent.name,R.parent.name)
  .replace('spx3_cpr5_live60_','eth1_cpr6_live60_').replace('11451','11452')
  .replace('2024-11-02-17-43-10','2024-10-01-11-29-55')
  .replace('/GrandTour/SPX-3','/GrandTour').replace('SPX-3','ETH1')
  .replace('CBSMS_BAG_START_SEC=32.0','CBSMS_BAG_START_SEC=0.0')
  .replace('CBSMS_SEED_CLOCK_SECS=1730565901','CBSMS_SEED_CLOCK_SECS=1727775007')
  .replace('"${bag_start_sec}" == 32.0','"${bag_start_sec}" == 0.0')
  .replace("['CBSMS_BAG_START_SEC']=='32.0'","['CBSMS_BAG_START_SEC']=='0.0'"))
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'map_guard.py','source.env','receiver_fl4e.sh','run_case.py','run_case_on.py',
 'verify_effective.py','verify_effective_on.py','source_runner.sh','source_runner_on.sh',
 'active_window.launch','active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live60_off.json','live60_on.json','validate_local.py',
 'evaluate_fl4f.py','preflight_on.py']
diff=[]
for name in files:
 old=(T/name).read_text();s=common(old)
 if name in ('live60_off.json','live60_on.json'):
  obj=json.loads(s);obj['sequence']='ETH1 LOCAL CP-R6 bag0-60s matched '+('ON' if '_on' in name else 'OFF')
  s=json.dumps(obj,indent=2)+'\n'
 (R/name).write_text(s);shutil.copymode(T/name,R/name)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(T/name),tofile=name+' CP-R6'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
for name in ('summarize.py','final_checks.py'):
 (H/name).write_text(common((F/name).read_text()))
(H/'inspect_bags.py').write_text(check)
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
for folder in profiles:
 for base in (S,A):
  for p in (base/folder).rglob('*'):
   if p.is_file():manifest[str(p)]=sha(p)
for base in (R,H):
 for p in base.rglob('*'):
  if p.is_file():manifest[str(p)]=sha(p)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
preflight=dict(passed=True,template=str(T),bags=bags,bag_sha256=baghash,
 profile_sha256=profiles,source_profile_origin=str(S),archive_profile_origin=str(A),
 requested_s=60,bag_start_s=0,protected_files=len(manifest),free_bytes=shutil.disk_usage(W).free,
 binaries_identical=True,receiver_model_identical=True,scoring_helpers_unchanged=True,
 initial_container_state='both stopped; started only for this authorized comparison')
(R/'preflight/validation.json').write_text(json.dumps(preflight,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps({k:v for k,v in preflight.items() if k!='profile_sha256'},indent=2))
