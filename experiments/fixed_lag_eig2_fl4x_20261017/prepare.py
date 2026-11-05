"""Mechanical FL4w harness reuse; only EIG-2 identity/window/profile paths change."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_spx3_live60_off_on_fl4w_20261017/prepared'
R=W/'runs/gnssfgo_eig2_live60_off_on_fl4x_20261017/prepared'
S=W/'runs/grandtour_eig2_glim_kimera/eig2_stereo_shared_cpt7_cbs_off_full305_start5p7_rate010_20260921'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
def inside(p):return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))

assert not R.parent.exists(),'Refusing duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
manifest=json.loads((T/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
subprocess.run(['python3',str(H.parent/'checkpoints/20261017_fl4v/verify_archive.py'),'--originals'],check=True)
check=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
 'source /opt/ros/noetic/setup.bash; python3 -'],input=(H/'inspect_bags.py').read_text(),
 capture_output=True,text=True,check=True)
bags=json.loads(check.stdout);assert bags['passed']
seed=str(bags['window']['seed_clock_seconds'])
profiles={}
for folder in ('glim_config','kimera_profile'):
 profiles[folder]={str(p.relative_to(S/folder)):sha(p) for p in (S/folder).rglob('*') if p.is_file()}
 baseline={str(p.relative_to(T/'input_profiles'/folder)):sha(p) for p in (T/'input_profiles'/folder).rglob('*') if p.is_file()}
 assert profiles[folder] and profiles[folder]==baseline,(folder,'profile differs: review before launch')
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
for folder in profiles:shutil.copytree(S/folder,R/'input_profiles'/folder)
def common(s):
 return (s.replace(T.parent.name,R.parent.name)
  .replace('gnssfgo_spx3_live60_fl4w_20261017','gnssfgo_eig2_live60_fl4x_20261017')
  .replace('spx3_fl4w_live60_','eig2_fl4x_live60_').replace('11445','11446'))
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'evaluate_fl4f.py','compare_belief_cov_fl4f.py','preflight_on_fl4f.py','map_guard.py',
 'source.env','receiver_fl4e.sh','run_case.py','run_case_on.py','verify_effective.py',
 'verify_effective_on.py','source_runner.sh','source_runner_on.sh','active_window.launch',
 'active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live60_off.json','live60_on.json']
diff=[]
for n in files:
 old=(T/n).read_text();s=common(old)
 if n=='source.env':
  s=once(s,'CBSMS_MISSION=2024-11-02-17-43-10','CBSMS_MISSION=2024-11-03-13-59-54')
  s=once(s,'CBSMS_BAG_START_SEC=32.0','CBSMS_BAG_START_SEC=5.7')
  s=once(s,'CBSMS_SEED_CLOCK_SECS=1730565901','CBSMS_SEED_CLOCK_SECS='+seed)
  s=once(s,'datasets/GrandTour/SPX-3','datasets/GrandTour/EIG-2')
 if n.startswith('source_runner'):
  s=once(s,'"${bag_start_sec}" == 32.0','"${bag_start_sec}" == 5.7')
  s=s.replace('SPX-3','EIG-2')
 if n.startswith('verify_effective'):
  s=once(s,"['CBSMS_MISSION']=='2024-11-02-17-43-10'","['CBSMS_MISSION']=='2024-11-03-13-59-54'")
  s=once(s,"['CBSMS_BAG_START_SEC']=='32.0'","['CBSMS_BAG_START_SEC']=='5.7'")
 if n=='evaluate_fl4f.py':
  s=once(s,'GrandTour/SPX-3/2024-11-02-17-43-10_cpt7_ie_tc.bag','GrandTour/EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag')
 if n.endswith('.json'):
  cfg=json.loads(s);cfg['sequence']='EIG-2 bag5.7–65.7s FL4x matched live60 '+('OFF' if 'off' in n else 'ON')
  s=json.dumps(cfg,indent=2)+'\n'
 target=R/n;target.write_text(s);shutil.copymode(T/n,target)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=n+' FL4w',tofile=n+' FL4x'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):
 old=p.read_text();new=common(old)
 if old!=new:p.write_text(new)
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
assert sha('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')=='53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'
assert sha(R/'private_build/libkimera_vio.so')=='6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
assert sha(R/'receiver_fl4e.sh')==sha(T/'receiver_fl4e.sh')
for p in (R/'private_build').rglob('*'):
 if p.is_file():assert sha(p)==sha(T/'private_build'/p.relative_to(R/'private_build'))
for mode in ('off','on'):
 old=json.loads((T/f'live60_{mode}.json').read_text());new=json.loads((R/f'live60_{mode}.json').read_text())
 for c in (old,new):
  for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
  for src in c['sources']:src.pop('session')
 assert old==new,'Receiver model config changed'
for p in R.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 if p.suffix=='.launch':ET.parse(p)
 if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
bag_hashes={}
for kind in ('hesai','cpt7_imu','alphasense','cpt7_ie_tc'):
 p=W/('src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_'+kind+'.bag')
 bag_hashes[str(p)]=sha(p)
manifest.update(bag_hashes)
for folder in profiles:
 for p in (S/folder).rglob('*'):
  if p.is_file():manifest[str(p)]=sha(p)
for p in R.rglob('*'):
 if p.is_file():manifest[str(p)]=sha(p)
for n in ('prepare.py','PROTOCOL.txt','inspect_bags.py'):manifest[str(H/n)]=sha(H/n)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=dict(passed=True,bags=bags,bag_sha256=bag_hashes,profile_sha256=profiles,
 profiles_verified_identical_to_fl4w_eth1=True,source_profile_origin=str(S),
 binaries_identical=True,receiver_model_identical=True,protected_files=len(manifest),
 initial_container_state='both stopped; restored for authorized capture',free_bytes=shutil.disk_usage(W).free)
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'preflight.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps({k:v for k,v in result.items() if k not in ('profile_sha256',)},indent=2))
