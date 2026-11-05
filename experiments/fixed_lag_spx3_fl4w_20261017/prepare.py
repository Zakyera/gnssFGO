"""Mechanical FL4t/FL4v harness reuse for SPX-3; no estimator changes."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess,types
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
R=W/'runs/gnssfgo_spx3_live60_off_on_fl4w_20261017/prepared'
S=W/'runs/spx3_full_system_revision_diagnostics_20260927/spx3_off_classic_revisions_nodelog'
E=W/'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared/live120_on_source'
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
bag_check=subprocess.run(['docker','exec','-i','cbsms_ws_gpu','bash','-lc',
 'source /opt/ros/noetic/setup.bash; python3 -'],input=(H/'inspect_bags.py').read_text(),
 capture_output=True,text=True,check=True)
bags=json.loads(bag_check.stdout);assert bags['passed']
seed=str(bags['window']['seed_clock_seconds'])
profiles={}
for folder in ('glim_config','kimera_profile'):
 profiles[folder]={str(p.relative_to(S/folder)):sha(p) for p in (S/folder).rglob('*') if p.is_file()}
 assert profiles[folder]
 # Retained SPX-3 profiles happen to match ETH1 byte-for-byte. Validate,
 # record and use the SPX-3 provenance; never assume common calibration.
 eth={str(p.relative_to(E/folder)):sha(p) for p in (E/folder).rglob('*') if p.is_file()}
 assert profiles[folder]==eth,(folder,'unexpected profile change requires review')
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
for folder in profiles:shutil.copytree(S/folder,R/'input_profiles'/folder)
def common(s):
 return (s.replace(T.parent.name,R.parent.name).replace('11442','11445')
  .replace('gnssfgo_fixed_lag_live_fl4t_20261016','gnssfgo_spx3_live60_fl4w_20261017')
  .replace('eth1_fl4t_live120_','spx3_fl4w_live60_').replace('_20261016','_20261017')
  .replace('live120','live60'))
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'evaluate_fl4f.py','compare_belief_cov_fl4f.py','preflight_on_fl4f.py','map_guard.py',
 'source.env','receiver_fl4e.sh','run_case.py','run_case_on.py','verify_effective.py',
 'verify_effective_on.py','source_runner.sh','source_runner_on.sh','active_window.launch',
 'active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live120_off.json','live120_on.json']
diff=[]
for n in files:
 old=(T/n).read_text();s=common(old)
 if n=='source.env':
  s=once(s,'CBSMS_MISSION=2024-10-01-11-29-55','CBSMS_MISSION=2024-11-02-17-43-10')
  s=once(s,'CBSMS_BAG_START_SEC=0.0','CBSMS_BAG_START_SEC=32.0')
  s=once(s,':-120.0',':-60.0')
  s=once(s,'CBSMS_SEED_CLOCK_SECS=1727775007','CBSMS_SEED_CLOCK_SECS='+seed)
  for folder,var in [('glim_config','GLIM_CONFIG'),('kimera_profile','KIMERA_PROFILE')]:
   oldline=next(l for l in s.splitlines() if l.startswith('export CBSMS_'+var+'_SOURCE='))
   s=once(s,oldline,'export CBSMS_'+var+'_SOURCE='+inside(R/'input_profiles'/folder))
  s+='\nexport CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour/SPX-3\n'
 if n.startswith('source_runner'):
  s=once(s,'"${duration_sec}" == 120.0','"${duration_sec}" == 60.0')
  s=once(s,'"${bag_start_sec}" == 0.0','"${bag_start_sec}" == 32.0')
  s=s.replace('Prepared 120-second','Prepared SPX-3 60-second')
  s=once(s,'min_glim_rows=1170','min_glim_rows=570')
  s=once(s,'min_kimera_rows=570','min_kimera_rows=270')
  s=s.replace('GrandTour ETH-1 run complete','GrandTour SPX-3 run complete')
 if n.startswith('verify_effective'):
  s=once(s,"['CBSMS_BAG_DURATION_SEC'] == '120.0'","['CBSMS_BAG_DURATION_SEC'] == '60.0'")
  s=s.replace("enabled = os.environ", "assert os.environ['CBSMS_MISSION']=='2024-11-02-17-43-10'\nassert os.environ['CBSMS_BAG_START_SEC']=='32.0'\nenabled = os.environ")
 if n.startswith('run_case'):s=once(s,'duration=120','duration=60')
 if n=='validate_candidate.py':s=once(s,'len(a)>=570','len(a)>=270')
 if n=='validate_case.py':
  s=once(s,"source_cov['requested_s']==120","source_cov['requested_s']==60")
  s=once(s,"['rows']>=1170 and source_cov['native_kimera']['rows']>=570","['rows']>=570 and source_cov['native_kimera']['rows']>=270")
  s=once(s,'*1e-9>=118.','*1e-9>=58.')
 if n=='compare_belief_cov_fl4f.py':s=once(s,'len(off)>=1700','len(off)>=800')
 if n=='evaluate_fl4f.py':
  s=once(s,'GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag','GrandTour/SPX-3/2024-11-02-17-43-10_cpt7_ie_tc.bag')
  s=once(s,'len(times)>1100','len(times)>500')
 if n.endswith('.json'):
  cfg=json.loads(s);cfg['sequence']='SPX-3 bag32–92s FL4w matched live60 '+('OFF' if 'off' in n else 'ON')
  s=json.dumps(cfg,indent=2)+'\n'
 target=R/n.replace('live120','live60');target.write_text(s);shutil.copymode(T/n,target)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=n+' FL4t',tofile=target.name+' FL4w'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):
 old=p.read_text();new=common(old)
 if old!=new:p.write_text(new)
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
exe=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')
assert sha(exe)=='53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9'
assert sha(R/'private_build/libkimera_vio.so')=='6ce24c266b0f0e2561f0c05de2a661a72d5a4d9335df7c3cdfa0a4c77af9b4ae'
assert sha(R/'receiver_fl4e.sh')==sha(T/'receiver_fl4e.sh')
for p in (R/'private_build').rglob('*'):
 if p.is_file():assert sha(p)==sha(T/'private_build'/p.relative_to(R/'private_build'))
for mode in ('off','on'):
 old=json.loads((T/f'live120_{mode}.json').read_text());new=json.loads((R/f'live60_{mode}.json').read_text())
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
 p=W/('src/datasets/GrandTour/SPX-3/2024-11-02-17-43-10_'+kind+'.bag')
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
 profiles_verified_identical_to_eth1=True,source_profile_origin=str(S),
 binaries_identical=True,receiver_model_identical=True,protected_files=len(manifest),
 initial_container_state='both stopped; restored for authorized capture',free_bytes=shutil.disk_usage(W).free)
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'preflight.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps({k:v for k,v in result.items() if k not in ('bags','profile_sha256')},indent=2))
