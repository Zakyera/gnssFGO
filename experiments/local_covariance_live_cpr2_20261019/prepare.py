"""Fresh OFF-only30s harness. Reuse CP-R1; only provider and run identity change."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared'
R=W/'runs/gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
C=W/'runs/native_covariance_cache_cpi3_20261019/release_v2'
CS='/workspace/cbs_gtsam4.3/runs/native_covariance_cache_cpi3_20261019/release_v2'
BRIDGE='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
assert not R.parent.exists(),'refuse duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
manifest=json.loads((T/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
assert sha(C/'libgtsam_points.so.1.2.1')=='3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7'
gate=H.parent/'native_covariance_cache_cpi3_20261019'
assert json.loads((gate/'verification.json').read_text())['summary']['max_covariance_relative_error']==0
assert (C/'passed.json').exists() and (C.parent/'san/passed.json').exists()
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
shutil.copytree(T/'input_profiles',R/'input_profiles')
def common(s):
 return s.replace(T.parent.name,R.parent.name).replace('eig2_cpr1_live60_off_20261017','eig2_cpr2_live30_off_20261019').replace('live60','live30').replace('11447','11448')
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'map_guard.py','source.env','receiver_fl4e.sh','run_case.py','verify_effective.py',
 'source_runner.sh','active_window.launch','glim_profile.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live60_off.json','validate_local.py']
diff=[]
for name in files:
 old=(T/name).read_text();s=common(old)
 if name=='glim_profile.launch':s=once(s,'value="'+BRIDGE+':','value="'+CS+':'+BRIDGE+':')
 if name=='map_guard.py':s=once(s,'else library_dir','else (Path("'+CS+'") if name == "libgtsam_points.so.1.2.1" else library_dir)')
 if name=='source.env':s=once(s,'CBSMS_BAG_DURATION_SEC:-60.0','CBSMS_BAG_DURATION_SEC:-30.0')
 if name=='run_case.py':s=once(s,'duration=60','duration=30')
 if name=='source_runner.sh':
  s=once(s,'"${duration_sec}" == 60.0','"${duration_sec}" == 30.0').replace('60-second OFF','30-second OFF')
  s=once(s,'min_glim_rows=570','min_glim_rows=280');s=once(s,'min_kimera_rows=270','min_kimera_rows=135')
 if name=='verify_effective.py':
  s=once(s,"== '60.0'","== '30.0'")
  s=once(s,"assert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)","assert hashlib.sha256(Path('"+CS+"/libgtsam_points.so.1.2.1').read_bytes()).hexdigest()=='3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7'\nassert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)")
  s=once(s,"glim_revision_covariance='native_local'","glim_revision_covariance='native_local',cached_provider_sha256='3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7'")
 if name=='validate_case.py':
  s=once(s,"source_cov['requested_s']==60","source_cov['requested_s']==30")
  s=once(s,"['rows']>=570","['rows']>=280");s=once(s,"['rows']>=270","['rows']>=135")
  s=once(s,'*1e-9>=58.','*1e-9>=28.')
  s=once(s,"assert MODE in ('off','on')","assert MODE == 'off'")
 if name=='validate_candidate.py':s=once(s,'len(a)>=270','len(a)>=135')
 if name=='validate_local.py':
  s=once(s,"assert mode in ('off','on')","assert mode == 'off'")
  s=once(s,'first>=570','first>=280')
  s=once(s,'for raw in paths[0].open():',"payload_log=paths[0] if kind=='glim' else S/'estimators.log'\n if kind=='kimera':\n  for telemetry in paths[0].open():\n   if 'CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,k,' in telemetry:\n    fields=telemetry.split('CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,k,',1)[1].split(',')\n    assert fields[0]=='tmp_marginals',('MAIN fallback',telemetry);tmp+=1\n for raw in payload_log.open():")
  s=once(s,"  if kind=='kimera' and 'CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,k,' in line:\n   f=line.split('CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,k,',1)[1].split(',')\n   assert f[0]=='tmp_marginals',('MAIN fallback',line);tmp+=1\n",'')
 if name.endswith('.json'):
  obj=json.loads(s);obj['sequence']='EIG-2 LOCAL CP-R2 bag5.7-35.7s OFF';s=json.dumps(obj,indent=2)+'\n'
 dest=R/name.replace('live60','live30');dest.write_text(s);shutil.copymode(T/name,dest)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=name+' CP-R1',tofile=dest.name+' CP-R2'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
assert sha(R/'receiver_fl4e.sh')==sha(T/'receiver_fl4e.sh')
old=json.loads((T/'live60_off.json').read_text());new=json.loads((R/'live30_off.json').read_text())
for obj in (old,new):
 for k in ('sequence','ros_master_uri','receiver_binary'):obj.pop(k)
 for source in obj['sources']:source.pop('session')
assert old==new
for p in R.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 if p.suffix=='.launch':ET.parse(p)
 if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for path in [C/'libgtsam_points.so.1.2.1',C/'passed.json',C.parent/'san/passed.json',gate/'verification.json',T/'protected_manifest.json']:
 manifest[str(path)]=sha(path)
for p in R.rglob('*'):
 if p.is_file():manifest[str(p)]=sha(p)
for name in ('prepare.py','PROTOCOL.txt'):manifest[str(H/name)]=sha(H/name)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=json.loads((T/'preflight/validation.json').read_text())
result.update(passed=True,template=str(T),requested_s=30,bag_start_s=5.7,
 candidate_provider_sha256=sha(C/'libgtsam_points.so.1.2.1'),protected_files=len(manifest),
 free_bytes=shutil.disk_usage(W).free,change='CP-I3 cached provider; OFF-only30s gate',
 input_validation='Frozen EIG-2 bags/profiles hash-verified;30s is contained in validated60s; no accuracy scoring here')
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps(dict(prepared=str(R),protected=len(manifest),free_bytes=result['free_bytes']),indent=2))
