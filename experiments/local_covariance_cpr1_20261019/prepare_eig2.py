"""Reuse the complete validated FL4x harness; change only CP-I1 bridge routing."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_eig2_live60_off_on_fl4x_20261017/prepared'
R=W/'runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared'
C=W/'runs/local_revision_covariance_cpi1_20261019'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
assert not R.parent.exists(),'refuse duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
oldmanifest=json.loads((T/'protected_manifest.json').read_text())
for p,h in oldmanifest.items():assert sha(p)==h,p
gate=W/'runs/kimera_native_prior_cpi2_20261019'
for tag in ['release','san']:
 records=json.loads((gate/(tag+'_commands.json')).read_text())
 assert len(records)==4 and all(x['returncode']==0 for x in records)
 assert 'PASS retirements=13' in (gate/(tag+'_test.log')).read_text()
verify=json.loads((H.parent/'local_revision_covariance_cpi1_20261019/verification.json').read_text())
for group in ('protected_hashes','candidate_hashes'):
 for p,h in verify[group].items():assert sha(p)==h,p
R.mkdir(parents=True)
shutil.copytree(T/'private_build',R/'private_build',symlinks=True)
shutil.copytree(T/'input_profiles',R/'input_profiles')
def common(s):
 return s.replace(T.parent.name,R.parent.name).replace('gnssfgo_eig2_live60_fl4x_20261017','gnssfgo_eig2_local_cov60_cpr1_20261019').replace('eig2_fl4x_live60_','eig2_cpr1_live60_').replace('11446','11447')
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'evaluate_fl4f.py','compare_belief_cov_fl4f.py','preflight_on_fl4f.py','map_guard.py',
 'source.env','receiver_fl4e.sh','run_case.py','run_case_on.py','verify_effective.py',
 'verify_effective_on.py','source_runner.sh','source_runner_on.sh','active_window.launch',
 'active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live60_off.json','live60_on.json']
diff=[]
candidate='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019'
provider='/workspace/cbs_gtsam4.3/runs/shared_linearization_s4_provider_20261012/release_2'
for name in files:
 old=(T/name).read_text();s=common(old)
 if name.startswith('glim_profile'):
  s=once(s,'value="'+provider+':$(optenv LD_LIBRARY_PATH)"','value="'+candidate+':'+provider+':$(optenv LD_LIBRARY_PATH)"')
  s=once(s,'<env name="GLIM_SHARED_NATIVE_PROVIDER" value="shared_native"/>','<env name="GLIM_SHARED_NATIVE_PROVIDER" value="shared_native"/>\n        <env name="GLIM_REVISION_COVARIANCE_POLICY" value="native_local"/>')
 if name=='map_guard.py':
  s=once(s,'target = str((library_dir / name).resolve(strict=True))',
   'directory = Path("'+candidate+'") if name == "libglim_ros.so" else library_dir\n        target = str((directory / name).resolve(strict=True))')
  s=once(s,"str(Path(library_dir) / 'libglim_ros.so')","str(Path('"+candidate+"') / 'libglim_ros.so')")
 if name.startswith('verify_effective'):
  s=once(s,"assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'",
   "assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'\nassert env(glim_pid)['GLIM_REVISION_COVARIANCE_POLICY']=='native_local'\nassert 'GLIM_REVISION_COVARIANCE_POLICY' not in env(pid)\nimport hashlib\nassert hashlib.sha256(Path('"+candidate+"/libglim_ros.so').read_bytes()).hexdigest()=='42df73c172232a612da965bf43ead8ec454caf08434b84e2da5de74d26cdb79f'")
  s=once(s,'kimera_maps=loaded,glim_maps=glim_loaded)',"kimera_maps=loaded,glim_maps=glim_loaded,glim_revision_covariance='native_local')")
 if name=='preflight_on_fl4f.py':
  s=once(s,"assert (P/'live60_off_control/complete.json').is_file()", "assert (P/'live60_off_control/complete.json').is_file()\nassert json.loads((P/'live60_off_source/local_provenance.json').read_text())['passed']")
 if name.endswith('.json'):
  obj=json.loads(s);obj['sequence']='EIG-2 LOCAL CP-R1 bag5.7-65.7s '+('OFF' if 'off' in name else 'ON');s=json.dumps(obj,indent=2)+'\n'
 dest=R/name;dest.write_text(s);shutil.copymode(T/name,dest)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=name+' frozen',tofile=name+' CP-R1'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
for name in ('PROTOCOL.txt','validate_local.py'):shutil.copy2(H/name,R/name)
assert sha(R/'receiver_fl4e.sh')==sha(T/'receiver_fl4e.sh')
for mode in ('off','on'):
 old=json.loads((T/f'live60_{mode}.json').read_text());new=json.loads((R/f'live60_{mode}.json').read_text())
 for c in (old,new):
  for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
  for source in c['sources']:source.pop('session')
 assert old==new
for p in R.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 if p.suffix=='.launch':ET.parse(p)
 if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
manifest=dict(oldmanifest)
manifest.update(verify['protected_hashes']);manifest.update(verify['candidate_hashes'])
for p in R.rglob('*'):
 if p.is_file():manifest[str(p)]=sha(p)
for name in ('prepare_eig2.py','validate_local.py','PROTOCOL.txt'):manifest[str(H/name)]=sha(H/name)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=json.loads((T/'preflight/validation.json').read_text())
result.update(passed=True,template=str(T),prior_gate=str(gate),candidate_glim_sha256=sha(C/'libglim_ros.so'),
 protected_files=len(manifest),free_bytes=shutil.disk_usage(W).free,
 change='GLIM revisions query native_local; exact frozen Kimera and receiver',
 input_validation='Frozen FL4x bags/profiles verified byte-identical by full protected hashes')
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'eig2_harness.diff').write_text(''.join(diff))
print(json.dumps(dict(prepared=str(R),protected=len(manifest),free_bytes=result['free_bytes']),indent=2))
