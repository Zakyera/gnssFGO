"""Create one fresh OFF harness from frozen CP-K23; no capture here."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,socket,subprocess,xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared'
P=W/'runs/gnssfgo_eth1_incremental_cpv29_20261028/prepared'
B=W/'runs/glim_incremental_publisher_cpv29_20261028'
V=Path('/home/yeranis/results/gnssfgo_eth1_incremental_cpv29_20261028')
OLD='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019'
NEW='/workspace/cbs_gtsam4.3/runs/'+B.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def common(s):return s.replace(S.parent.name,P.parent.name).replace('eth1_cpk23_full','eth1_cpv29_full').replace('11473','11529').replace('CP-K23','CP-V29')
def replace(s,a,b):
 assert a in s,a
 return s.replace(a,b)
assert not P.parent.exists() and not V.exists()
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as sock:sock.bind(('127.0.0.1',11529))
build=json.loads((B/'build_passed.json').read_text());assert build['passed']
assert sha(B/'libglim_ros.so')==build['binary_sha256']
for mode in ('release','sanitized'):
 t=json.loads((B/(mode+'_test.log')).read_text());assert t['passed'] and t['fixture_rows']==3454 and t['max_matrix_error']<1e-8
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p
pre=json.loads((S/'preflight/validation.json').read_text());assert pre['passed']
for item in pre['bag_files'].values():
 st=Path(item['path']).stat();assert st.st_size==item['bytes'] and st.st_mtime_ns==item['mtime_ns'],item['path']
P.mkdir(parents=True)
for folder in ('private_build','input_profiles','transport_tools'):
 shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
names=('active_window.launch','glim_profile.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','source.env','source_runner.sh',
 'receiver_cpk19.sh','full_off.json','experiment.py','run_case.py','coverage_checks.py','expiry_audit.py',
 'map_guard.py','verify_effective.py','validate_local.py','validate_candidate.py','validate_case.py','evaluate_fl4f.py')
diff=[]
for name in names:
 old=(S/name).read_text();new=common(old)
 if name in ('map_guard.py','verify_effective.py','glim_profile.launch'):new=replace(new,OLD,NEW)
 if name=='glim_profile.launch':
  new=replace(new,'<env name="GLIM_SHARED_NATIVE_PROVIDER"',
   '<env name="GLIM_INCREMENTAL_OUTPUT_CPV29" value="1"/>\n'
   '        <env name="GLIM_INCREMENTAL_CSV_CPV29" value="$(env CBSMS_RUN_ROOT)/$(env CBSMS_RUN_NAME)/glim_incremental_pairs.csv"/>\n'
   '        <env name="GLIM_SHARED_NATIVE_PROVIDER"')
 if name=='verify_effective.py':
  new=replace(new,'42df73c172232a612da965bf43ead8ec454caf08434b84e2da5de74d26cdb79f',sha(B/'libglim_ros.so'))
  new=replace(new,"assert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)",
   "assert env(glim_pid)['GLIM_INCREMENTAL_OUTPUT_CPV29']=='1'\n"
   "assert Path(env(glim_pid)['GLIM_INCREMENTAL_CSV_CPV29'])==source/'glim_incremental_pairs.csv'\n"
   "assert 'GLIM_INCREMENTAL_OUTPUT_CPV29' not in env(pid)\n"
   "assert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)")
 if name=='source_runner.sh':
  anchor='rostopic echo -p /kimera_vio_ros/odometry'
  new=replace(new,anchor,'rostopic echo -p /glim/incremental_pose_cpv29 >"${run_dir}/glim_incremental_ros.csv" &\n'
    'recorders+=("$!"); pids+=("$!")\n'+anchor)
  new=replace(new,'/glim/cbs/odometry /kimera_vio_ros/odometry \\\n',
                  '/glim/cbs/odometry /kimera_vio_ros/odometry /glim/incremental_pose_cpv29 \\\n')
 if name=='run_case.py':new=replace(new,"['SHARED_NATIVE_SHADOW_FAILURE'","['CPV29_INCREMENTAL_FAILURE','SHARED_NATIVE_SHADOW_FAILURE'")
 if name=='evaluate_fl4f.py':
  # Library functions only: retain original file, do NOT execute its old main
  # (which scores retrospective GLIM dumps). The new scorer uses native CSVs.
  pass
 with (P/name).open('x') as f:f.write(new)
 shutil.copymode(S/name,P/name)
 if new!=old:diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for p in (P/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
with (H/'harness.diff').open('x') as f:f.writelines(diff)
for folder in ('private_build','input_profiles'):
 for p in (S/folder).rglob('*'):
  if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
for name in ('coverage_checks.py','expiry_audit.py'):assert sha(S/name)==sha(P/name)
old=json.loads((S/'full_off.json').read_text());new=json.loads((P/'full_off.json').read_text())
for d in (old,new):
 for k in ('sequence','ros_master_uri','receiver_binary'):d.pop(k)
 for source in d['sources']:source.pop('session')
assert old==new,'receiver model changed'
for p in P.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 elif p.suffix=='.launch':ET.parse(p)
 elif p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for root in (P,B):
 for p in root.rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts:manifest[str(p)]=sha(p)
with (P/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
pre.update(template=str(S),protected_files=len(manifest),candidate_glim_ros_sha256=sha(B/'libglim_ros.so'),
 output_only_incremental=True,unchanged_model_profiles_coverage=True,bag_identity_metadata_unchanged=True)
(P/'preflight').mkdir()
with (P/'preflight/validation.json').open('x') as f:json.dump(pre,f,indent=2)
with (H/'prepared.json').open('x') as f:json.dump(dict(passed=True,prepared=str(P),validation=str(V),
 protected_files=len(manifest),new_library_sha256=sha(B/'libglim_ros.so')),f,indent=2)
print('Prepared ONE full CBS-OFF capture:',P,flush=True)
