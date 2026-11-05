"""Exclusive mechanical copy of CP-K27 harness with reviewed sensor/window delta."""
from pathlib import Path
import ast,difflib,hashlib,importlib.util,json,re,shutil,socket,subprocess,sys
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
OLD=H.parent/'spx3_stationary_preroll_cpk27_20261023'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_spx3_stationary_preroll_cpk27_20261023/prepared'
P=W/'runs/gnssfgo_spx3_velodyne_smoke_cpv2_20261023/prepared'
V=Path('/home/yeranis/results/gnssfgo_spx3_velodyne_smoke_cpv2_20261023')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def config(s):return json.loads(re.sub(r'/\*.*?\*/|//[^\n]*','',s,flags=re.S))
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
def rebase(s):
 for a,b in [(S.parent.name,P.parent.name),(OLD.name,H.name),
   ('spx3_cpk27_preroll_off_20261021','spx3_cpv2_velodyne_off_20261023'),
   ('spx3_cpk27_preroll_on_20261021','spx3_cpv2_velodyne_on_20261023'),
   ('SPX3-CPK27','SPX3-CPV2'),('CP-K27','CP-V2'),('11477','11480'),
   ('bag29-57s','Velodyne bag29-59s'),('glim_hesai','glim_velodyne'),
   ('/boxi/hesai/points','/anymal/velodyne/points'),('${mission}_hesai.bag','${mission}_anymal_velodyne.bag'),
   ('glim_sensor_pair=hesai+cpt7_imu','glim_sensor_pair=velodyne+cpt7_imu')]:s=s.replace(a,b)
 # Exact bounded-window guards, no optimizer parameter changes.
 for a,b in [('28.0','30.0'),('26.0','28.0'),('260','280'),('277','297'),('280','300')]:
  # Protect introduced replacements from matching later substitutions.
  s=re.sub(r'(?<![\w.])'+re.escape(a)+r'(?![\w.])','__CPV2_'+b+'__',s)
 s=re.sub(r'__CPV2_([0-9.]+)__',r'\1',s)
 return s
diff=[]
def generate(src,dest,s):
 original=src.read_text()
 if dest.exists():
  assert dest.read_text()==s,('partial preparation differs; refuse overwrite',dest)
 else:
  with dest.open('x') as f:f.write(s)
  shutil.copymode(src,dest)
 diff.extend(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(dest)))
assert not V.exists() and not (H/'status.json').exists()
assert not (P/'protected_manifest.json').exists() and not any(P.glob('full_*_source'))
assert json.loads((OLD/'closeout.json').read_text())['passed']
pre=json.loads((H/'bag_preflight.json').read_text());assert pre['passed'] and pre['stereo_pairs']==300
assert shutil.disk_usage(W).free>9*1024**3,'require1GiB pair budget plus8GiB launch headroom'
with socket.socket() as s:s.bind(('127.0.0.1',11480))
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p
P.mkdir(parents=True,exist_ok=True)
for folder in ('private_build','transport_tools'):
 if not (P/folder).exists():shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
(P/'input_profiles').mkdir(exist_ok=True)
profile_delta={}
for folder in ('glim_config','kimera_profile'):
 (P/'input_profiles'/folder).mkdir(exist_ok=True)
 for src in (S/'input_profiles'/folder).rglob('*'):
  dest=P/'input_profiles'/folder/src.relative_to(S/'input_profiles'/folder)
  if src.is_dir():
   dest.mkdir(exist_ok=True)
   continue
  assert src.is_file()
  text=src.read_text();out=text
  if folder=='glim_config' and src.name=='config_ros.json':
   out=once(out,'"lidar_frame_id": "hesai_lidar"','"lidar_frame_id": "velodyne_lidar"')
   out=once(out,'"points_topic": "/boxi/hesai/points"','"points_topic": "/anymal/velodyne/points"')
   a=config(text);b=config(out)
   assert {k for k in a['glim_ros'] if a['glim_ros'][k]!=b['glim_ros'][k]}=={'lidar_frame_id','points_topic'}
  if folder=='glim_config' and src.name=='config_sensors.json':
   out,n=re.subn(r'"T_lidar_imu":\s*\[[^\]]+\]', '"T_lidar_imu": '+json.dumps(pre['T_lidar_imu']),out)
   assert n==1
   out=once(out,'// Mission TF gives T_box_base_hesai_lidar. cpt7_imu is coincident with\n    // box_base, so this is its inverse: T_hesai_lidar_cpt7_imu.',
      '// CP-V2 verified mission TF: T_velodyne_lidar_cpt7_imu; maps IMU into LiDAR.')
   a=config(text);b=config(out)
   assert {k for k in a['sensors'] if a['sensors'][k]!=b['sensors'][k]}=={'T_lidar_imu'}
  generate(src,dest,out)
  if out!=text:profile_delta[str(dest.relative_to(P))]=dict(before_sha256=sha(src),after_sha256=sha(dest))
assert len(profile_delta)==2
files=['coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py','eof_boundary.py',
 'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
 'preflight_on.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env',
 'source_runner.sh','source_runner_on.sh','validate_candidate.py','validate_case.py',
 'validate_local.py','verify_effective.py','verify_effective_on.py']
inside='/workspace/'+str(P.relative_to('/home/yeranis/repos/V4RL'))
for name in files:
 s=rebase((S/name).read_text())
 # Remove unsafe historical fallback; explicit runner environment stays identical.
 s=s.replace('${project_root}/clean_glim_ws/src/glim/config_grandtour_eth1_gpu_hesai_cpt7',inside+'/input_profiles/glim_config')
 s=s.replace('/workspace/cbs_gtsam4.3/clean_glim_ws/src/glim/config_grandtour_eth1_gpu_hesai_cpt7',inside+'/input_profiles/glim_config')
 if name in ('source_runner.sh','source_runner_on.sh'):
  marker='python3 "${experiment_dir}/verify_effective'+('_on' if '_on' in name else '')+'.py" "${run_dir}"'
  s=once(s,marker,marker+'\npython3 "${experiment_dir}/verify_velodyne_startup.py" "${run_dir}"')
 if name=='preflight_on.py':
  s=once(s,"P/'full_off_source/local_provenance.json',","P/'full_off_source/local_provenance.json',P/'full_off_source/velodyne_validation.json',")
 generate(S/name,P/name,s)
for path in (P/'transport_tools').rglob('*.py'):
 old=path.read_text();new=rebase(old)
 if old!=new:
  path.write_text(new)
  diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/path.relative_to(P)),tofile=str(path)))
for name in ('PROTOCOL.txt','verify_velodyne_startup.py','validate_velodyne.py'):
 shutil.copy2(H/name,P/name)
shutil.copy2(H/'bag_preflight.json',P/'input_audit.json')
for name in ('run_pair.py','summarize.py'):
 s=rebase((OLD/name).read_text())
 if name=='run_pair.py':
  s=once(s,"P/f'full_{mode}_source/local_provenance.json',","P/f'full_{mode}_source/local_provenance.json',P/f'full_{mode}_source/velodyne_validation.json',")
  s=once(s,"[('local',['validate_local.py',mode]),","[('sensor',['validate_velodyne.py',mode]),('local',['validate_local.py',mode]),")
  s=s.replace('three gates','four gates')
 else:
  s=once(s,"('last20s',8.0)","('last20s',10.0)")
  s=s.replace('One28s startup pre-roll pair','One30s rear-Velodyne integration pair')
  s=s.replace('bounded28.0s','bounded30.0s')
 generate(OLD/name,H/name,s)
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for folder in ('private_build','input_profiles/kimera_profile'):
 for p in (S/folder).rglob('*'):
  if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
for name in ('expiry_audit.py','eof_boundary.py','map_guard.py'):assert sha(S/name)==sha(P/name),name
for mode in ('off','on'):
 old=json.loads((S/f'full_{mode}.json').read_text());new=json.loads((P/f'full_{mode}.json').read_text())
 for c in (old,new):
  for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
  for source in c['sources']:
   source.pop('session')
   if source['kind']=='glim':source['id']='GLIM_SENSOR'
 assert old==new,'Receiver numerical settings changed'
for p in [*P.rglob('*'),*H.glob('*.py')]:
 if not p.is_file():continue
 if p.suffix=='.py':ast.parse(p.read_text())
 elif p.suffix=='.launch':ET.parse(p)
 elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
for base in (P,H):
 for p in base.rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts and p.name!='REPORT.txt':manifest[str(p)]=sha(p)
with (P/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
(P/'preflight').mkdir()
result=dict(passed=True,template=str(S),prepared=str(P),protected_files=len(manifest),
 receiver_model_identical=True,kimera_profile_byte_identical=True,private_builds_identical=True,
 profile_delta=profile_delta,window=pre['window'],stereo_pairs=300,
 remaining_unchanged_guards='8GiB launch,5GiB runtime,stop on failure,no retries',
 disk_free_bytes=shutil.disk_usage(W).free)
with (P/'preflight/validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
