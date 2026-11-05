"""Isolated duration/mission-only copies of the validated CP-V2 harness."""
from pathlib import Path
import ast,difflib,hashlib,importlib.util,json,re,shutil,socket,subprocess,sys
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
OLD=H.parent/'velodyne_spx3_smoke_cpv2_20261023'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_spx3_velodyne_smoke_cpv2_20261023/prepared'

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def main(slug):
 cfg=next(c for c in json.loads((H/'campaign.json').read_text())['sequences'] if c['slug']==slug)
 out=H/slug;pre=json.loads((out/'bag_preflight.json').read_text());win=pre['window']
 assert pre['passed'] and pre['calibration_matches_frozen_profiles']
 label=f'gnssfgo_{slug}_velodyne_full_cpv4_20261024'
 P=W/'runs'/label/'prepared';V=Path('/home/yeranis/results')/label
 assert not V.exists() and not (out/'status.json').exists()
 assert not (P/'protected_manifest.json').exists() and not any(P.glob('full_*_source'))
 assert json.loads((OLD/'closeout.json').read_text())['passed']
 assert shutil.disk_usage(W).free>10*1024**3,'8GiB gate plus minimum preparation margin'
 with socket.socket() as s:s.bind(('127.0.0.1',cfg['port']))
 manifest=json.loads((S/'protected_manifest.json').read_text())
 for p,digest in manifest.items():assert sha(p)==digest,p
 for b in pre['bag_files'].values():
  st=Path(b['path']).stat();assert (st.st_size,st.st_mtime_ns)==(b['bytes'],b['mtime_ns']),b['path']
 duration=str(win['duration_s']);span=str(round(win['duration_s']-2,9));offset=str(win['bag_offset_s'])
 start=f"{win['start_ns']//10**9}.{win['start_ns']%10**9:09d}"
 who=slug.upper()+'-CPV4';diff=[]
 def common(text):
  for a,b in [(str(OLD),str(out)),(S.parent.name,label),
   ('spx3_cpv2_velodyne',slug+'_cpv4_velodyne'),('20261023','20261024')]:
   # Date replacement is confined to generated run labels below, never binary paths.
   if a=='20261023':continue
   text=text.replace(a,b)
  text=text.replace(slug+'_cpv4_velodyne_off_20261023',slug+'_cpv4_velodyne_off_20261024')
  text=text.replace(slug+'_cpv4_velodyne_on_20261023',slug+'_cpv4_velodyne_on_20261024')
  for a,b in [('SPX3-CPV2',who),('CP-V2','CP-V4'),('SPX-3',cfg['dataset']),
   ('2024-11-02-17-43-10',cfg['mission']),('11480',str(cfg['port'])),
   ('1730565899.872888651',start),('1730565898',str(win['seed_clock_seconds'])),
   ('Velodyne bag29-59s',f"Velodyne physical {win['start_ns']}..{win['end_ns']}ns")]:text=text.replace(a,b)
  for a,b in [('CBSMS_BAG_START_SEC=29.0','CBSMS_BAG_START_SEC='+offset),
   ('CBSMS_BAG_START_SEC:-29.0','CBSMS_BAG_START_SEC:-'+offset),
   ('"${bag_start_sec}" == 29.0','"${bag_start_sec}" == '+offset),
   ("['CBSMS_BAG_START_SEC']=='29.0'","['CBSMS_BAG_START_SEC']=='"+offset+"'")]:text=text.replace(a,b)
  return text
 def bounds(text):
  text=text.replace('bounded30.0', 'full-usable'+duration)
  for a,b in [('30.0',duration),('28.0',span),('297',str(pre['frontend_min'])),
              ('300',str(pre['frontend_max'])),('280',str(pre['glim_min']))]:
   text=re.sub(r'(?<![\w.])'+re.escape(a)+r'(?![\w.])','__V4_'+b+'__',text)
  text=re.sub(r'__V4_([0-9.]+)__',r'\1',text)
  text=text.replace('bounded'+duration,'full-usable'+duration)
  return text
 def generate(src,dest,text):
  old=src.read_text()
  if dest.exists():assert dest.read_text()==text,('partial preparation differs',dest)
  else:
   with dest.open('x') as f:f.write(text)
   shutil.copymode(src,dest)
  diff.extend(difflib.unified_diff(old.splitlines(True),text.splitlines(True),fromfile=str(src),tofile=str(dest)))
 P.mkdir(parents=True,exist_ok=True)
 for folder in ('private_build','input_profiles','transport_tools'):
  if not (P/folder).exists():shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
 files=['coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py','eof_boundary.py',
  'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
  'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
  'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
  'preflight_on.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env',
  'source_runner.sh','source_runner_on.sh','validate_candidate.py','validate_case.py',
  'validate_local.py','verify_effective.py','verify_effective_on.py',
  'validate_velodyne.py','verify_velodyne_startup.py']
 bounded={'coverage_checks.py','preflight_on.py','run_case.py','run_case_on.py','source.env',
  'source_runner.sh','source_runner_on.sh','validate_case.py','validate_local.py','verify_effective.py','verify_effective_on.py'}
 for name in files:
  text=common((S/name).read_text())
  if name in bounded:text=bounds(text)
  generate(S/name,P/name,text)
 for src in (S/'transport_tools').rglob('*.py'):
  dest=P/'transport_tools'/src.relative_to(S/'transport_tools')
  text=common(src.read_text())
  if text!=src.read_text():
   assert dest.read_text()==src.read_text(),dest
   dest.write_text(text)
   diff.extend(difflib.unified_diff(src.read_text().splitlines(True),text.splitlines(True),fromfile=str(src),tofile=str(dest)))
 for name in ('run_pair.py','summarize.py'):
  text=common((OLD/name).read_text())
  if name=='summarize.py':
   text=text.replace("('last20s',10.0)","('last20s',"+str(round(win['duration_s']-20,9))+")")
   text=text.replace('One30s rear-Velodyne integration pair','Full usable rear-Velodyne matched pair')
   text=bounds(text)
  generate(OLD/name,out/name,text)
 for dest in (out/'PROTOCOL.txt',P/'PROTOCOL.txt'):
  text=(H/'PROTOCOL.txt').read_text()+'\nTHIS PAIR: '+json.dumps(cfg)+'\nADMITTED WINDOW: '+json.dumps(win)+'\n'
  with dest.open('x') as f:f.write(text)
 shutil.copy2(out/'bag_preflight.json',P/'input_audit.json')
 with (out/'REPORT.txt').open('x') as f:
  f.write('CP-V4 '+cfg['dataset']+' full usable Velodyne pair — PREPARED, NOT LAUNCHED.\n'+json.dumps(win,indent=2)+'\n')
 with (out/'harness.diff').open('x') as f:f.write(''.join(diff))
 for folder in ('private_build','input_profiles'):
  for p in (S/folder).rglob('*'):
   if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
 for name in ('expiry_audit.py','eof_boundary.py','map_guard.py','validate_velodyne.py','verify_velodyne_startup.py'):
  assert sha(S/name)==sha(P/name),name
 for mode in ('off','on'):
  a=json.loads((S/f'full_{mode}.json').read_text());b=json.loads((P/f'full_{mode}.json').read_text())
  for c in (a,b):
   for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
   for source in c['sources']:source.pop('session')
  assert a==b,'Receiver model changed'
 for p in [*P.rglob('*'),*out.glob('*.py')]:
  if not p.is_file():continue
  if p.suffix=='.py':ast.parse(p.read_text())
  elif p.suffix=='.launch':ET.parse(p)
  elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
 spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 assert m.test_boundaries()=={'valid':1,'rejected':4}
 for b in pre['bag_files'].values():manifest[b['path']]=b['sha256']
 for base in (P,out):
  for p in base.rglob('*'):
   if p.is_file() and '__pycache__' not in p.parts and p.name!='REPORT.txt':manifest[str(p)]=sha(p)
 with (P/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
 (P/'preflight').mkdir()
 result=dict(passed=True,template=str(S),prepared=str(P),validation_root=str(V),
   protected_files=len(manifest),window=win,stereo_pairs=pre['stereo_pairs'],
   private_builds_and_source_profiles_byte_identical=True,receiver_model_identical=True,
   independently_validated_mission_calibration=True,guard_policy='unchanged; only input-count/window identities updated',
   disk_free_bytes=shutil.disk_usage(W).free)
 with (P/'preflight/validation.json').open('x') as f:json.dump(result,f,indent=2)
 with (out/'prepared.json').open('x') as f:json.dump(result,f,indent=2)
 print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':main(sys.argv[1])
