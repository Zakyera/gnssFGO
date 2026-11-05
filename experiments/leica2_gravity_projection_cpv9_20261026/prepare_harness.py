"""Derive ONE candidate capture harness without copying old capture outputs."""
import ast,difflib,hashlib,json,shutil,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
B=W/'runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared'
BH=H.parent/'velodyne_full_campaign_cpv4_20261024/leica2'
R=W/'runs/leica2_gravity_projection_cpv9_20261026'
P=W/'runs/gnssfgo_leica2_gravity_projection_cpv9_20261026/prepared'
V=Path('/home/yeranis/results/gnssfgo_leica2_gravity_projection_cpv9_20261026')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
def once(s,a,b):assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
assert not P.exists() and not V.exists()
assert json.loads((R/'control_result.json').read_text())['passed']
assert json.loads((R/'candidate_attempt3_result.json').read_text())['passed']
assert sha(R/'libcbs.so')==json.loads((R/'candidate_attempt3_result.json').read_text())['sha256']
assert sha(R/'libcbs.so')==json.loads((R/'test_final_result.json').read_text())['candidate_sha256']
assert json.loads((R/'test_final_result.json').read_text())['passed']
pins=json.loads((R/'protected_before.json').read_text())
for p,h in pins.items():assert sha(p)==h,p
assert json.loads((BH/'closeout.json').read_text())['passed']
assert shutil.disk_usage(W).free>10*1024**3
with socket.socket() as s:s.bind(('127.0.0.1',11487))
P.mkdir(parents=True)
diff=[]
def common(s):return s.replace(B.parent.name,P.parent.name).replace(str(BH),str(H)).replace('11483','11487').replace('LEICA2-CPV4','LEICA2-CPV9')
def save(src,dest,s):
 with dest.open('x') as f:f.write(s)
 shutil.copymode(src,dest)
 diff.extend(difflib.unified_diff(src.read_text().splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(dest)))
for folder in ('input_profiles','transport_tools','private_build'):
 shutil.copytree(B/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__','libcbs.so') if folder=='private_build' else shutil.ignore_patterns('__pycache__'))
shutil.copy2(R/'libcbs.so',P/'private_build/libcbs.so')
names=('coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py','eof_boundary.py',
 'full_off.json','full_on.json','active_window_on.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch',
 'kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py','preflight_on.py',
 'receiver_cpk19.sh','run_case_on.py','source.env','source_runner_on.sh',
 'validate_candidate.py','validate_case.py','validate_local.py','verify_effective_on.py',
 'validate_velodyne.py','verify_velodyne_startup.py')
for name in names:
 s=common((B/name).read_text())
 if name=='kimera_vio_ros.launch':
  s=once(s,'<env name="CPK9_PRIOR_REFRESH" value="1"/>','<env name="CPK9_PRIOR_REFRESH" value="1"/>\n    <env name="CPV9_G2K_GRAVITY_PROJECTION" value="1"/>')
 if name=='verify_effective_on.py':
  s=once(s,"assert env(pid)['FL4Q_ENABLE']=='0'", "assert env(pid)['CPV9_G2K_GRAVITY_PROJECTION']=='1'\nassert 'CPV9_G2K_GRAVITY_PROJECTION' not in env(glim_pid)\nassert env(pid)['FL4Q_ENABLE']=='0'")
  s=once(s,'import hashlib\n','import hashlib\nassert hashlib.sha256((root/\'private_build/libcbs.so\').read_bytes()).hexdigest()=='+repr(sha(R/'libcbs.so'))+'\n')
 if name=='preflight_on.py':
  s=once(s,"for p in [P/'full_off_source", "B=Path("+repr(str(B))+")\nfor p in [B/'full_off_source")
  s=s.replace("P/'full_off_source", "B/'full_off_source").replace("P/'full_off_control", "B/'full_off_control")
  s=s.replace(str(V/'off'),str(Path('/home/yeranis/results')/B.parent.name/'off'))
  s=s.replace('same30.0s OFF passes all gates before matched ON','saved CP-V4 full OFF passes all gates; CP-V9 changes only G2K gravity projection')
 if name=='evaluate_fl4f.py':
  s=once(s,"ON_ROOT = ROOT\ndef source_root(mode):\n    return ON_ROOT if mode == 'on' else ROOT", "BASE = Path("+repr(cp(B))+ ")\ndef source_root(mode):\n    return ROOT if mode == 'on' else BASE\ndef case_mode(mode):\n    return 'on' if mode == 'baseline_on' else mode")
  s=s.replace("('off','on')","('off','baseline_on','on')").replace('full_{mode}_','full_{case_mode(mode)}_')
 save(B/name,P/name,s)
for src in (B/'transport_tools').rglob('*.py'):
 dest=P/'transport_tools'/src.relative_to(B/'transport_tools')
 s=common(src.read_text())
 if s!=src.read_text():dest.write_text(s)
assert (P/'receiver_cpk19.sh').stat().st_mode & 0o111
shutil.copy2(B/'input_audit.json',P/'input_audit.json')
shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
s=common((BH/'run_pair.py').read_text())
s=once(s,"for mode in ('off','on'):","for mode in ('on',):")
s=once(s,"def gate_paths(mode):\n", "def gate_paths(mode):\n    if mode=='off':\n        B=Path("+repr(str(B))+ ")\n        return [B/'full_off_source/local_provenance.json', B/'full_off_source/velodyne_validation.json', B/'full_off_source/candidate_validation.json', Path("+repr('/home/yeranis/results/'+B.parent.name+'/off/validation.json')+")]\n")
s=once(s,"            assert all(json.loads(path.read_text())['passed'] for path in gate_paths(mode))", "            step('validate_on_projection', [PY, str(H/'validate_projection.py')])\n            assert all(json.loads(path.read_text())['passed'] for path in gate_paths(mode))")
s=s.replace('Both modes and four gates each passed.','Candidate ON and all gates passed; saved CP-V4 OFF/original ON controls used.')
s=once(s,"        assert json.loads((H/'results.json').read_text())['passed']", "        assert json.loads((H/'results.json').read_text())['passed']\n        step('decision', [PY, str(H/'decision.py')])")
save(BH/'run_pair.py',H/'run_candidate.py',s)
# Keep the proven telemetry reader, reading old OFF and new ON, with the 3-way
# evaluator below exposing original ON as well. No historical file is rewritten.
s=common((BH/'summarize.py').read_text())
s=once(s,"for mode in ('off','on'):\n S=P/", "BASE=Path("+repr(str(B))+")\nBASE_V=Path("+repr('/home/yeranis/results/'+B.parent.name)+")\nfor mode in ('off','on'):\n Q=P if mode=='on' else BASE\n QV=V if mode=='on' else BASE_V\n S=Q/")
s=s.replace(";R=P/f'full_", ";R=Q/f'full_").replace(";C=P/f'full_", ";C=Q/f'full_").replace("receiver=V/mode/", "receiver=QV/mode/")
s=s.replace("d=P/f'full_{mode}_source'/folder", "d=(P if mode=='on' else BASE)/f'full_{mode}_source'/folder")
s=s.replace(" for mode in ('off','on'):\n  r=evaluation", " for mode in ('off','baseline_on','on'):\n  r=evaluation")
s=s.replace('CP-V4 matched LEICA-2 batched finalized-policy','CP-V9 LEICA-2 gravity-projection candidate with saved controls')
s=s.replace('One capture per mode,0.20x,headless. No paired observer,optimizer rebuild or tuning.', 'One new ON capture at0.20x; OFF/baseline_on are saved CP-V4. Only receiving G2K factor is projected to 4D.')
s=s.replace('SOURCE COVARIANCE SUMMARY: unchanged dense6x6 scales1; rotation then translation.', 'SOURCE EXPORT COVARIANCE SUMMARY: no direct rescale; optimizer states may change. G2K received factor gravity-separated 4D marginal projection; see projection_validation.json.')
s=s.replace("result=dict(passed=True,", "result=dict(passed=True,projection_validation=load(P/'full_on_source/projection_validation.json'),")
save(BH/'summarize.py',H/'summarize.py',s)
for p in [*P.rglob('*'),*H.glob('*.py')]:
 if not p.is_file():continue
 if p.suffix=='.py':ast.parse(p.read_text())
 elif p.suffix=='.launch':ET.parse(p)
 elif p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for mode in ('off','on'):
 for name in ('full_'+mode+'.json',):
  a=json.loads((B/name).read_text());b=json.loads((P/name).read_text())
  for c in (a,b):
   for k in ('ros_master_uri','receiver_binary','sequence'):c.pop(k)
   for source in c['sources']:source.pop('session')
  assert a==b
for name in ('validate_local.py','validate_candidate.py','validate_velodyne.py','coverage_checks.py','validate_case.py'):
 assert (P/name).read_text()==common((B/name).read_text()),name
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for p in [*P.rglob('*'),*R.rglob('*'),*H.glob('*')]:
 if p.is_file() and p.name not in ('REPORT.txt','protected_before.json') and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
for mode in ('off','on'):
 for p in [B/f'full_{mode}_control/complete.json',B/f'full_{mode}_source/odom_imu.txt',B/f'full_{mode}_source/kimera_odometry.csv',B/f'full_{mode}_receiver/receiver/online_trajectory.tum',B/f'full_{mode}_receiver/receiver/finalized_trajectory.tum']:
  pins[str(p)]=sha(p)
with (P/'protected_manifest.json').open('x') as f:json.dump(pins,f,indent=2)
(P/'preflight').mkdir()
result=dict(passed=True,prepared=str(P),validation_root=str(V),protected_files=len(pins),control_binary_identical=True,candidate_changes='only private G2K gravity-separated 4D marginal projection',saved_controls=str(B),disk_free_bytes=shutil.disk_usage(W).free)
for dest in (P/'preflight/validation.json',H/'prepared.json'):
 with dest.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
