"""Prepare one fresh authorized ON, rebasing only identities in frozen CP-V17."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,socket,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
OLD=H.parent/'arc3_velodyne_qcquarter_cpv17_20261026'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLDLABEL='gnssfgo_arc3_velodyne_qcquarter_cpv17_20261026'
LABEL='gnssfgo_arc3_velodyne_on_cpv18_20261027'
S=W/'runs'/OLDLABEL/'prepared';P=W/'runs'/LABEL/'prepared'
V=Path('/home/yeranis/results')/LABEL
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2)
def rebase(t):
 for a,b in [(OLDLABEL,LABEL),(str(OLD/'arc3'),str(H)),('11488','11489'),
             ('arc3_cpv17_velodyne_on_20261026','arc3_cpv18_velodyne_on_20261027'),
             ('arc3_cpv17_velodyne_off_20261026','arc3_cpv18_velodyne_off_20261027')]:t=t.replace(a,b)
 return t
def main():
 assert not P.exists() and not V.exists() and not (H/'prepared.json').exists()
 assert shutil.disk_usage(W).free>16*1024**3,'One capture forecast plus reserve'
 assert json.loads((OLD/'arc3/status.json').read_text())['state']=='failed'
 assert json.loads((OLD/'arc3/closeout.json').read_text())['passed']
 assert (S/'full_off_control/failed.txt').exists() and not (S/'full_off_control/complete.json').exists()
 assert not any((S/f'full_on_{part}').exists() for part in ('source','receiver','control'))
 manifest=json.loads((S/'protected_manifest.json').read_text())
 for p,want in manifest.items():assert sha(p)==want,p
 with socket.socket() as sock:sock.bind(('127.0.0.1',11489))
 pre=json.loads((S/'input_audit.json').read_text())
 assert pre['passed'] and pre['window']['duration_s']==726.38
 P.mkdir(parents=True)
 diffs=[];copied=[]
 for folder in ('private_build','input_profiles','transport_tools'):
  shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
 # Do not install the old OFF prerequisite or the old two-mode evaluator.
 excluded={'PROTOCOL.txt','protected_manifest.json','preflight_on.py','evaluate_fl4f.py','run_case.py','controller.lock'}
 for src in S.iterdir():
  if src.is_file() and src.name not in excluded:
   old=src.read_text();new=old if src.name=='input_audit.json' else rebase(old);dest=P/src.name
   with dest.open('x') as f:f.write(new)
   shutil.copymode(src,dest);copied.append(src.name)
   diffs.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(src),tofile=str(dest)))
 for src in (S/'transport_tools').rglob('*.py'):
  dest=P/'transport_tools'/src.relative_to(S/'transport_tools')
  old=src.read_text();new=rebase(old)
  if new!=old:
   dest.write_text(new)
   diffs.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(src),tofile=str(dest)))
 shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
 for folder in ('private_build','input_profiles'):
  for src in (S/folder).rglob('*'):
   if src.is_file():assert sha(src)==sha(P/src.relative_to(S)),src
 for name in copied:
  old=(S/name).read_text()
  assert (P/name).read_text()==(old if name=='input_audit.json' else rebase(old)),name
 assert sha(P/'input_audit.json')==sha(S/'input_audit.json')
 cfg=[]
 for root,mode in ((S,'off'),(S,'on'),(P,'on')):
  c=json.loads((root/f'full_{mode}.json').read_text())
  for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
  for s in c['sources']:s.pop('session')
  cfg.append(c)
 assert cfg[0]==cfg[1]==cfg[2],'Unexpected receiver model change'
 for p in [*P.rglob('*'),*H.glob('*.py')]:
  if not p.is_file():continue
  if p.suffix=='.py':ast.parse(p.read_text())
  elif p.suffix=='.launch':ET.parse(p)
  elif p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
 evidence=[S/'protected_manifest.json',OLD/'arc3/status.json',OLD/'arc3/closeout.json',
  S/'full_off_control/failed.txt',S/'full_off_receiver/receiver.log',
  S/'full_off_source/glim_odometry.csv',S/'full_off_source/kimera_odometry.csv',
  S/'full_off_receiver/receiver/online_trajectory.tum',S/'full_off_receiver/events.csv']
 for p in evidence:manifest[str(p)]=sha(p)
 for base in (P,H):
  for p in base.rglob('*'):
   if p.is_file() and '__pycache__' not in p.parts and p.name!='REPORT.txt':manifest[str(p)]=sha(p)
 save(P/'protected_manifest.json',manifest)
 result=dict(passed=True,admission='Explicit standalone ON after preserved failed OFF; not matched-pair admission',
  prepared=str(P),validation_root=str(V),window=pre['window'],protected_files=len(manifest),
  source_profiles_binaries_identical=True,receiver_model_identical_to_OFF=True,qc_multiplier_original=.25,
  port=11489,disk_free_bytes=shutil.disk_usage(W).free,no_guard_changes=True,old_failure_preserved=True)
 (P/'preflight').mkdir();save(P/'preflight/validation.json',result)
 # The runner requires this receipt. It truthfully records the NEW authorization,
 # never claims OFF passed or invokes the old matched-pair admission script.
 save(P/'preflight_on.json',result);save(H/'prepared.json',result)
 with (H/'harness.diff').open('x') as f:f.write(''.join(diffs))
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
