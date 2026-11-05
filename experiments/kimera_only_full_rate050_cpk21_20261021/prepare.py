"""Snapshot existing standalone-compatible launch chain, not estimator code."""
import ast,hashlib,json,shutil,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eth1_batched_full_cpk18_20261021/prepared'
P=W/'runs/kimera_eth1_full_rate050_cpk21_20261021'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert not P.exists(),'refuse existing output root'
protected=json.loads((S/'protected_manifest.json').read_text())
for p,want in protected.items():assert sha(p)==want,p
assert shutil.disk_usage(W).free>10*1024**3
P.mkdir()
for folder,target in [('private_build','private_build'),('input_profiles/kimera_profile','kimera_profile')]:
 shutil.copytree(S/folder,P/target,symlinks=True)
for name in ('grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch'):
 shutil.copy2(S/name,P/name)
tree=ET.parse(S/'active_window.launch');root=tree.getroot();removed=[]
for child in list(root):
 if child.tag=='include' and ('glim_profile.launch' in child.get('file','') or 'newer_college_cbs_ekf.launch' in child.get('file','')):
  removed.append(child.attrib);root.remove(child)
 elif child.tag=='param' and child.get('name','').startswith('/glim_rosnode/'):
  root.remove(child)
assert len(removed)==2
tree.write(P/'kimera_only.launch',encoding='unicode')
for name in ('observe.py','run_inside.py','PROTOCOL.txt'):shutil.copy2(H/name,P/name)
shutil.copy2(S/'full_off_source/effective_mode_check.json',P/'reference_effective.json')
for p in P.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 elif p.suffix=='.launch':ET.parse(p)
assert sha(P/'private_build/libkimera_vio.so')=='be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9'
manifest={str(p.relative_to(P)):sha(p) for p in P.rglob('*') if p.is_file()}
(P/'prepared_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(H/'preparation.json').write_text(json.dumps(dict(passed=True,root=str(P),protected=len(protected),
 unchanged_kimera_launch_chain=True,removed_includes=removed,prepared_files=len(manifest)),indent=2)+'\n')
print(json.dumps(dict(passed=True,root=str(P),prepared_files=len(manifest))))
