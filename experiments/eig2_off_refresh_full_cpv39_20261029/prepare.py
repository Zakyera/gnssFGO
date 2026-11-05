"""Prepare/freeze one fresh full replay using the already validated private DSO."""
import ast, hashlib, json, shutil, xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eig2_off_refresh_full_cpv39_20261029'
OLD=W/'runs/eig2_counterfactual_cpv36_20261029'
K=W/'runs/kimera_off_incremental_refresh_cpv38_20261029'
H38=H.parent/'kimera_off_incremental_refresh_cpv38_20261029'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,d):
    with p.open('x') as f:json.dump(d,f,indent=2)
assert not (H/'ready.json').exists() and not (P/'capture').exists()
assert shutil.disk_usage(W).free>8*1024**3
receipt=json.loads((H38/'closeout.json').read_text());assert receipt['passed']
assert sha(K/'private_build/libkimera_vio.so')=='560ad4e82323e91db43eae606bafcd3ecec6ccbd85dcf271fb84a7f50658ecf4'
pins=json.loads((H/'protected_before.json').read_text())
for p,s in pins.items():assert sha(p)==s,p
assert P.is_dir() and not (P/'run_inside.py').exists()
assert len(ET.parse(P/'kimera_vio_ros.launch').findall('.//env[@name="CPV38_OFF_REFRESH"]'))==1
for name in ('kimera_only.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','reference_effective.json','observe.py'):
    assert sha(P/name)==sha(OLD/name),name
for p in (P/'kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(OLD/p.relative_to(P)),p
text=(OLD/'run_inside.py').read_text()
replacements={
 'Single owned300s original-start Kimera OFF diagnostic':'Single owned355.02s original-start Kimera OFF refresh',
 "PORT=11536":"PORT=11539",
 "http://127.0.0.1:11536":"http://127.0.0.1:11539",
 "assert (P/'build_backend/passed.json').exists()":"K=W/'runs/kimera_off_incremental_refresh_cpv38_20261029'\nfor name,s in json.loads((P/'runtime_manifest.json').read_text()).items():assert sha(K/name)==s,name",
 "CPV36_AUDIT_DIR=str(C/'snapshots')":"CPV38_OFF_REFRESH='1'",
 "str(P/'private_build')":"str(K/'private_build')",
 "'FL4U_PROBE')":"'FL4U_PROBE','CPV35_AUDIT_DIR','CPV36_AUDIT_DIR')",
 "--duration=300":"--duration=355.02",
 "duration=300":"duration=355.02",
 "/cpv36_check":"/cpv39_check",
 "assert env['CPV36_AUDIT_DIR']==str(C/'snapshots')":"assert env['CPV38_OFF_REFRESH']=='1'",
 "str(P/'private_build'/name)":"str(K/'private_build'/name)",
 "'CPV36_AUDIT_DIR','KIMERA_CBS_REVISION_AUDIT'":"'CPV38_OFF_REFRESH','KIMERA_CBS_REVISION_AUDIT'",
 "CPV36 OFF replay STARTED,300s":"CPV39 OFF refresh STARTED,355.02s",
 "started<2000":"started<2300",
 "'CPV36 duplicate','CPV36 snapshot write failed'":"'CPV38 invalid','CPV38 requires','CPV38 unaccounted','CPV38 incomplete'",
 "coverage_and_parity_pending=True":"coverage_and_evaluation_pending=True",
 "CPV36 capture complete; parity and diagnostic analysis pending":"CPV39 capture complete; verification and evo analysis pending",
}
for old,new in replacements.items():
    assert old in text,old;text=text.replace(old,new)
ast.parse(text);(P/'run_inside.py').write_text(text)
shutil.copy2(P/'run_inside.py',H/'run_inside.py')
shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
runtime={str(p.relative_to(K)):sha(p) for p in (K/'private_build').iterdir() if p.is_file()}
save(P/'runtime_manifest.json',runtime)
for p in P.rglob('*.launch'):ET.parse(p)
for p in P.glob('*.py'):ast.parse(p.read_text())
manifest={str(p.relative_to(P)):sha(p) for p in P.rglob('*') if p.is_file()}
save(P/'prepared_manifest.json',manifest)
save(H/'ready.json',dict(passed=True,protected_files=len(pins),prepared_files=len(manifest),runtime=runtime,duration=355.02,rate=.20,start=5.7,drain=25,selector='CPV38_OFF_REFRESH=1',source_runs=0))
print('CP-V39 prepared and frozen; no replay yet',flush=True)
