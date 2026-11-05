"""Fresh mechanical CP108->SPX3 harness copy. No source or receiver launch."""
import ast,difflib,hashlib,json,re,shutil,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from paths import *
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
assert read(H/'raw_input_audit.json')['passed']
assert read(BASE_H/'complete.json')['passed'] and read(BASE_H/'post_closeout.json')['passed']
assert not P.exists();P.mkdir(parents=True)
audit=read(H/'raw_input_audit.json');inv=read(H/'available_data_review.json')
assert inv['CP103_lidar_extrinsic_max_abs']<1e-9
for z in inv['cameras'].values():assert max(z['comparison_to_CP103'].values())<1e-9
replacements=[(BASE_H.name,H.name),('2024-11-03-13-59-54','2024-11-02-17-43-10'),
 ('EIG-2','SPX-3'),('EIG2-CPV108','SPX3-CPV111'),('EIG2','SPX3'),
 ('eig2_cpv108_','spx3_cpv111_'),('CP-V108','CP-V111'),('11631','11641'),('11632','11642'),
 ('1730638815','1730565898'),('355020000000','197020000000'),('355.02','197.02'),('353.02','195.02')]
def rebase(s):
    for a,b in replacements:s=s.replace(a,b)
    s=re.sub(r'(?<![\w.])5\.7(?![\w.])','29.0',s)
    for a,b in [('3550','1970'),('3547','1967'),('3530','1950')]:
        s=re.sub(r'(?<!\d)'+a+r'(?!\d)',b,s)
    return s
records=[];diff=[]
def record(src,dst):
    records.append(dict(path=str(dst.relative_to(P)),source=str(src),source_sha256=sha(src),sha256=sha(dst)))
    if dst.suffix in ('.py','.sh','.env','.json','.launch','.txt') and dst.name not in ('input_audit.json','expected_headers.json'):
        diff.extend(difflib.unified_diff(src.read_text().splitlines(True),dst.read_text().splitlines(True),fromfile=str(src),tofile=str(dst)))
for row in read(BASE_H/'profile_parity.json')['files']:
    src=Path(row['path']);rel=src.relative_to(BASE_P);dst=P/rel
    assert src.is_file() and not src.is_symlink() and sha(src)==row['sha256']
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    if 'input_profiles' not in rel.parts and dst.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        s=rebase(src.read_text())
        if dst.name=='input_audit.json':s=(H/'raw_input_audit.json').read_text()
        elif dst.name=='expected_headers.json':s=(H/'expected_headers.json').read_text()
        elif dst.name=='frame_calibration.json':
            z=json.loads(s);z['T_alphasense_cpt7']=audit['T_alphasense_cpt7'];z['source']=str(H/'raw_input_audit.json');s=json.dumps(z,indent=2)
        dst.write_text(s)
    assert dst.stat().st_mode&0o777==src.stat().st_mode&0o777
    if 'input_profiles' in rel.parts or 'private_build' in rel.parts:assert sha(dst)==sha(src)
    records.append(dict(path=str(rel),source=str(src),source_sha256=sha(src),sha256=sha(dst)))

X=np.asarray(audit['T_alphasense_cpt7']);q=Rotation.from_matrix(X[:3,:3]).as_quat()
for mode in ('off','on'):
    p=P/mode/'active_window_on.launch';s=p.read_text();xml=ET.fromstring(s)
    node=next(x for x in xml.iter('node') if x.attrib.get('name')=='grandtour_alpha_imu_to_cpt7_imu_tf')
    args=' '.join(format(float(v),'.17g') for v in [*X[:3,3],*q])+' alphasense_imu cpt7_imu'
    p.write_text(once(s,node.attrib['args'],args))
    assert sha(P/mode/'private_build/libkimera_vio.so')=='985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09'
sys.path.insert(0,str(P/'transport_tools'))
from frame_adapter import native_to_cpt7,adjoint
items=read(P/'transport_tools/runtime_frame_fixture.json');J=np.asarray(adjoint(np.linalg.inv(X)));error=0.
for z in items:
    mu=np.asarray(z['native']['mean']);C=np.asarray(z['native']['covariance']).reshape(6,6)
    z['expected']=native_to_cpt7('kimera',z['native'],X)
    error=max(error,float(np.max(abs(np.asarray(z['expected']['mean'])-J@mu))),float(np.max(abs(np.asarray(z['expected']['covariance']).reshape(6,6)-J@C@J.T))))
assert error<1e-10
(P/'transport_tools/runtime_frame_fixture.json').write_text(json.dumps(items,indent=2))
save(H/'frame_fixture_preflight.json',dict(passed=True,fixtures=len(items),matrix_error_max=error,mission_transform=True,source_runs=0))

names=('validate_source.py','validate_case.py','validate_local.py','frame_on.py','frame_off.py','injection_schema.py',
       'evaluate.py','verify_report.py','controller.py','launch.py','post_closeout.py','preflight_tests.py','admission.py')
for name in names:
    src=BASE_H/name;old=src.read_text();s=rebase(old)
    if name=='preflight_tests.py':
        s=once(s,"old=W/'runs/eig2_symmetric_persistent_full_cpv84_20261101_attempt1/prepared/full_on_source'",
                  "old=W/'runs/gnssfgo_spx3_stationary_full_cpk28_20261023/prepared/full_on_source'")
    elif name=='verify_report.py':
        # SPX3 is shorter in distance, independently established by raw IE-TC.
        # Preserve positive path, exact EVO/archive checks, >100 pairs/poses.
        s=once(s,"v['reference_path_length_m']>100","v['reference_path_length_m']>50")
    elif name=='admission.py':
        s=once(s,"pins=read(QUAL_P/'protected_manifest.json')","pins=read(BASE_P/'protected_manifest.json')")
        oldseal="for name in (QUAL_H.name,'eth1_prior_fix_evaluation_cpv104_20261104','eth1_fusion_diagnosis_cpv105_20261104',\n             'eth1_covariance_freeze_cpv106_20261104','eth1_covariance_provenance_cpv107_20261104'):"
        newseal="for name in (BASE_H.name,'eig2_fusion_localization_cpv109_20261104','eig2_fusion_intervention_cpv110_20261104'):"
        s=once(s,oldseal,newseal)
        s=s.replace('SPX3 duration2.5%longer','SPX3 window shorter than CP108; retain conservative16GiB allowance')
    ast.parse(s)
    with (H/name).open('x') as f:f.write(s)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(H/name)))
for z in records:
    dst=P/z['path'];src=Path(z['source']);z['sha256']=sha(dst)
    if dst.suffix in ('.py','.sh','.env','.json','.launch','.txt') and dst.name not in ('input_audit.json','expected_headers.json'):
        diff.extend(difflib.unified_diff(src.read_text().splitlines(True),dst.read_text().splitlines(True),fromfile=str(src),tofile=str(dst)))
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file():continue
        if p.suffix=='.py':ast.parse(p.read_text(),str(p))
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
save(H/'copy_parity.json',dict(passed=True,files=records,source_profiles_byte_identical=True,source_binaries_byte_identical=True,
  baseline=str(BASE_H),mission_changes=['bag paths/window/clock','session/ports','mission calibration transform/fixture','raw counts and evaluation reference'],
  covariance_intervention=False,ports=[11641,11642]))
with (H/'mechanical_copy.diff').open('x') as f:f.write(''.join(diff))
print('Prepared fresh SPX3 harness; no source or receiver started',P)
