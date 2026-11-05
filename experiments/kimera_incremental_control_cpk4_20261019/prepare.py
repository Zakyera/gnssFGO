"""Copy the diagnostic harness/profile; never touch estimator sources/binaries."""
from pathlib import Path
import ast, difflib, hashlib, json, shutil, xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs/kimera_failed_factor_cpk1_20261019'
R=W/'runs/kimera_incremental_control_cpk4_20261019'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def cp(p):return str(p).replace('/home/yeranis/repos/V4RL','/workspace')
assert not R.exists(), 'Refuse duplicate preparation'
protected=json.loads((W/'runs/kimera_consistent_landmark_cpk2_20261019/protected_before.json').read_text())
protected.update(json.loads((K/'pre_capture_receipt.json').read_text())['diagnostic_manifest'])
for p in [K/'off_capture'/n for n in ('outputs.bag','acceptance.csv','objective.csv','preflight.json','validation.json')]:protected[str(p)]=sha(p)
assert all(sha(p)==h for p,h in protected.items()), 'Protected input changed'
assert shutil.disk_usage(W).free>8*1024**3
R.mkdir()
shutil.copytree(K/'kimera_profile', R/'kimera_profile')
changes=[]
for name in ('grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch','kimera_vio_ros.launch'):
    old=(K/name).read_text();new=old.replace(cp(K),cp(R))
    if name=='kimera_vio_ros.launch':
        a='<env name="FL4Q_ENABLE" value="1"/>'
        assert new.count(a)==1
        new=new.replace(a,'<env name="FL4Q_ENABLE" value="0"/>')
    ET.fromstring(new)
    (R/name).write_text(new)
    changes+=difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(K/name),tofile=str(R/name))
for name in ('run_case.py','PROTOCOL.txt'):
    shutil.copy2(H/name,R/name)
ast.parse((R/'run_case.py').read_text())
(H/'launch.diff').write_text(''.join(changes))
(R/'protected_before.json').write_text(json.dumps(protected,indent=2))
manifest={str(p):sha(p) for p in R.rglob('*') if p.is_file()}
for p in (K/'private_build').rglob('*'):
    if p.is_file():manifest[str(p)]=sha(p)
for name in ('test','test.cpp','RetirementAudit.h','CurrentChartFallback.h','include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h'):
    manifest[str(K/name)]=sha(K/name)
for name in ('test_passed.json','sanitizer_passed.json'):
    assert json.loads((K/name).read_text())['passed']
manifest[str(K/'test_passed.json')]=sha(K/'test_passed.json')
manifest[str(K/'sanitizer_passed.json')]=sha(K/'sanitizer_passed.json')
out={'manifest':manifest,'protected_count':len(protected),'algorithmic_change':'FL4Q_ENABLE=0',
     'binary_rebuilt':False,'source_edited':False,'source_capture_limit':1,'duration_s':111,'rate':0.1}
(R/'pre_capture_receipt.json').write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k!='manifest'},indent=2))
