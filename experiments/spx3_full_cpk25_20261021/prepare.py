"""Reuse the completed EIG-2 harness with SPX-3 identities and explicit EOF audit."""
from pathlib import Path
import ast,difflib,hashlib,json,re,shutil,socket,subprocess
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eig2_full_cpk24_20261021/prepared'
P=W/'runs/gnssfgo_spx3_full_cpk25_20261021/prepared'
V=Path('/home/yeranis/results/gnssfgo_spx3_full_cpk25_20261021')
H24=H.parent/'eig2_full_cpk24_20261021'
SP=W/'runs/gnssfgo_spx3_local_cov60_cpr5_20261019/prepared'
E=SP/'input_profiles'

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def common(s):
    for old,new in [(S.parent.name,P.parent.name),(H24.name,H.name),
        ('eig2_cpk24_full','spx3_cpk25_full'),('CP-K24','CP-K25'),
        ('EIG2-CPK24','SPX3-CPK25'),('EIG-2','SPX-3'),
        ('2024-11-03-13-59-54','2024-11-02-17-43-10'),
        ('1730638816.563546034','1730565902.872888651'),
        ('355.02','194.02'),('353.02','192.02'),('335.02','174.02'),
        ('bag5.7-360.72s','bag32-226.02s')]:
        s=s.replace(old,new)
    for old,new in [('11474','11475'),('1730638815','1730565901'),('3517','1907')]:
        s=re.sub(r'(?<![A-Za-z0-9_])'+old+r'(?![A-Za-z0-9_])',new,s)
    return s

def replace_once(s,old,new):
    assert s.count(old)==1, (old,s.count(old))
    return s.replace(old,new)

assert not P.parent.exists() and not V.exists()
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as sock:sock.bind(('127.0.0.1',11475))
manifest=json.loads((S/'protected_manifest.json').read_text())
for name,want in manifest.items():assert sha(name)==want,name
preflight=json.loads((H/'bag_preflight.json').read_text());assert preflight['passed']
assert preflight['window']['duration_s']==194.02 and preflight['stereo_pairs']==1940
for item in preflight['bag_files'].values():
    st=Path(item['path']).stat()
    assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns']),item['path']
old_profile=json.loads((SP/'preflight/validation.json').read_text())['profile_sha256']
for folder,files in old_profile.items():
    assert {str(p.relative_to(E/folder)):sha(p) for p in (E/folder).rglob('*') if p.is_file()}==files
P.mkdir(parents=True)
for folder in ('private_build','transport_tools'):
    shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(E,P/'input_profiles',symlinks=True)
shutil.copy2(H24/'eof_review/eof_boundary.py',P/'eof_boundary.py')
files=['PROTOCOL.txt','coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py',
 'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
 'preflight_on.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env',
 'source_runner.sh','source_runner_on.sh','validate_candidate.py','validate_case.py',
 'validate_local.py','verify_effective.py','verify_effective_on.py']
diff=[]
for name in files:
    old=(S/name).read_text();new=common(old)
    if name=='PROTOCOL.txt':new=(H/name).read_text()
    if name=='source.env':new=new.replace('CBSMS_BAG_START_SEC=5.7','CBSMS_BAG_START_SEC=32.0')
    if name.startswith('source_runner'):
        new=new.replace('CBSMS_BAG_START_SEC:-5.7','CBSMS_BAG_START_SEC:-32.0')
        new=new.replace('"${bag_start_sec}" == 5.7','"${bag_start_sec}" == 32.0')
    if name.startswith('verify_effective'):
        new=new.replace("['CBSMS_BAG_START_SEC']=='5.7'","['CBSMS_BAG_START_SEC']=='32.0'")
    if name=='coverage_checks.py':new=new.replace('3547','1937').replace('3550','1940')
    if name=='validate_case.py':
        new=replace_once(new,'from expiry_audit import ExpiryAudit','from expiry_audit import ExpiryAudit\nfrom eof_boundary import audit_eof')
        new=replace_once(new,"    assert action['unsupported_at_end']==summary['unsupported_end']==0",
                        '    boundary=audit_eof(P,MODE,summary,action,online_records,events)')
        new=replace_once(new,'result=dict(passed=True,source_coverage=',
                        "result=dict(passed=True,validation_contract='finite_recording_eof_cpk24a',terminal_boundary=boundary,source_coverage=")
    (P/name).write_text(new);shutil.copymode(S/name,P/name)
    if old!=new:diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for path in (P/'transport_tools').rglob('*.py'):path.write_text(common(path.read_text()))
for name in ('run_pair.py','summarize.py'):
    assert not (H/name).exists()
    old=(H24/name).read_text();new=common(old)
    if name=='summarize.py':
        new=replace_once(new,'result=dict(passed=True,cases=cases,',
            "result=dict(passed=True,validation_contract='finite_recording_eof_cpk24a',terminal_omissions={m:c['checks']['receiver']['terminal_boundary']['excluded_versions'] for m,c in cases.items()},cases=cases,")
        new=replace_once(new,"'Native GLIM / native Kimera ROS odometry / concurrent ONLINE fusion.',",
            "'Native GLIM / native Kimera ROS odometry / concurrent ONLINE fusion.',\n 'EOF contract: only final partial-cell omissions allowed; every omission disclosed below. No fabricated support.',\n 'Terminal omissions OFF/ON: '+str(result['terminal_omissions']),")
    (H/name).write_text(new)
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(H24/name),tofile=str(H/name)))
(H/'harness.diff').write_text(''.join(diff))
for folder in ('private_build','input_profiles'):
    for path in (S/folder).rglob('*'):
        if path.is_file():assert sha(path)==sha(P/path.relative_to(S)),path
for name in ('expiry_audit.py','map_guard.py'):assert sha(P/name)==sha(S/name),name
assert sha(P/'eof_boundary.py')==sha(H24/'eof_review/eof_boundary.py')
for name in ('full_off.json','full_on.json'):
    a=json.loads((S/name).read_text());b=json.loads((P/name).read_text())
    for c in (a,b):
        for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
        for source in c['sources']:source.pop('session')
    assert a==b,name
for path in [*P.rglob('*'),*H.glob('*.py')]:
    if path.suffix=='.py':ast.parse(path.read_text())
    elif path.suffix=='.launch':ET.parse(path)
    elif path.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(path)],check=True)
import importlib.util
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
boundary_tests=module.test_boundaries()
for base in (P,H):
    for path in base.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.name!='REPORT.txt':manifest[str(path)]=sha(path)
(P/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
preflight.update(template=str(S),protected_files=len(manifest),model_identical=True,
    source_profiles_byte_identical=True,source_profile_origin=str(E),window_specific_count_gates=True,
    coverage_semantics_unchanged=True,eof_contract='finite_recording_eof_cpk24a',boundary_tests=boundary_tests,
    sleep_prevention='required scoped systemd-inhibit',disk_free_bytes=shutil.disk_usage(W).free)
(P/'preflight').mkdir()
(P/'preflight/validation.json').write_text(json.dumps(preflight,indent=2)+'\n')
print(json.dumps(dict(passed=True,prepared=str(P),protected_files=len(manifest),window=preflight['window'],boundary_tests=boundary_tests),indent=2))
