"""Fresh mission-specific copy; no capture or changes to any predecessor."""
import ast,difflib,hashlib,json,shutil,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from paths import *
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def replace(s,old,new,n=1):
    assert s.count(old)==n,(old,s.count(old),n)
    return s.replace(old,new)
assert read(H/'raw_input_audit.json')['passed']
assert read(QUAL_H/'complete.json')['passed'] and read(QUAL_H/'post_closeout.json')['passed']
assert not P.exists()
P.mkdir(parents=True)
records=[];diffs=[]
newhash='985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09'
oldhash='b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8'
audit=read(H/'raw_input_audit.json')
def normalized(s):
    return s.replace(BASE_H.name,H.name).replace(QUAL_H.name,H.name)
for rel in read(BASE_H/'profile_parity.json')['relative_files']:
    src=BASE_P/rel;dst=P/rel
    assert src.is_file() and not src.is_symlink()
    dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    if rel.endswith('private_build/libkimera_vio.so'):
        shutil.copy2(QUAL_P/'on/private_build/libkimera_vio.so',dst)
    elif dst.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        text=normalized(src.read_text())
        if dst.name=='input_audit.json':text=json.dumps(audit,indent=2)
        elif dst.name=='expected_headers.json':text=(H/'expected_headers.json').read_text()
        elif dst.name=='source.env':
            text=text.replace('2024-10-01-11-29-55','2024-11-03-13-59-54')
            text=text.replace('CBSMS_BAG_START_SEC=0','CBSMS_BAG_START_SEC=5.7').replace('346.32','355.02')
            # Keep the established pre-bag seed; mission time starts with playback.
            text=text.replace('11611','11631').replace('11612','11632')
            text=replace(text,'export CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour',
                         'export CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour/EIG-2')
        elif dst.name=='run_case_on.py':
            text=text.replace('11611','11631').replace('346.32','355.02')
        elif dst.name in ('full_off.json','full_on.json'):
            text=text.replace('11611','11631').replace('11612','11632').replace('346.32','355.02')
            text=text.replace('ETH1 CP-V99','EIG-2 CP-V108').replace('eth1_cpv99_','eig2_cpv108_').replace('_20261103','_20261104')
        elif dst.name=='kimera_vio_ros.launch':
            needle='    <env name="CPK9_PRIOR_REFRESH" value="1"/>'
            text=replace(text,needle,needle+'\n    <env name="CPV102_PRIOR_FAILURE_DIR" value="$(env ROS_LOG_DIR)/../prior_failures"/>')
        elif dst.name=='source_runner_on.sh':
            needle='cp "${launch_file}" "${run_dir}/grandtour_launch_used.launch"'
            text=replace(text,needle,'mkdir -p "${run_dir}/prior_failures"\n'+needle)
        elif dst.name=='verify_effective_on.py':
            text=normalized((QUAL_P/'on/verify_effective_on.py').read_text())
            text=text.replace('346.32','355.02').replace('2024-10-01-11-29-55','2024-11-03-13-59-54')
            text=replace(text,"assert os.environ['CBSMS_BAG_START_SEC']=='0'","assert os.environ['CBSMS_BAG_START_SEC']=='5.7'")
            text=replace(text,"assert enabled, 'CP-V91 requires BOTH source receivers enabled'","assert enabled is (root.name=='on'), 'Mode mismatch'")
            text=replace(text,"assert glim['glim_cbs_receive_external_beliefs'] is True","assert glim['glim_cbs_receive_external_beliefs'] is enabled")
            text=text.replace('kimera_receive=True,glim_receive=True','kimera_receive=enabled,glim_receive=enabled')
            text=text.replace('CP-V91 split IMU','CP-V108 split IMU').replace('BOTH CBS receivers ON, persistent.','Matched requested receive mode, persistent.')
        elif dst.name=='coverage_control.py':text=text.replace('346.32','355.02')
        elif dst.name=='frame_calibration.json':
            z=json.loads(text);z['T_alphasense_cpt7']=audit['T_alphasense_cpt7'];z['source']=str(H/'raw_input_audit.json');text=json.dumps(z,indent=2)
        dst.write_text(text)
    assert dst.stat().st_mode&0o777==src.stat().st_mode&0o777,rel
    if 'input_profiles' in dst.parts:assert sha(dst)==sha(src),rel
    records.append(dict(path=rel,source=str(src),source_sha256=sha(src),sha256=sha(dst)))
    if dst.suffix in ('.py','.sh','.env','.json','.launch','.txt') and dst.read_text()!=src.read_text():
        diffs.extend(difflib.unified_diff(src.read_text().splitlines(True),dst.read_text().splitlines(True),fromfile=str(src),tofile=str(dst)))

# Mission static transform, not an ETH1 fallback. Keep every other launch value.
X=np.array(audit['T_alphasense_cpt7']);q=Rotation.from_matrix(X[:3,:3]).as_quat()
for mode in ('off','on'):
    path=P/mode/'active_window_on.launch';s=path.read_text();tree=ET.fromstring(s)
    node=next(x for x in tree.iter('node') if x.attrib.get('name')=='grandtour_alpha_imu_to_cpt7_imu_tf')
    args=' '.join(format(float(v),'.17g') for v in [*X[:3,3],*q])+' alphasense_imu cpt7_imu'
    path.write_text(replace(s,node.attrib['args'],args))
    # Copied input profiles remain byte-identical: mission extrinsics agree to <1e-9.
    assert sha(P/mode/'private_build/libkimera_vio.so')==newhash

# Recompute fixed-scalar fixture results for the mission transform; independent
# matrix multiplication and geometric conjugation are tested separately.
sys.path.insert(0,str(P/'transport_tools'))
from frame_adapter import native_to_cpt7,adjoint
items=read(P/'transport_tools/runtime_frame_fixture.json')
Y=np.linalg.inv(X);J=np.array(adjoint(Y));err=0.
for z in items:
    mu=np.array(z['native']['mean']);C=np.array(z['native']['covariance']).reshape(6,6)
    z['expected']=native_to_cpt7('kimera',z['native'],X)
    err=max(err,float(np.max(abs(np.array(z['expected']['mean'])-J@mu))),float(np.max(abs(np.array(z['expected']['covariance']).reshape(6,6)-J@C@J.T))))
assert err<1e-10
(P/'transport_tools/runtime_frame_fixture.json').write_text(json.dumps(items,indent=2))
save(H/'frame_fixture_preflight.json',dict(passed=True,fixtures=len(items),matrix_error_max=err,mission_transform=True,source_runs=0))

for name in ('validate_source.py','validate_case.py','validate_local.py','frame_on.py','frame_off.py','injection_schema.py','evaluate.py','verify_report.py','controller.py'):
    src=(QUAL_H if name=='validate_source.py' else BASE_H)/name
    text=src.read_text()
    if name in ('validate_source.py','validate_case.py'):
        text=text.replace('346.32','355.02').replace('344.32','353.02').replace('==3463','==3550').replace('>=3443','>=3530').replace('ETH1','EIG-2')
    if name=='validate_local.py':text=text.replace('first>=3440','first>=3530')
    if name=='evaluate.py':
        text=text.replace('346.32','355.02').replace('CP-V99','CP-V108').replace('FULL ETH1','FULL EIG-2')
        text=replace(text,"row=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1');_,_,ref=d.m.c.b.load(row)\n    reference=physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')",'reference=load_reference()')
        text=text.replace('One absent right image203.898841154s and actual native keyframe gaps retained;', 'All3550 raw stereo pairs matched; actual native keyframe gaps retained;')
        text=text.replace('Both live arms use new captures; CP89/91 remain historical saved controls, not these pair results.', 'Both arms are fresh EIG-2 captures with the same corrected private Kimera binary.')
        text=text.replace('def main():', '''def load_reference():
    from scipy.spatial.transform import Rotation
    sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
    from rosbags.highlevel import AnyReader
    path=W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag'
    ts=[];pp=[];qq=[]
    with AnyReader([d.pin(path)]) as reader:
        for conn,_,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/boxi/inertial_explorer/tc/odometry']):
            msg=reader.deserialize(raw,conn.msgtype);v=msg.pose.pose
            ts.append(int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec))
            pp.append([v.position.x,v.position.y,v.position.z]);qq.append([v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
    ts=np.array(ts,np.int64);assert np.all(np.diff(ts)>0)
    return physical((ts,np.array(pp),Rotation.from_quat(qq).as_matrix()),'cpt7')

def main():''')
    if name=='controller.py':
        text=text.replace('ETH1-CPV99','EIG2-CPV108').replace('346.32','355.02')
        text=replace(text,"    step('compare_repeat',[PY,'-B',str(H/'compare_repeat.py')],timeout=1200)\n    assert read(H/'repeat_comparison.json')['passed'];count=hashes()",'    count=hashes()')
    (H/name).write_text(text)

text=(QUAL_H/'launch.py').read_text().replace('ON qualification','OFF/ON pair').replace('ETH1-CPV103','EIG2-CPV108')
text=text.replace('User-authorized corrected-prior ETH1 ON validation with fusion','User-authorized EIG-2 split-IMU full OFF/ON with fusion')
text=text.replace('ON_only=True,source_captures_authorized=1','fresh_pair=True,source_captures_authorized=2')
(H/'launch.py').write_text(text)
text=(QUAL_H/'post_closeout.py').read_text().replace("P.glob('full_on_control/*_process.json')","P.glob('full_*_control/*_process.json')").replace('source_runs_started=1','source_runs_started=2')
(H/'post_closeout.py').write_text(text)
for root in (H,P):
    for path in root.rglob('*'):
        if path.is_file() and path.suffix=='.py':ast.parse(path.read_text(),str(path))
        if path.is_file() and path.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(path)],check=True)
        if path.is_file() and path.suffix=='.launch':ET.parse(path)
for z in records:z['sha256']=sha(P/z['path'])
save(H/'copy_parity.json',dict(passed=True,files=records,source_profiles_byte_identical=True,
    changed_binary='both private Kimera use qualified CP102 DSO',mission_changes=['bag paths/window','session/ports','calibration transform/fixture','raw counts and evaluation reference'],
    covariance_intervention=False,ports=[11631,11632]))
with (H/'mechanical_copy.diff').open('x') as f:f.write(''.join(diffs))
print('Fresh EIG-2 files prepared; no launch',P)
