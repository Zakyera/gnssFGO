"""Fresh-root mechanical harness adaptation; never starts estimators."""
import ast,csv,difflib,hashlib,json,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import yaml
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=E/'eth1_split_imu_off60_cpv85_20261101_attempt1';Q=W/'runs'/OLD.name/'prepared'
P=W/'runs'/H.name/'prepared';BASE=W/'runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared/input_profiles'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def swap(text,old,new,count=1):
    assert text.count(old)==count,(old,text.count(old));return text.replace(old,new)
assert json.loads((OLD/'report_completion_1/complete.json').read_text())['report_complete']
if P.exists():
    assert [p.name for p in P.iterdir()]==['source_runner_on.sh']
    assert sha(P/'source_runner_on.sh')==sha(Q/'source_runner_on.sh')
P.mkdir(parents=True,exist_ok=True)
names=['source_runner_on.sh','source.env','verify_effective_on.py','verify_hesai_startup.py','map_guard.py','coverage_checks.py',
       'active_window_on.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch']
diff=[]
for name in names:
    src=Q/name;dst=P/name;shutil.copy2(src,dst)
    original=src.read_text();s=original.replace(OLD.name,H.name).replace('11587','11588').replace('CP-V85','CP-V86').replace('cpv85_frame','cpv86_frame')
    if name=='source.env':s=swap(s,'CBSMS_KIMERA_MODE=stereo\n','CBSMS_KIMERA_MODE=stereo_shared_cpt7\n')
    if name=='source_runner_on.sh':
        s=swap(s,'&& "${kimera_mode}" == stereo ]] ||','&& "${kimera_mode}" == stereo_shared_cpt7 ]] ||')
        s=swap(s,'if [[ "${kimera_mode}" == stereo ]]; then\n  bags+=','if [[ "${kimera_mode}" == stereo_shared_cpt7 ]]; then\n  bags+=')
        s=swap(s,'if [[ "${kimera_mode}" == stereo ]]; then\n  play_topics+=','if [[ "${kimera_mode}" == stereo_shared_cpt7 ]]; then\n  play_topics+=')
        s=s.replace('coverage_split.py','coverage_control.py').replace('split-IMU','CPT7-control')
    if name=='verify_effective_on.py':
        s=swap(s,"os.environ['CBSMS_KIMERA_MODE'] == 'stereo'","os.environ['CBSMS_KIMERA_MODE'] == 'stereo_shared_cpt7'")
        s=swap(s,"assert '/boxi/alphasense/imu' in kimera_subscriptions and '/boxi/cpt7/imu' not in kimera_subscriptions","assert '/boxi/cpt7/imu' in kimera_subscriptions and '/boxi/alphasense/imu' not in kimera_subscriptions")
        s=swap(s,"assert kimera['base_link_frame_id']=='alphasense_imu'","assert kimera['base_link_frame_id']=='cpt7_imu'")
        start=s.index('import tf\n');end=s.index('master = rosgraph.Master',start)
        s=s[:start]+"# Both source bodies are CPT7; no cross-body transform is required in OFF.\n"+s[end:]
        s=s.replace('split IMU topics/body transform','shared CPT7 topics/body')
    dst.write_text(s)
    diff.extend(difflib.unified_diff(original.splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(dst)))
shutil.copytree(Q/'private_build',P/'private_build')
shutil.copytree(BASE,P/'input_profiles')
profile_diff={}
for p in (P/'input_profiles').rglob('*'):
    if not p.is_file():continue
    rel=p.relative_to(P/'input_profiles');before=Q/'input_profiles'/rel
    if sha(p)!=sha(before):
        assert str(rel) in ['kimera_profile/LeftCameraParams.yaml','kimera_profile/RightCameraParams.yaml','kimera_profile/ImuParams.yaml'],rel
        a=yaml.safe_load(before.read_text().replace('%YAML:1.0',''));b=yaml.safe_load(p.read_text().replace('%YAML:1.0',''))
        keys=[k for k in a if a[k]!=b[k]];assert a.keys()==b.keys()
        assert set(keys)<=({'T_BS'} if 'Camera' in p.name else {'rate_hz','gyroscope_noise_density','accelerometer_noise_density','gyroscope_random_walk','accelerometer_random_walk'})
        profile_diff[str(rel)]=dict(changed_fields=keys,old_sha256=sha(before),new_sha256=sha(p))
assert len(profile_diff)==3
for name in ['controller.py','launch.py']:
    s=(OLD/name).read_text().replace('CPV85','CPV86').replace('separate-IMU CBS-OFF source qualification','matched CPT7 CBS-OFF control')
    with (H/name).open('x') as f:f.write(s)
shutil.copy2(H/'coverage_control.py',P/'coverage_control.py')
audit=json.loads((Q/'input_audit.json').read_text())
audit['scope']='CP86 CPT7 control; CP85 raw input audit reused after hash verification'
audit['kimera_body']='cpt7_imu';audit['profile_differences_from_CP85']=profile_diff
audit['prior_input_audit']=dict(path=str(Q/'input_audit.json'),sha256=sha(Q/'input_audit.json'))
audit.pop('noise');audit.pop('calibration');audit.pop('noise_density_matches_raw_message_covariance')
audit['kimera_imu_parameters']=yaml.safe_load((P/'input_profiles/kimera_profile/ImuParams.yaml').read_text().replace('%YAML:1.0',''))
for v in audit['bag_files'].values():assert sha(v['path'])==v['sha256']
assert sha(audit['tf']['path'])==audit['tf']['sha256']
# Preserve exact raw header sequences and camera timestamps for endpoint conservation.
expected={}
for topic,name in [('/boxi/cpt7/imu','cpt7_headers.csv'),('/boxi/alphasense/imu','alphasense_headers.csv')]:
    with (Q/'off_source'/name).open() as f:rows=list(csv.DictReader(f))
    key=next(k for k in rows[0] if k.endswith('.stamp'));expected[topic]=[int(r[key]) for r in rows]
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader
with AnyReader([Path(audit['bag_files']['alphasense']['path'])]) as r:
    topics=['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']
    for c,t,b in r.messages(connections=[c for c in r.connections if c.topic in topics],start=audit['epoch_ns'],stop=audit['end_ns']):
        m=r.deserialize(b,c.msgtype);expected.setdefault(c.topic,[]).append(int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec))
assert expected[topics[0]]==expected[topics[1]] and len(expected[topics[0]])==601
save(P/'expected_headers.json',expected);save(P/'input_audit.json',audit)
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
save(H/'profile_parity.json',dict(passed=True,CP85=str(Q),CPT7_profile=str(BASE),differences=profile_diff,
    private_libraries={p.name:sha(p) for p in (P/'private_build').iterdir() if p.is_file()},no_build=True,unapplied_diagnostic_corrections=True))
for root in (P,H):
    for p in root.rglob('*'):
        if p.suffix=='.py':ast.parse(p.read_text())
        if p.suffix in ['.sh','.env']:subprocess.run(['bash','-n',str(p)],check=True)
print('Prepared fresh files:',P)
