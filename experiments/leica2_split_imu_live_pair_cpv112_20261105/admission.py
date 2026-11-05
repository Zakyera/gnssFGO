"""Final host admission, exact parity, old-evidence preservation; no launch."""
import ast,datetime,difflib,hashlib,json,os,shutil,socket,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
from paths import *
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,z):
    with p.open('x') as f:json.dump(z,f,indent=2,allow_nan=False)
def call(args):return subprocess.check_output(args,text=True)
def identity(pid):
    p=Path(f'/proc/{pid}/stat');return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
assert not (H/'prepared.json').exists() and not (H/'start.json').exists()
assert not V.exists() and not any(P.glob('full_*_source')) and not any(P.glob('full_*_receiver'))
for name in ('raw_input_audit.json','report_preflight.json','validator_test.json','live_adapter_test.json','host_runtime_preflight.json'):
    assert read(H/name)['passed'],name
assert call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for port in (11651,11652):
    with socket.socket() as s:s.bind(('127.0.0.1',port))
assert all(not os.environ.get(k) for k in ('CPV66_AUDIT_DIR','CPV66_TEST_ALL','KIMERA_IMU_SAMPLE_SUPPORT','CPV102_PRIOR_FAILURE_DIR'))
profiles={};files=[];diff=[]
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if p.suffix=='.py':ast.parse(p.read_text(),str(p))
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
for mode in ('off','on'):
    for p in (P/mode/'input_profiles').rglob('*'):
        if p.is_file():
            ref=QUAL_P/'on/input_profiles'/p.relative_to(P/mode/'input_profiles')
            assert sha(p)==sha(ref),p;profiles[str(p)]=sha(p)
    for lib in ('libkimera_vio.so','libcbs.so'):
        assert sha(P/mode/'private_build'/lib)==sha(QUAL_P/'on/private_build'/lib)
    runner=(P/mode/'source_runner_on.sh').read_text()
    guard=next(x for x in runner.splitlines() if 'CP-V112 LEICA-2 split-IMU full' in x and '[[' in x)
    assert '428.75' in guard and '6.0' in guard and '2024-11-25-16-36-19' in guard
    assert ('${cbs_enable}" == '+str(mode=='on').lower()) in guard
    env=(P/mode/'source.env').read_text()
    for expected in ('CBSMS_MISSION=2024-11-25-16-36-19','CBSMS_BAG_START_SEC=6.0','CBSMS_BAG_DURATION_SEC=428.75',
                     'CBSMS_BAG_RATE=0.20','CBSMS_POST_PLAYBACK_DRAIN_SEC=25','CBSMS_SEED_CLOCK_SECS=1732549034',
                     'CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour/LEICA-2',
                     'CBSMS_ROS_MASTER_PORT='+str(11651+(mode=='on'))):assert expected in env,expected
    assert ('/full_'+mode+'_source/e1b_profile.jsonl') in (P/mode/'glim_profile_on.launch').read_text()
    for name in ('input_audit.json','expected_headers.json'):assert sha(P/name)==sha(P/mode/name)
    actual=read(P/f'full_{mode}.json');old=read(QUAL_P/'full_on.json')
    for k in ('sequence','ros_master_uri','receiver_binary'):actual.pop(k);old.pop(k)
    for a,b in zip(actual['sources'],old['sources']):a.pop('session');b.pop('session')
    assert actual==old,'Receiver mathematical configuration changed'
assert os.access(P/'receiver_cpk19.sh',os.X_OK)
for z in read(H/'copy_parity.json')['files']:
    p=P/z['path'];old=Path(z['source'])
    assert sha(old)==z['source_sha256'],old
    assert p.stat().st_mode&0o777==old.stat().st_mode&0o777,p
    files.append(dict(path=str(p),sha256=sha(p),source=str(old),source_sha256=sha(old)))
    if p.suffix in ('.py','.sh','.env','.json','.launch','.txt') and 'input_audit' not in p.name and 'expected_headers' not in p.name:
        diff.extend(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),fromfile=str(old),tofile=str(p)))
for filename in ('frame_adapter.py','live_receiver_protocol.py','launch_grandtour_live_receiver.py','grandtour_live_receiver_ros1.py'):
    actual=(P/'transport_tools'/filename).read_bytes()
    expected=(QUAL_P/'transport_tools'/filename).read_bytes()
    if filename=='live_receiver_protocol.py':
        # The sole allowed difference is the exact pinned wrapper pathname.
        old_path=str(QUAL_P/'receiver_cpk19.sh').encode()
        new_path=str(P/'receiver_cpk19.sh').encode()
        assert expected.count(old_path)==1 and actual.count(new_path)==1
        expected=expected.replace(old_path,new_path)
    assert actual==expected,filename
save(H/'profile_parity.json',dict(passed=True,profiles=profiles,files=files,corrected_binary_both_modes=True,
    scientific_settings_CP103_unchanged=True,mission_specific_changes=True,source_covariance_freeze=False,
    operational_changes=['LEICA2 bags/window/clock seed','mission rigid transform','ports/sessions','expected input counts and reference']))
with (H/'final_copy.diff').open('x') as f:f.write(''.join(diff))
pins=read(BASE_P/'protected_manifest.json')
seals={}
for name in (BASE_H.name,'eig2_fusion_localization_cpv109_20261104','eig2_fusion_intervention_cpv110_20261104'):
    root=E/name;z=read(root/'complete.json');assert z['passed']
    pins.update(z['output_sha256']);pins[str(root/'complete.json')]=sha(root/'complete.json')
    seals[name]=len(z['output_sha256'])
    if (root/'post_closeout.json').exists():pins[str(root/'post_closeout.json')]=sha(root/'post_closeout.json')
pins.update(read(H/'report_contract_test/input_sha256.json'))
audit=read(P/'input_audit.json')
for v in audit['bag_files'].values():pins[v['path']]=v['sha256']
pins[audit['tf']['path']]=audit['tf']['sha256']
pins[str(W/'src/datasets/GrandTour/LEICA-2/2024-11-25-16-36-19_cpt7_ie_tc.bag')]=sha(W/'src/datasets/GrandTour/LEICA-2/2024-11-25-16-36-19_cpt7_ie_tc.bag')
for p,v in pins.items():assert sha(p)==v,p
print('Preserved prior pins, terminal evidence and mission data',len(pins),flush=True)
jobs=[]
for root in (QUAL_H,BASE_H):
    for p in [root/'start.json',*root.glob('*_process.json')]:
        z=read(p);assert identity(z['pid'])!=str(z['proc_start']),p
        jobs.append(dict(receipt=str(p),pid=z['pid'],proc_start=z['proc_start'],exited=True))
    z=read(root/'launch.json');assert identity(z['owned_supervisor_pid'])!=str(z['supervisor_proc_start'])
for z in read(E/'eig2_fusion_intervention_cpv110_20261104/closeout.json')['owned_processes']:
    assert identity(z['pid'])!=str(z['proc_start']),z
    jobs.append(dict(receipt='CP110/closeout.json',pid=z['pid'],proc_start=z['proc_start'],exited=True))
save(H/'prior_jobs_exited.json',dict(passed=True,jobs=jobs))
hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
storage=dict(passed=True,free_bytes=shutil.disk_usage(P).free,forecast_bytes=20*2**30,
             required_free_bytes=25*2**30,runtime_floor_bytes=5*2**30,
             basis='20GiB full-pair allowance scales CP108 16GiB by LEICA2/EIG2 duration ratio1.208, plus5GiB reserve')
assert storage['free_bytes']>storage['required_free_bytes'];save(H/'storage_preflight.json',storage)
for root in (H,P):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.name not in ('status.json','REPORT.txt','prepared.json','preflight.json','protected_manifest.json'):
            pins[str(p)]=sha(p)
save(P/'protected_manifest.json',pins)
out=dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),prepared_path=str(P),
    protected_files=len(pins),hesai=hesai,prior_terminal_outputs=seals,source_captures_authorized=2,receiver_captures_authorized=2,
    mission='LEICA-2',duration_s=428.75,offset_s=6.0,rate=.20,ports=[11651,11652],
    corrected_backend_both_modes=True,source_covariance_handling_unchanged=True,old_captures_reused=False)
save(P/'preflight.json',out);save(H/'prepared.json',out)
print(json.dumps(out,indent=2))
