"""Mechanical copy of frozen CP112 static inputs; never launches a capture."""
import ast,difflib,hashlib,json,os,re,shutil,stat,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=E/'leica2_split_imu_live_pair_cpv112_20261105';OP=W/'runs'/OLD.name/'prepared';P=W/'runs'/H.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
def rebase(s):
    s=s.replace(OLD.name,H.name).replace('leica2_cpv112_off_20261104','leica2_cpv112a1_off_20261105').replace('leica2_cpv112_on_20261104','leica2_cpv112a1_on_20261105').replace('LEICA2-CPV112','LEICA2-CPV112-A1')
    return re.sub(r'(?<![\d.])11651(?!\d)','11653',re.sub(r'(?<![\d.])11652(?!\d)','11654',s))
assert read(OLD/'post_closeout.json')['passed'] and read(OLD/'failed.json')['stage']=='capture_off'
assert not P.exists();P.mkdir(parents=True)
records=[];diff=[]
for z in read(OLD/'profile_parity.json')['files']:
    src=Path(z['path']);assert sha(src)==z['sha256'] and not src.is_symlink()
    rel=src.relative_to(OP);dst=P/rel;dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    if dst.suffix in ('.py','.sh','.env','.json','.launch','.txt') and 'input_profiles' not in rel.parts:
        s=rebase(src.read_text())
        if dst.name=='source.env':
            s=once(s,'export CBSMS_BAG_START_SEC=6.0','export CBSMS_BAG_START_SEC=6.0\nexport CBSMS_BAG_PLAYBACK_START_SEC=6.099999100')
        elif dst.name=='source_runner_on.sh':
            s=once(s,'bag_start_sec="${CBSMS_BAG_START_SEC:-6.0}"','bag_start_sec="${CBSMS_BAG_START_SEC:-6.0}"\nplayback_start_sec="${CBSMS_BAG_PLAYBACK_START_SEC:?Exact bag-relative offset required}"')
            s=once(s,'"${bag_start_sec}" == 6.0 &&','"${bag_start_sec}" == 6.0 && "${playback_start_sec}" == 6.099999100 &&')
            s=once(s,'--start="${bag_start_sec}"','--start="${playback_start_sec}"')
            s=once(s,'bag_pid=$!','bag_pid=$!\nprintf "selected_offset_s=%s\\nrosbag_start_s=%s\\n" "${bag_start_sec}" "${playback_start_sec}" >"${run_dir}/playback_offsets.txt"')
        elif dst.name=='verify_effective_on.py':
            s=once(s,"assert os.environ['CBSMS_BAG_START_SEC']=='6.0'","assert os.environ['CBSMS_BAG_START_SEC']=='6.0'\nassert os.environ['CBSMS_BAG_PLAYBACK_START_SEC']=='6.099999100'\nassert json.loads((root.parent/'playback_window.json').read_text())['passed']")
        dst.write_text(s)
    if 'input_profiles' in rel.parts or 'private_build' in rel.parts:assert sha(src)==sha(dst)
    records.append(dict(path=str(rel),source=str(src),source_sha256=sha(src),sha256=sha(dst)))
    if dst.suffix in ('.py','.sh','.env','.json','.launch','.txt') and sha(src)!=sha(dst):
        diff.extend(difflib.unified_diff(src.read_text().splitlines(True),dst.read_text().splitlines(True),fromfile=str(src),tofile=str(dst)))
names=('paths.py','validate_source.py','validate_case.py','validate_local.py','frame_off.py','frame_on.py','injection_schema.py','evaluate.py','verify_report.py','controller.py','launch.py','post_closeout.py','preflight_tests.py','admission.py')
for name in names:
    src=OLD/name;s=rebase(src.read_text())
    if name=='paths.py':s+='\nFAILED_H=E/'+repr(OLD.name)+'\nFAILED_P=W/"runs"/FAILED_H.name/"prepared"\n'
    elif name=='admission.py':
        s=once(s,"'host_runtime_preflight.json'):","'host_runtime_preflight.json','playback_window_test.json'):")
        s=once(s,"'CBSMS_BAG_RATE=0.20'","'CBSMS_BAG_PLAYBACK_START_SEC=6.099999100','CBSMS_BAG_RATE=0.20'")
        s=once(s,"pins=read(BASE_P/'protected_manifest.json')","pins=read(FAILED_P/'protected_manifest.json')\npins.update(read(H/'preserved_failed_regular.json'))\nfrom preserve_failed import verify_special\nverify_special()")
        s=once(s,"for root in (QUAL_H,BASE_H):","for root in (QUAL_H,BASE_H,FAILED_H):")
        s=once(s,"ports=[11653,11654],","ports=[11653,11654],rosbag_start_s='6.099999100',absolute_window_unchanged=True,failed_attempt_preserved=True,")
    elif name=='controller.py':
        s=once(s,"return len(pins)","from preserve_failed import verify_special\n    verify_special()\n    return len(pins)")
        s=once(s,"    for mode in ('off','on'):","    step('container_playback_window',['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash && python3 -B '+inside+'/playback_window.py --noetic'],timeout=600)\n    for mode in ('off','on'):")
    elif name=='post_closeout.py':
        s=once(s,"pins=read(P/'protected_manifest.json')","from preserve_failed import verify_special\nverify_special()\npins=read(P/'protected_manifest.json')")
    ast.parse(s)
    with (H/name).open('x') as f:f.write(s)
    diff.extend(difflib.unified_diff(src.read_text().splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(H/name)))
for name in ('raw_input_audit.json','available_data_review.json','expected_headers.json','frame_fixture_preflight.json'):
    shutil.copy2(OLD/name,H/name)
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file():continue
        if p.suffix=='.py':ast.parse(p.read_text(),str(p))
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
save(H/'copy_parity.json',dict(passed=True,files=records,source_profiles_byte_identical=True,source_binaries_byte_identical=True,baseline=str(OLD),ports=[11653,11654],changes=['fresh paths/sessions/ports','explicit bag-origin offset translation','new read-only preplayback window guard'],scientific_settings_unchanged=True))
with (H/'mechanical_copy.diff').open('x') as f:f.write(''.join(diff))
regular={};special={}
for root in (OLD,OP):
    for p in root.rglob('*'):
        st=p.lstat()
        if stat.S_ISREG(st.st_mode):regular[str(p)]=sha(p)
        elif stat.S_ISLNK(st.st_mode):special[str(p)]=dict(kind='symlink',target=os.readlink(p),mode=stat.S_IMODE(st.st_mode))
        elif stat.S_ISFIFO(st.st_mode):special[str(p)]=dict(kind='fifo',mode=stat.S_IMODE(st.st_mode))
        else:assert stat.S_ISDIR(st.st_mode),(str(p),st.st_mode)
save(H/'preserved_failed_regular.json',regular);save(H/'preserved_failed_special.json',special)
save(H/'preservation_summary.json',dict(passed=True,regular_files=len(regular),special_files=len(special),failed_root=str(OLD),source_root=str(OP),fifo_read=False))
print(json.dumps(dict(prepared_static=True,path=str(P),files=len(records),failed_regular_files=len(regular),failed_special_files=len(special),captures_started=0)))
