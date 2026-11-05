"""Fresh path copy with an explicit, recorded three-file instrumentation patch."""
import ast,difflib,hashlib,json,os,shutil,subprocess,xml.etree.ElementTree as ET
from paths import H,E,W,P,V,PY,BASE_H,BASE_P,FIX_H,FIX_K
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
from pathlib import Path
def save(n,v):
    with (H/n).open('x') as f:json.dump(v,f,indent=2)
assert read(FIX_H/'complete.json')['passed']
assert not P.exists() and not V.exists()
old_hash='b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8'
new_hash=sha(FIX_K/'private_build/libkimera_vio.so')
assert new_hash==read(FIX_H/'complete.json')['library_sha256']['libkimera_vio.so']
P.mkdir(parents=True)
records=[];patches={}
for rel in read(BASE_H/'profile_parity.json')['relative_files']:
    if rel.startswith('off/') or rel=='full_off.json':continue
    src=BASE_P/rel;dst=P/rel
    assert src.is_file() and not src.is_symlink()
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(src,dst)
    if dst.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        text=src.read_text().replace(BASE_H.name,H.name)
        if dst.name in ('source.env','run_case_on.py','full_on.json'):
            text=text.replace('11611','11621').replace('11612','11622')
        if dst.name=='full_on.json':text=text.replace('CP-V99','CP-V103').replace('eth1_cpv99_','eth1_cpv103_')
        base=text
        if rel=='run_case_on.py':
            assert text.count("choices=['off','on']")==1
            text=text.replace("choices=['off','on']","choices=['on']")
        if rel=='on/kimera_vio_ros.launch':
            needle='    <env name="CPK9_PRIOR_REFRESH" value="1"/>'
            assert text.count(needle)==1
            text=text.replace(needle,needle+'\n    <env name="CPV102_PRIOR_FAILURE_DIR" value="$(env ROS_LOG_DIR)/../prior_failures"/>')
        if rel=='on/source_runner_on.sh':
            needle='cp "${launch_file}" "${run_dir}/grandtour_launch_used.launch"'
            assert text.count(needle)==1
            text=text.replace(needle,'mkdir -p "${run_dir}/prior_failures"\n'+needle)
        if rel=='on/verify_effective_on.py':
            assert text.count(old_hash)==1;text=text.replace(old_hash,new_hash)
            needle="assert env(pid)['CPK9_PRIOR_REFRESH']=='1'"
            addition="\narchive_dir=Path(env(pid)['CPV102_PRIOR_FAILURE_DIR']).resolve()\nassert archive_dir==source/'prior_failures' and archive_dir.is_dir() and os.access(archive_dir,os.W_OK)\nassert 'CPV102_PRIOR_FAILURE_DIR' not in env(glim_pid)\nassert not list(archive_dir.iterdir())"
            assert text.count(needle)==1;text=text.replace(needle,needle+addition)
            needle='kimera_pid=pid, kimera_argv=argv,'
            assert text.count(needle)==1
            text=text.replace(needle,"prior_failure_archive=str(archive_dir), private_kimera_sha256='"+new_hash+"', "+needle)
        if text!=base:patches[rel]=''.join(difflib.unified_diff(base.splitlines(True),text.splitlines(True),fromfile='CP99 normalized',tofile='CP103'))
        dst.write_text(text)
    if rel=='on/private_build/libkimera_vio.so':shutil.copy2(FIX_K/'private_build/libkimera_vio.so',dst)
    if 'input_profiles' in dst.parts:assert sha(src)==sha(dst),rel
    if rel=='on/private_build/libcbs.so':assert sha(src)==sha(dst)
    assert src.stat().st_mode&0o777==dst.stat().st_mode&0o777,rel
    if dst.suffix=='.py':ast.parse(dst.read_text(),str(dst))
    if dst.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(dst)],check=True)
    if dst.suffix=='.launch':ET.parse(dst)
    records.append(dict(path=rel,old_sha256=sha(src),new_sha256=sha(dst)))
for name in ('validate_source.py','validate_local.py','validate_case.py','frame_on.py','injection_schema.py'):
    shutil.copy2(BASE_H/name,H/name)
    if name=='validate_source.py':
        text=(H/name).read_text();assert text.count(old_hash)==1
        text=text.replace(old_hash,new_hash)
        needle="assert not (S/'acceptance.csv').exists()"
        text=text.replace(needle,needle+"\nassert effective['private_kimera_sha256']=='"+new_hash+"'\nassert Path(effective['prior_failure_archive']).name=='prior_failures'\nassert (S/'prior_failures').is_dir() and not list((S/'prior_failures').iterdir())")
        (H/name).write_text(text)
save('copy_parity.json',dict(passed=True,files=records,explicit_patches=patches,
    changed_binary='on/private_build/libkimera_vio.so',scientific_settings_unchanged=True,
    scope='Fresh roots/port/session, ON-only CLI, new qualified DSO, failure-only archive and checks'))
print('Prepared fresh CP103 ON-only paths; no launch.',P)
