"""Copy the frozen CP-K26 harness; change only identity and declared crop bounds."""
from pathlib import Path
import ast, difflib, hashlib, importlib.util, json, re, shutil, socket, subprocess, sys
import xml.etree.ElementTree as ET
sys.dont_write_bytecode = True
H=Path(__file__).resolve().parent
OLD=H.parent/'spx3_awake_full_cpk26_20261021'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared'
P=W/'runs/gnssfgo_spx3_stationary_preroll_cpk27_20261023/prepared'
V=Path('/home/yeranis/results/gnssfgo_spx3_stationary_preroll_cpk27_20261023')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def rebase(s):
    for a,b in [(S.parent.name,P.parent.name),(OLD.name,H.name),
                ('spx3_cpk26_full','spx3_cpk27_preroll'),('SPX3-CPK26','SPX3-CPK27'),
                ('CP-K26','CP-K27'),('11476','11477'),
                ('194.02','28.0'),('192.02','26.0'),('1907','260'),
                ('1937','277'),('1940','280'),('32.0','29.0'),
                ('1730565901','1730565898'),('bag32-226.02s','bag29-57s')]:s=s.replace(a,b)
    return s
diff=[]
def generate(src,dest,s):
    old=src.read_text()
    with dest.open('x') as f:f.write(s)
    shutil.copymode(src,dest)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(dest)))
assert not P.parent.exists() and not V.exists() and not (H/'status.json').exists()
assert json.loads((OLD/'closeout.json').read_text())['passed']
assert json.loads((OLD/'status.json').read_text())['state']=='complete'
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as s:s.bind(('127.0.0.1',11477))
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p

# Reuse input audit, with exact bounded crop and unchanged sensor synchronization checks.
s=(OLD/'bag_preflight.py').read_text().replace('epoch + 32_000_000_000','epoch + 29_000_000_000')
s=s.replace('duration_ns = math.ceil((last_hesai-start+1)/10_000_000)*10_000_000','duration_ns = 28_000_000_000')
s=s.replace('assert 130_000_000_000 < duration_ns < 230_000_000_000, (epoch,start,last_hesai,duration_ns)',
            'assert duration_ns == 28_000_000_000 and start == 1730565899872888651')
s=s.replace("assert selected[TOPICS['hesai'][0]][-1] == last_hesai", "last_hesai = selected[TOPICS['hesai'][0]][-1]")
s=s.replace('bag_offset_s=32.0','bag_offset_s=29.0')
generate(OLD/'bag_preflight.py',H/'bag_preflight.py',s)
with (H/'input_preflight.log').open('x') as log:
    subprocess.run(['/home/yeranis/.local/share/pipx/venvs/evo/bin/python',str(H/'bag_preflight.py')],stdout=log,stderr=subprocess.STDOUT,check=True)
pre=json.loads((H/'bag_preflight.json').read_text())
assert pre['stereo_pairs']==280 and pre['window']['duration_s']==28.0
P.mkdir(parents=True)
for folder in ('private_build','input_profiles','transport_tools'):
    shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
files=['coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py','eof_boundary.py',
       'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
       'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
       'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
       'preflight_on.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env',
       'source_runner.sh','source_runner_on.sh','validate_candidate.py','validate_case.py',
       'validate_local.py','verify_effective.py','verify_effective_on.py']
for name in files:
    s=rebase((S/name).read_text())
    if name=='evaluate_fl4f.py':
        assert 'len(times)>500' in s
        s=s.replace('len(times)>500','len(times)>200')
    generate(S/name,P/name,s)
shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for path in (P/'transport_tools').rglob('*.py'):
    old=path.read_text();new=rebase(old)
    if old!=new:
        path.write_text(new)
        diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/path.relative_to(P)),tofile=str(path)))
for name in ('run_pair.py','summarize.py'):
    s=rebase((OLD/name).read_text())
    if name=='summarize.py':
        s=s.replace('start=1730565902.872888651','start=1730565899.872888651').replace("('last20s',174.02)","('last20s',8.0)")
        s=s.replace('One full-sequence experimental pair','One28s startup pre-roll pair').replace('full28.0s','bounded28.0s')
    generate(OLD/name,H/name,s)
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for folder in ('private_build','input_profiles'):
    for p in (S/folder).rglob('*'):
        if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
for name in ('expiry_audit.py','eof_boundary.py','map_guard.py'):assert sha(S/name)==sha(P/name)
for p in [*P.rglob('*'),*H.glob('*.py')]:
    if p.suffix=='.py':ast.parse(p.read_text())
    elif p.suffix=='.launch':ET.parse(p)
    elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
for base in (P,H):
    for p in base.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.name!='REPORT.txt':manifest[str(p)]=sha(p)
with (P/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
pre.update(template=str(S),protected_files=len(manifest),model_identical=True,
           source_profiles_byte_identical=True,all_input_bytes_identical=True,
           change='3s stationary pre-roll; bounded28s capture',
           eof_contract='finite_recording_eof_cpk24a',disk_free_bytes=shutil.disk_usage(W).free)
(P/'preflight').mkdir()
with (P/'preflight/validation.json').open('x') as f:json.dump(pre,f,indent=2)
print(json.dumps(dict(passed=True,prepared=str(P),protected_files=len(manifest),window=pre['window']),indent=2))
