"""Mechanical private harness derivation; no estimator or production edits."""
from pathlib import Path
import json
import shutil

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE = WS/'runs/gnssfgo_e1_e2_live160_20261011'
RUN = WS/'runs/gnssfgo_e1b_profile40_20261012'
ROOT = RUN/'prepared'
LIB = WS/'runs/e1b_local_profile_20261012/release_2'
VALID = HERE.parent/'e1_e2_live160_validation_20261011'
def inside(p): return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
def put(p, text):
    with p.open('x') as f: f.write(text)
def once(s, a, b):
    assert s.count(a) == 1, (a, s.count(a))
    return s.replace(a, b)
def rewrite(s):
    return s.replace(BASE.name, RUN.name).replace('live160_on','profile40_on').replace('11405','11406')
ROOT.mkdir(parents=True, exist_ok=False)
shutil.copy2(HERE/'PROTOCOL.txt', RUN/'PROTOCOL.txt')
names = ['experiment.py','run_case.py','source.env','source_runner.sh',
         'active_window.launch','grandtour_eth1_kimera_stereo.launch',
         'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
         'receiver_e2.sh','verify_effective.py','coverage_checks.py',
         'validate_numerical_case.py','numerical_equivalence.py']
for name in names:
    s = rewrite((BASE/'prepared'/name).read_text())
    if name == 'run_case.py':
        s = once(s,"choices=['live160']","choices=['profile40']")
        s = once(s,'duration=160','duration=40')
    if name in ['source.env','source_runner.sh','verify_effective.py','validate_numerical_case.py']:
        s = s.replace('160.0','40.0')
    if name == 'source_runner.sh':
        s = once(s,'min_glim_rows=1400\nmin_kimera_rows=600','min_glim_rows=350\nmin_kimera_rows=150')
        s = s.replace('Prepared 160-second ON E1/E2 capacity test only.','Prepared 40-second ON E1b profiling capture only.')
    if name == 'validate_numerical_case.py':
        s = once(s,'/file),160,imu_times[0]','/file),40,imu_times[0]')
    if name == 'active_window.launch':
        s = once(s,'$(find glim_ros)/launch/glim_rosnode.launch',inside(ROOT/'glim_profile.launch'))
    if name == 'verify_effective.py':
        start=s.index("assert str(root/'private_build/libglim_ros.so')")
        end=s.index("with (source/'effective_mode_check.json')",start)
        s=s[:start]+'''candidate = Path("'''+inside(LIB)+'''")
for name in ['libglim_ros.so','libglim.so','libcbs.so','libe1b_profile.so',
             'libgtsam.so.4.3a1','libgtsam_points_cuda.so.1.2.1']:
    assert str(candidate/name) in glim_loaded, name
assert str(root/'private_build') not in glim_loaded
assert str(candidate) not in loaded and 'libe1b_profile' not in loaded
assert str(root/'private_build/libcbs.so') in loaded
def env(pid):
    return dict(x.decode().split('=',1) for x in Path('/proc/%d/environ'%pid).read_bytes().split(b'\\0') if b'=' in x)
assert env(glim_pid)['CBS_E1B_PROFILE_PATH'] == str(source/'e1b_profile.jsonl')
assert 'CBS_E1B_PROFILE_PATH' not in env(pid)
'''+s[end:]
    put(ROOT/name,s)
shutil.copytree(BASE/'prepared/private_build',ROOT/'private_build',symlinks=True)
(ROOT/'receiver_e2.sh').chmod(0o755)
s=(WS/'src/glim_ros1/launch/glim_rosnode.launch').read_text()
node='<node pkg="glim_ros" type="glim_rosnode" name="glim_rosnode" output="$(arg glim_node_output)">'
s=once(s,node,node+'\n\t\t<env name="LD_LIBRARY_PATH" value="'+inside(LIB)+':$(optenv LD_LIBRARY_PATH)"/>\n\t\t<env name="CBS_E1B_PROFILE_PATH" value="'+inside(ROOT/'profile40_on_source/e1b_profile.jsonl')+'"/>')
put(ROOT/'glim_profile.launch',s)
cfg=json.loads((BASE/'prepared/live160_on.json').read_text())
cfg['sequence']='ETH1 first40s E1b profiling only'
cfg['ros_master_uri']='http://127.0.0.1:11406'
cfg['receiver_binary']=str(ROOT/'receiver_e2.sh')
for src in cfg['sources']: src['session']='eth1_e1b_profile40_on_20261012'
put(ROOT/'profile40_on.json',json.dumps(cfg,indent=2)+'\n')
put(RUN/'validate_native.py',rewrite((VALID/'evidence/validate_native_v2.py').read_text()))
for name in ['admission_contract.py','lifecycle_contract.py']:
    shutil.copy2(VALID/name,RUN/name)
s=rewrite((VALID/'evidence/lifecycle_corrected.py').read_text())
s=once(s,'import sys','import sys\nfrom lifecycle_contract import check_kimera')
s=once(s,"ROOT = Path(__file__).resolve().parent/'prepared'","ROOT = Path(__file__).resolve().parent/'prepared'\nOUT = ROOT")
put(RUN/'validate_lifecycle.py',s)
put(RUN/'validate_diagnostics.py',rewrite((BASE/'validate_diagnostics.py').read_text()))
s=(HERE.parent/'e1_e2_live160_20261011/preflight.py').read_text()
s=once(s,"BASE=WS/'runs/gnssfgo_e1_e2_live60_20261011/prepared'","BASE=WS/'runs/gnssfgo_e1_e2_live160_20261011/prepared'")
s=once(s,"ROOT=WS/'runs/gnssfgo_e1_e2_live160_20261011/prepared'","ROOT=WS/'runs/gnssfgo_e1b_profile40_20261012/prepared'\nLIB=WS/'runs/e1b_local_profile_20261012/release_2'")
s=s.replace('11405','11406').replace("ROOT/'live160_on.json'","ROOT/'profile40_on.json'").replace("BASE/'live60_on.json'","BASE/'live160_on.json'")
s=once(s,"libenv='export LD_LIBRARY_PATH='+inside(ROOT/'private_build')", "libenv='export LD_LIBRARY_PATH='+inside(LIB)")
s=s.replace("inside(ROOT/'private_build/libglim_ros.so')","inside(LIB/'libglim_ros.so')")
s=once(s,"inside(ROOT/'private_build'/name) in ldd","inside(LIB/name) in ldd")
s=once(s,"old.update(evidence['candidate']);old.update(evidence['e1_executables'])", "old.update(evidence['candidate']);old.update(evidence['e1_executables'])\nfor name,h in json.loads((LIB/'complete.json').read_text())['artifacts'].items(): old[str(LIB/name)]=h\npinned=json.loads((HERE.parent/'e1b_local_profile_20261012/validation.json').read_text())['source_git_unchanged']\nfor repo,v in pinned.items():\n    assert subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()==v['head']\n    assert not subprocess.check_output(['git','-C',repo,'status','--porcelain'],text=True).strip()")
s=s.replace('E1_E2_LIVE160_PREFLIGHT_PASS','E1B_PROFILE40_PREFLIGHT_PASS')
put(HERE/'preflight.py',s)
s=rewrite((HERE.parent/'e1_e2_live160_20261011/sample_resources.py').read_text())
s=s.replace("'WNOJ_INCREMENTAL_PROFILE']","'WNOJ_INCREMENTAL_PROFILE','CBS_E1B_PROFILE_PATH']")
s=once(s,"  if exe.endswith('/replay'):","  item['all_library_maps']=[s for s in (p/'maps').read_text().splitlines() if '.so' in s]\n  if exe.endswith('/replay'):")
put(HERE/'sample_resources.py',s)
print(ROOT)
