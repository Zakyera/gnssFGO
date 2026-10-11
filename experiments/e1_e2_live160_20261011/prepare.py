"""Mechanical duration extension of the validated private60s harness only."""
from pathlib import Path
import json
import shutil

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=WS/'runs/gnssfgo_e1_e2_live60_20261011'
RUN=WS/'runs/gnssfgo_e1_e2_live160_20261011'
ROOT=RUN/'prepared'
PREV=HERE.parent/'e1_e2_live60_20261011'
assert json.loads((BASE/'prepared/live60_on_validation.json').read_text())['passed']
assert json.loads((BASE/'engineering_summary.json').read_text())['headroom']['met']
ROOT.mkdir(parents=True,exist_ok=False)
def put(p,s):
    with p.open('x') as f:f.write(s)
def replace(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def rewrite(s):
    return s.replace('gnssfgo_e1_e2_live60_20261011','gnssfgo_e1_e2_live160_20261011').replace('live60_on','live160_on').replace('11404','11405')
for name in ['postvalidate.py','check_lifecycle.py','validate_diagnostics.py','status.py']:
    s=rewrite((BASE/name).read_text()).replace('FULL 60-SECOND','FULL 160-SECOND')
    put(RUN/name,s)
for p in (BASE/'prepared').iterdir():
    if not p.is_file() or p.suffix not in ['.py','.sh','.env','.launch']:continue
    s=rewrite(p.read_text())
    if p.name=='run_case.py':
        s=replace(s,"choices=['live60']","choices=['live160']")
        s=replace(s,'duration=60','duration=160')
    if p.name in ['source.env','source_runner.sh','verify_effective.py','validate_numerical_case.py']:
        s=s.replace('60.0','160.0')
    if p.name=='source_runner.sh':
        s=replace(s,'min_glim_rows=550\nmin_kimera_rows=250','min_glim_rows=1400\nmin_kimera_rows=600')
        s=s.replace('Prepared 60-second ON E1 live acceptance only.','Prepared 160-second ON E1/E2 capacity test only.')
    if p.name=='validate_numerical_case.py':s=replace(s,"/file),60,imu_times[0]","/file),160,imu_times[0]")
    put(ROOT/p.name,s)
shutil.copytree(BASE/'prepared/private_build',ROOT/'private_build',symlinks=True)
(ROOT/'receiver_e2.sh').chmod(0o755)
cfg=json.loads((BASE/'prepared/live60_on.json').read_text())
cfg['sequence']='ETH1 first160s E1 plus corrected E2 capacity gate'
cfg['ros_master_uri']='http://127.0.0.1:11405'
cfg['receiver_binary']=str(ROOT/'receiver_e2.sh')
for src in cfg['sources']:src['session']='eth1_e1_e2_live160_on_20261011'
put(ROOT/'live160_on.json',json.dumps(cfg,indent=2)+'\n')
shutil.copy2(HERE/'PROTOCOL.txt',RUN/'PROTOCOL.txt')
for name in ['sample_resources.py','finish_checks.py','check_exact_repeat.py','archive.py']:
    put(HERE/name,rewrite((PREV/name).read_text()))
s=(PREV/'preflight.py').read_text()
s=replace(s,"BASE=WS/'runs/gnssfgo_e1_live60_20261011/prepared'","BASE=WS/'runs/gnssfgo_e1_e2_live60_20261011/prepared'")
s=replace(s,"ROOT=WS/'runs/gnssfgo_e1_e2_live60_20261011/prepared'","ROOT=WS/'runs/gnssfgo_e1_e2_live160_20261011/prepared'")
s=s.replace('11404','11405').replace("(ROOT/'live60_on.json')","(ROOT/'live160_on.json')")
s=s.replace("assert (ROOT/'receiver_e2.sh').read_bytes()==(HERE/'receiver_e2.sh').read_bytes()","assert (ROOT/'receiver_e2.sh').read_bytes()==(BASE/'receiver_e2.sh').read_bytes()")
s=s.replace('E1_E2_LIVE60_PREFLIGHT_PASS','E1_E2_LIVE160_PREFLIGHT_PASS')
put(HERE/'preflight.py',s)
print(ROOT)
