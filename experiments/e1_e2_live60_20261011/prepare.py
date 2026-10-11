"""Mechanical private harness derivation, no estimator or old-file edits."""
from pathlib import Path
import json
import shutil

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE=WS/'runs/gnssfgo_e1_live60_20261011'
RUN=WS/'runs/gnssfgo_e1_e2_live60_20261011'
ROOT=RUN/'prepared'
BINARY='/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay'
ROOT.mkdir(parents=True,exist_ok=False)
def put(p,s):
    with p.open('x') as f:f.write(s)
def rewrite(s):
    return s.replace('gnssfgo_e1_live60_20261011','gnssfgo_e1_e2_live60_20261011').replace('11403','11404')
for name in ['postvalidate.py','check_lifecycle.py','validate_diagnostics.py','status.py']:
    put(RUN/name,rewrite((BASE/name).read_text()))
for p in (BASE/'prepared').iterdir():
    if p.is_file() and p.suffix in ['.py','.sh','.env','.launch']:
        s=rewrite(p.read_text())
        if p.name=='experiment.py':
            s=s.replace('/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build/replay',BINARY)
            s=s.replace("ENV = dict(","ENV = dict(E2_EXACT_LINEARIZATION_CACHE='1', ")
        put(ROOT/p.name,s)
shutil.copytree(BASE/'prepared/private_build',ROOT/'private_build',symlinks=True)
cfg=json.loads((BASE/'prepared/live60_on.json').read_text())
cfg['sequence']='ETH1 first60s E1 plus corrected E2 combined live integration'
cfg['ros_master_uri']='http://127.0.0.1:11404'
cfg['receiver_binary']=str(ROOT/'receiver_e2.sh')
for src in cfg['sources']:src['session']='eth1_e1_e2_live60_on_20261011'
put(ROOT/'live60_on.json',json.dumps(cfg,indent=2)+'\n')
shutil.copy2(HERE/'receiver_e2.sh',ROOT/'receiver_e2.sh')
(ROOT/'receiver_e2.sh').chmod(0o755)
shutil.copy2(HERE/'PROTOCOL.txt',RUN/'PROTOCOL.txt')
print(ROOT)
