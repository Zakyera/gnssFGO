"""Finish exactly the already-authorized matched pair; never restart a case.

OFF is already running. Wait for its owned completion, validate once, then ON
once, validate once, score. Any failure stops this controller permanently.
"""
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'prepared'
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
lock=(ROOT/'pair_controller.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (ROOT/'pair_failed.txt').exists()
assert not (ROOT/'pair_complete.json').exists()
assert (RUN/'full160_off_control').is_dir(), 'OFF must already be started'
assert not (RUN/'full160_on_control').exists(), 'Refuse duplicate ON controller'
with (ROOT/'pair_controller.pid').open('x') as f:f.write(str(os.getpid())+'\n')
started=time.time()
def execute(label,args):
    print(label,flush=True)
    with (ROOT/(label+'.log')).open('x') as f:
        subprocess.run(args,cwd=RUN,stdout=f,stderr=subprocess.STDOUT,check=True)
try:
    while not (RUN/'full160_off_control/complete.json').exists():
        assert time.time()-started<2300,'OFF completion timeout'
        for path in [RUN/'full160_off_control/failed.txt',RUN/'full160_off_receiver/failed.txt']:
            assert not path.exists(),str(path)+': '+(path.read_text() if path.exists() else '')
        time.sleep(5)
    execute('validate_off',[PY,str(RUN/'validate_case.py'),'full160_off'])
    assert json.loads((RUN/'full160_off_validation.json').read_text())['passed']
    execute('run_on',['python3',str(RUN/'run_case.py'),'full160','on'])
    execute('validate_on',[PY,str(RUN/'validate_case.py'),'full160_on'])
    assert json.loads((RUN/'full160_on_validation.json').read_text())['passed']
    inside='/workspace/'+str((ROOT/'evaluate_full160.py').relative_to('/home/yeranis/repos/V4RL'))
    execute('evaluate',['docker','exec','cbsms_ws_gpu','bash','-lc',
        'source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && exec '+shlex.join(['python3',inside])])
    execute('covariance',[PY,str(ROOT/'analyze_covariance.py')])
    with (ROOT/'pair_complete.json').open('x') as f:
        json.dump(dict(passed=True,seconds=time.time()-started,
            captures=['full160_off','full160_on'],validation='Both source and receiver, same-journal deterministic replay',
            evaluation=str(ROOT/'evaluation/summary.json'),covariance=str(ROOT/'covariance_analysis/summary.json')),f,indent=2)
    print('MATCHED PAIR COMPLETE',flush=True)
except BaseException as e:
    with (ROOT/'pair_failed.txt').open('x') as f:f.write(repr(e)+'\n')
    raise
