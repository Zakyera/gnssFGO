"""Run retained validators, then one deterministic journal replay; no source retry."""
from pathlib import Path
import fcntl
import json
import shlex
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
PREP=ROOT/'prepared'
TAG='live160_on'
assert (PREP/(TAG+'_control/complete.json')).is_file()
lock=(PREP/'controller.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
logs=ROOT/'validation_logs'; logs.mkdir(exist_ok=False)
def run(name,cmd):
    with (logs/(name+'.log')).open('x') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
try:
    script='/workspace/'+str((PREP/'validate_native_bidirectional.py').relative_to('/home/yeranis/repos/V4RL'))
    run('native',['docker','exec','cbsms_ws_gpu','bash','-lc',
        'source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '+shlex.join(['python3',script,TAG])])
    run('lifecycle',[sys.executable,str(ROOT/'check_lifecycle.py'),TAG])
    run('diagnostics',[sys.executable,str(ROOT/'validate_diagnostics.py'),TAG])
    run('numerical',[sys.executable,str(PREP/'validate_numerical_case.py'),TAG])
    assert json.loads((PREP/(TAG+'_validation.json')).read_text())['passed']
    print('FULL 160-SECOND ON VALIDATED',flush=True)
except BaseException as e:
    (logs/'failed.txt').write_text(repr(e)+'\n')
    raise
