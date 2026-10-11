"""One validation pass after a genuine capture; no automatic retries.

Runs immutable-journal numerical repeat only, never reruns source estimation.
"""
import fcntl
import json
import shlex
import subprocess
import sys
from experiment import ROOT, REPO, inside

tag=sys.argv[1]
assert tag in ['full160_off','full160_on']
lock=(ROOT/'controller.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert (ROOT/(tag+'_control/complete.json')).is_file()
assert not (ROOT/(tag+'_validation.json')).exists()
out=ROOT/(tag+'_validation_logs');out.mkdir(exist_ok=False)
def run(name,cmd):
    with (out/(name+'.log')).open('x') as f:
        subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
try:
    cmd='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && exec '+shlex.join(
        ['python3',inside(ROOT/'validate_native_bidirectional.py'),tag])
    run('native',['docker','exec','cbsms_ws_gpu','bash','-lc',cmd])
    run('diagnostics',[sys.executable,str(ROOT/'validate_diagnostics.py'),tag])
    if tag.endswith('_on'):
        run('lifecycle',[sys.executable,str(ROOT/'check_lifecycle.py'),tag])
    run('inventory',[sys.executable,str(REPO/'experiments/covariance_interaction_eth1_160_preparation_20261010/inspect_capture.py'),
        '--source',str(ROOT/(tag+'_source')),'--receiver',str(ROOT/(tag+'_receiver')),
        '--out',str(ROOT/(tag+'_covariance_inventory.json'))])
    run('numerical',[sys.executable,str(ROOT/'validate_numerical_case.py'),tag])
    assert json.loads((ROOT/(tag+'_validation.json')).read_text())['passed']
    print(tag+' VALIDATED',flush=True)
except BaseException as e:
    with (out/'failed.txt').open('x') as f:f.write(repr(e)+'\n')
    raise
