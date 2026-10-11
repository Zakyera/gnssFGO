#!/usr/bin/env python3
"""One owned, bounded E1 capture. No retry, receiver, or automatic next step."""
import fcntl
import json
import subprocess
import time
from pathlib import Path
root=Path(__file__).resolve().parent
assert str(root)=='/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/glim_e1_real_graph_20261011/prepared'
inside='/workspace/cbs_gtsam4.3/runs/glim_e1_real_graph_20261011/prepared'
lock=(root/'controller.lock').open('a'); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (root/'control').exists() and not (root/'e1_source').exists()
assert json.loads((root/'preflight.json').read_text())['passed']
own=root/'control'; own.mkdir()
code='''import os,subprocess,sys
from pathlib import Path
root=Path(sys.argv[1])
command='export CBSMS_RUN_NAME=e1_source CBSMS_CBS_ENABLE=true CBSMS_BAG_DURATION_SEC=20.0; source '+str(root/'source.env')+'; exec bash '+str(root/'source_runner.sh')
proc=subprocess.Popen(['bash','-lc',command],start_new_session=True)
(root/'control/owned.pid').write_text(str(proc.pid))
raise SystemExit(proc.wait())
'''
log=(own/'source.log').open('x')
proc=subprocess.Popen(['docker','exec','cbsms_ws_gpu','python3','-c',code,inside],stdout=log,stderr=subprocess.STDOUT)
start=time.monotonic()
try:
    print('STARTED E1 real-graph ETH1 first20s, source-only, rate0.1',flush=True)
    while proc.poll() is None:
        if (root/'audit/FAILED.txt').exists(): raise RuntimeError((root/'audit/FAILED.txt').read_text())
        if time.monotonic()-start>420: raise RuntimeError('bounded source wall deadline exceeded')
        time.sleep(1)
    assert proc.returncode==0 and (root/'e1_source/complete.marker').exists(),'source failed; see control/source.log'
    for n in [10,30,60,100,150,180]:
        d=root/'audit'/('sample_'+str(n))
        assert json.loads((d/'summary.json').read_text())['passed']
        assert (d/'means_unchanged.marker').exists()
    (own/'complete.json').write_text(json.dumps({'passed':True,'wall_s':time.monotonic()-start},indent=2))
    print('CAPTURE_COMPLETE; post-validation still required',flush=True)
except BaseException as exc:
    (own/'failed.txt').write_text(str(exc)+'\n')
    raise
finally:
    if proc.poll() is None:
        stop='''import os,signal,sys
from pathlib import Path
p=Path(sys.argv[1]); pid=int(p.read_text()) if p.exists() else 0
c=Path('/proc/%d/cmdline'%pid); e=Path('/proc/%d/environ'%pid)
if pid>1 and c.exists() and e.exists() and b'CBSMS_RUN_NAME=e1_source\\x00' in e.read_bytes():
    os.killpg(os.getpgid(pid),signal.SIGINT)
'''
        subprocess.run(['docker','exec','cbsms_ws_gpu','python3','-c',stop,inside+'/control/owned.pid'],check=False)
        try: proc.wait(timeout=30)
        except subprocess.TimeoutExpired: print('Owned group did not exit within 30s; inspect before further action',flush=True)
    log.close()
