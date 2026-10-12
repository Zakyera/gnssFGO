"""Single execution of the inherited frozen-run gates, then recorded oracle."""
from pathlib import Path
import json
import os
import subprocess
import sys
from prepare import RUN,ROOT,HERE,inside

assert (ROOT/'profile40_on_control/complete.json').exists()
assert not (ROOT/'profile40_on_control/failed.txt').exists()
logs=RUN/'validation_logs';logs.mkdir(exist_ok=False)
py='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
commands=[('native',['docker','exec','cbsms_ws_gpu','bash','-lc',
  'source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '+
  f'python3 {inside(RUN)}/validate_native.py --capture {inside(RUN)} --out {inside(ROOT)}/profile40_on_native_validation.json']),
  ('lifecycle',[py,str(RUN/'validate_lifecycle.py'),'profile40_on']),
  ('diagnostics',[py,str(RUN/'validate_diagnostics.py'),'profile40_on']),
  ('numerical',[py,str(ROOT/'validate_numerical_case.py'),'profile40_on']),
  ('oracle',[py,str(HERE/'oracle.py'),str(ROOT/'profile40_on_source/native_gaussian.bin'),str(HERE/'real_oracle.json')])]
(logs/'commands.json').write_text(json.dumps(commands,indent=2))
env=os.environ.copy();env['OPENBLAS_NUM_THREADS']='1'
try:
    for name,command in commands:
        with (logs/(name+'.log')).open('x') as f:
            r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,env=env)
        assert r.returncode==0,(name,r.returncode,str(logs/(name+'.log')))
        print(name.upper()+'_PASS',flush=True)
    o=json.loads((HERE/'real_oracle.json').read_text())
    assert o['retirements']>=100 and len(o['queries'])>=15
    assert o['queries'][0]['epoch']<50 and o['queries'][-1]['epoch']>700
    assert all(r['byte_identical'] for r in json.loads((ROOT/'profile40_on_validation.json').read_text())['repeat_files'].values())
    (logs/'complete.json').write_text(json.dumps(dict(passed=True)))
except BaseException as exc:
    (logs/'failed.txt').write_text(str(exc)+'\n');raise
