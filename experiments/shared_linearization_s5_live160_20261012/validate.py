"""Frozen functional/receiver checks plus ordinary publication provenance."""
from pathlib import Path
import json,os,subprocess
from prepare import RUN,ROOT,HERE,inside
assert (ROOT/'capacity160_on_control/complete.json').exists()
assert not (ROOT/'capacity160_on_control/failed.txt').exists()
logs=RUN/'validation_logs';logs.mkdir(exist_ok=False)
py='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
setup='source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
commands=[('native',['docker','exec','cbsms_ws_gpu','bash','-lc',setup+
    f'python3 {inside(RUN)}/validate_native.py --capture {inside(RUN)} --out {inside(ROOT)}/capacity160_on_native_validation.json']),
    ('lifecycle',[py,str(RUN/'validate_lifecycle.py'),'capacity160_on']),
    ('diagnostics',[py,str(RUN/'validate_diagnostics.py'),'capacity160_on']),
    ('provider',['docker','exec','cbsms_ws_gpu','bash','-lc',setup+
       f'python3 {inside(RUN)}/validate_provider.py {inside(RUN)} {inside(ROOT)}/capacity160_on_provider_validation.json']),
    ('numerical',[py,str(ROOT/'validate_numerical_case.py'),'capacity160_on'])]
(logs/'commands.json').write_text(json.dumps(commands,indent=2))
env=os.environ.copy();env['OPENBLAS_NUM_THREADS']='1'
try:
    for name,command in commands:
        with (logs/(name+'.log')).open('x') as f:r=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,env=env)
        assert r.returncode==0,(name,r.returncode,str(logs/(name+'.log')))
        print(name.upper()+'_PASS',flush=True)
    assert all(r['byte_identical'] for r in json.loads((ROOT/'capacity160_on_validation.json').read_text())['repeat_files'].values())
    (logs/'complete.json').write_text(json.dumps(dict(passed=True)))
except BaseException as exc:
    (logs/'failed.txt').write_text(str(exc)+'\n');raise
