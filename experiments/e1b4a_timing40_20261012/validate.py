"""Invoke unchanged completed-capture validators once; stop on any failure."""
from pathlib import Path
import json
import subprocess
import sys

HERE=Path(__file__).resolve().parent
RUN=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1b4a_timing40_20261012')
ROOT=RUN/'prepared'
INSIDE='/workspace/cbs_gtsam4.3/runs/'+RUN.name
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
def main():
    assert (ROOT/'profile40_on_control/complete.json').is_file()
    assert not (ROOT/'profile40_on_control/failed.txt').exists()
    output=RUN/'validation_logs';output.mkdir(exist_ok=False)
    commands=[('native',['docker','exec','cbsms_ws_gpu','bash','-lc',
      'source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && '
      f'python3 {INSIDE}/validate_native.py --capture {INSIDE} --out {INSIDE}/prepared/profile40_on_native_validation.json']),
      ('lifecycle',[PY,str(RUN/'validate_lifecycle.py'),'profile40_on']),
      ('diagnostics',[PY,str(RUN/'validate_diagnostics.py'),'profile40_on']),
      ('numerical',[PY,str(ROOT/'validate_numerical_case.py'),'profile40_on']),
      ('summary',[sys.executable,str(HERE/'summarize.py')])]
    with (output/'commands.json').open('x') as f:json.dump(commands,f,indent=2)
    try:
        for name,command in commands:
            with (output/(name+'.log')).open('x') as log:
                result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            assert result.returncode==0,(name,result.returncode,str(output/(name+'.log')))
            print(name.upper()+'_PASS',flush=True)
        with (output/'complete.json').open('x') as f:json.dump(dict(passed=True),f)
    except BaseException as exc:
        with (output/'failed.txt').open('x') as f:f.write(str(exc)+'\n')
        raise
if __name__=='__main__':main()
