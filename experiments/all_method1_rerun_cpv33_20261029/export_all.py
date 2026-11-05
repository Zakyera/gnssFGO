"""Finite visualization export/check job, never launches an estimator."""
import json
import os
from pathlib import Path
import subprocess

H = Path(__file__).resolve().parent
PY = '/home/yeranis/.local/share/pipx/venvs/rerun-sdk/bin/python'
env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
inputs = json.loads((H/'input_checks.json').read_text())
assert inputs['passed']
for case in inputs['cases']:
    with (H/(case+'_export.log')).open('x') as log:
        subprocess.run([PY,str(H/'build.py'),case],env=env,stdout=log,
                       stderr=subprocess.STDOUT,check=True)
    print('Exported '+case,flush=True)
with (H/'verify.log').open('x') as log:
    subprocess.run([PY,str(H/'verify.py')],env=env,stdout=log,
                   stderr=subprocess.STDOUT,check=True)
print('All seven RRDs passed exact geometry and input-hash readback.',flush=True)

