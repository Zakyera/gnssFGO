"""Run inside the existing build container. Inspect DSO only; no ROS launch."""
import json
import os
from pathlib import Path
import subprocess
R=Path('/workspace/cbs_gtsam4.3/runs/kimera_incremental_prior_cpk9_20261019')
out=R/'backend_initial'
assert json.loads((out/'passed.json').read_text())['passed']
env=dict(os.environ)
env['LD_LIBRARY_PATH']=str(R/'private_build')+':/workspace/cbs_gtsam4.3/devel/lib:'+env.get('LD_LIBRARY_PATH','')
p=subprocess.run(['ldd','-r',str(R/'private_build/libkimera_vio.so')],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
with (out/'dependencies.log').open('x') as f:f.write(p.stdout)
assert p.returncode==0 and 'not found' not in p.stdout and 'undefined symbol:' not in p.stdout,p.stdout
symbols=subprocess.check_output(['strings',str(R/'private_build/libkimera_vio.so')],text=True)
assert 'CPK9_PRIOR_REFRESH' in symbols and 'CPK9 unexplained fixed key' in symbols
print(json.dumps({'candidate_link_inspection_passed':True,'estimator_launched':False}))
