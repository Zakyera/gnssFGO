"""Inside Noetic container, compile once per named stage with archived commands."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
R=Path('/workspace/cbs_gtsam4.3/runs/leica2_rotation_weight_cpv6_20261024')
stage=sys.argv[1];assert stage in ('control','candidate')
assert not (R/(stage+'_result.json')).exists()
with (R/(stage+'.log')).open('x') as log:
 started=time.time()
 for cmd in json.loads((R/(stage+'_commands.json')).read_text()):
  subprocess.run(cmd,cwd='/workspace/cbs_gtsam4.3/build/cbs',stdout=log,stderr=subprocess.STDOUT,check=True)
out=R/('control.so' if stage=='control' else 'libcbs.so')
digest=hashlib.sha256(out.read_bytes()).hexdigest()
if stage=='control':assert digest=='82f85c2a307a9d9a849b2b8f09bb5c1e026ad80a4ef0a633c903d30cca712476',digest
result=dict(passed=True,stage=stage,sha256=digest,wall_seconds=time.time()-started)
with (R/(stage+'_result.json')).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
