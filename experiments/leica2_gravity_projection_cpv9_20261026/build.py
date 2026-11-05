"""Inside Noetic: one compile per stage, isolated outputs, no install."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
R=Path('/workspace/cbs_gtsam4.3/runs/leica2_gravity_projection_cpv9_20261026')
stage=sys.argv[1];assert stage in ('control','candidate')
attempt=sys.argv[2] if len(sys.argv)>2 else '1'
result_name=stage+'_result.json' if attempt=='1' else stage+'_attempt'+attempt+'_result.json'
assert not (R/result_name).exists()
with (R/(stage+'_attempt'+attempt+'.log')).open('x') as log:
 started=time.time()
 for cmd in json.loads((R/(stage+'_commands.json')).read_text()):
  subprocess.run(cmd,cwd='/workspace/cbs_gtsam4.3/build/cbs',stdout=log,stderr=subprocess.STDOUT,check=True)
out=R/('control.so' if stage=='control' else 'libcbs.so')
digest=hashlib.sha256(out.read_bytes()).hexdigest()
if stage=='control':assert digest=='82f85c2a307a9d9a849b2b8f09bb5c1e026ad80a4ef0a633c903d30cca712476',digest
result=dict(passed=True,stage=stage,sha256=digest,wall_seconds=time.time()-started)
with (R/result_name).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
