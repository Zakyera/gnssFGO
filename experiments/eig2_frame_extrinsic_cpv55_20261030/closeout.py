"""Seal the completed diagnostic report without modifying old evidence."""
from pathlib import Path
import json,hashlib,datetime,subprocess
H=Path(__file__).resolve().parent;K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
assert json.loads((H/'verification.json').read_text())['passed']
assert json.loads((H/'native_attempt1/native_test.json').read_text())['passed']
assert json.loads((H/'saved_native_attempt3/results.json').read_text())['passed']
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
files={}
for root in [H,K]:
 for p in root.rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts:
   with p.open('rb') as f:files[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
with (H/'retained_manifest.json').open('x') as f:json.dump(files,f,indent=2)
out=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),scope='Completed bounded EIG-2 bidirectional spatial-frame audit, independent calibration comparison and standalone native tests; no root-cause/fix claim',report='REPORT.txt',verification='verification.json',retained_files=len(files),containers=states,production_promoted=False,source_captures=0,weight_changes=0,limits=['Physical author-calibration accuracy not independently reestimated.','No historical native GLIM final inserted-factor matrix observation.','Not a blanket historical source/binary equivalence or full timing/ordering proof.'])
with (H/'complete.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
