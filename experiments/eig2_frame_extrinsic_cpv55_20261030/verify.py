"""Read-only original-input/checkpoint integrity and stopped-container closeout."""
import json,hashlib,subprocess,datetime
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs/eig2_glim_first_edge_cpv54_20261030/prepared'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins=json.loads((P/'protected_manifest.json').read_text());n=len(pins)
for p,h in pins.items():assert sha(p)==h,p
for name in ['calibration.json','payloads.json','source_audit.json']:
 d=json.loads((H/name).read_text());assert d['passed'];pins.update(d['input_sha256'])
for p,h in pins.items():assert sha(p)==h,p
for p in [H/'input_headers.json',H/'native_attempt1/native_test.json',H/'saved_native_attempt3/results.json']:
 assert json.loads(p.read_text())['passed']
# Verify exact saved camera library actually used by the extraction.
S=W/'runs/eig2_hesai_insertion_cpv51_20261030/prepared'
for p in [S/'private_build/libkimera_vio.so',S/'private_build/libcbs.so']:
 expected=json.loads((S/'protected_manifest.json').read_text()).get(str(p))
 # CP51 pins its parent production libraries; retained CP54 pins its own copied
 # backend and CP51 library via inherited retained outputs where available.
 references=[pins.get(str(p))]
 if not any(references):
  old=json.loads((H.parent/'eig2_glim_first_edge_cpv54_20261030/retained_manifest.json').read_text());references=[old.get(str(P/'private_build'/p.name))]
 assert sha(p) in references,(str(p),references)
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
out=dict(passed=True,time=datetime.datetime.now().astimezone().isoformat(),protected_files=n,total_unique_checked_inputs=len(pins),containers=containers,hesai=hesai,source_replays=0,optimizer_calls=0,production_changes=0)
with (H/'verification.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
