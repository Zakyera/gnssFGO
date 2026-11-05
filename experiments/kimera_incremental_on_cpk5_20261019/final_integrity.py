"""Read-only preservation checks and final owned-container status receipt."""
import hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_incremental_on_cpk5_20261019')
def host(p):return p.replace('/workspace/cbs_gtsam4.3','/home/yeranis/repos/V4RL/cbs_gtsam4.3')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
assert not (H/'integrity.json').exists(),'Refuse overwrite'
protected=json.loads((R/'protected_before.json').read_text())
assert all(sha(p)==h for p,h in protected.items())
manifest=json.loads((R/'pre_capture_receipt.json').read_text())['manifest']
assert all(sha(host(p))==h for p,h in manifest.items())
assert json.loads((R/'on_capture/cleanup.json').read_text())['kimera_process_gone']
containers=json.loads(subprocess.check_output(['docker','inspect','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True))
assert all(not c['State']['Running'] for c in containers)
out=dict(passed=True,protected_unchanged=len(protected),prepared_manifest_unchanged=len(manifest),
         containers={c['Name']:c['State']['Status'] for c in containers},
         result_manifest={str(p):sha(p) for p in (R/'on_capture').iterdir() if p.is_file()},
         scripts={str(p):sha(p) for p in H.iterdir() if p.suffix=='.py'},
         source_playbacks=1,production_edits=False,builds=0,tuning=False)
with (H/'integrity.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('result_manifest','scripts')},indent=2))
