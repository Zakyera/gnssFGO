"""Saved-data-only admission to offline analysis. Never launches a source."""
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
C=P/'capture'
S=P.parent/'gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert (C/'capture_complete.json').exists() and (C/'playback_complete.json').exists()
assert not (C/'failed.txt').exists() and not (H/'controller_failed.json').exists()
cleanup=json.loads((C/'cleanup.json').read_text())
assert not cleanup['errors'] and cleanup['kimera_process_gone']
assert all(v==0 for v in cleanup['returncodes'].values())
restored=json.loads((H/'container_restoration.json').read_text())
assert 'true' not in restored['states']
python='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
parity=json.loads(subprocess.check_output([python,str(H/'monitor.py')],text=True))
assert parity['objective_exact'] and parity['native_exact'] and parity['last_sensor_s']>299
assert parity['native_rows']==parity['objective_rows']==1498
marker='KIMERA_CBS_REVISION_EDGE_ROW,'
def rows(p):return [l.split(marker,1)[1] for l in p.read_text().splitlines() if marker in l]
new=rows(C/'estimators.log');old=rows(S/'estimators.log')
assert len(new)==37100 and new==old[:len(new)],'MAIN mean/covariance payload parity'
targets=[800,934,1000,1335,1336,1337,1338]
assert sorted(int(p.name.split('_')[1]) for p in (C/'snapshots').iterdir())==targets
snapshots={}
for k in targets:
    d=C/'snapshots'/('update_'+str(k));assert (d/'complete.txt').exists()
    checks=dict(l.split(',',1) for l in (d/'roundtrip.txt').read_text().splitlines())
    assert float(checks['max_relative_linear_difference'])<1e-12
    snapshots[k]=dict(checks=checks,sha256=sha(d/'checkpoint.bin'),bytes=(d/'checkpoint.bin').stat().st_size)
pins=json.loads((H/'protected_before.json').read_text())
for p,s in pins.items():assert sha(p)==s,p
manifest=json.loads((P/'prepared_manifest.json').read_text())
for p,s in manifest.items():assert sha(P/p)==s,p
checkpoint=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
result=dict(passed=True,parity=parity,main_revision_rows_exact=len(new),snapshots=snapshots,
    protected_files_unchanged=len(pins),private_files_unchanged=len(manifest),hesai_checkpoint=checkpoint,
    source_runs=1,source_retries=0,source_duration_s=300,source_rate=.2,
    production_or_model_changes=False,new_accuracy_metrics=False,cleanup=cleanup)
with (H/'capture_verification.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
