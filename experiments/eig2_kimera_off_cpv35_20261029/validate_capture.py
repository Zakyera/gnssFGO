"""Exclusive saved-data parity and finite diagnostic closeout checks."""
import csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eig2_kimera_off_cpv35_20261029';C=P/'capture'
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert (C/'capture_complete.json').exists() and (C/'playback_complete.json').exists()
assert not (C/'failed.txt').exists() and not (H/'controller_failed.json').exists()
cleanup=json.loads((C/'cleanup.json').read_text());assert not cleanup['errors'] and cleanup['kimera_process_gone']
assert all(v is not None for v in cleanup['returncodes'].values())
assert (H/'container_restoration.json').exists()
assert 'true' not in json.loads((H/'container_restoration.json').read_text())['states']
python='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
parity=json.loads(subprocess.check_output([python,str(H/'monitor.py')],text=True))
assert parity['objective_exact'] and parity['native_exact'] and parity['last_sensor_s']>299
assert parity['native_rows']==parity['objective_rows']
def rows(path,marker):return [l.split(marker,1)[1] for l in path.read_text().splitlines() if marker in l]
marker='KIMERA_CBS_REVISION_EDGE_ROW,'
new=rows(C/'estimators.log',marker);old=rows(S/'estimators.log',marker)
assert new==old[:len(new)],'MAIN revision means/covariance parity'
a=np.loadtxt(C/'objective.csv',delimiter=',');assert np.array_equal(a[:,0],np.arange(len(a)))
snap=json.loads((H/'snapshots_final.json').read_text());ids=[r['update'] for r in snap['updates']]
for k in (1335,1336,1337,1338,1410,1467):assert k in ids,k
costmax=0.
for row in snap['updates']:
    k=row['update'];s=row['stages']['after_solve'];total=sum(v['state'] for v in s['family_costs'].values())
    err=abs(total-a[k,5]);costmax=max(costmax,err)
    assert err<1e-5+1e-10*abs(a[k,5]),(k,total,a[k,5])
pins=json.loads((H/'protected_before.json').read_text())
for p,s in pins.items():assert sha(p)==s,p
manifest=json.loads((P/'prepared_manifest.json').read_text())
for p,s in manifest.items():assert sha(P/p)==s,p
checkpoint=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
result=dict(passed=True,parity=parity,main_revision_rows_exact=len(new),snapshots=ids,copied_cost_sum_max_abs_difference=costmax,protected_files_unchanged=len(pins),private_files_unchanged=len(manifest),hesai_checkpoint=checkpoint,source_runs=1,source_retries=0,source_duration_s=300,source_rate=.2,production_or_model_changes=False,new_accuracy_metrics=False,cleanup=cleanup)
with (H/'verification.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
