"""CP-V16A final evidence checks; does not run or modify an estimator."""
import csv, hashlib, json, shutil
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_motion_prior_tenth_cpv16a_20261026')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
PREV=H.parent/'leica2_receiver_trace_cpv13_20261026'

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()

assert not (H/'closeout.json').exists()
assert not (H/'failed.txt').exists()
pins=json.loads((H/'input_pins.json').read_text())
pins.update(json.loads((H/'scoring_pins.json').read_text()))
previous=json.loads((PREV/'closeout.json').read_text())
for mode in ['off','on']:
    path=PREV/f'{mode}_analysis.json'
    pins[str(path)]=previous['artifact_sha256'][str(path)]
for path,expected in pins.items():assert sha(path)==expected,path
complete=json.loads((H/'replay_complete.json').read_text());assert complete['pass_']
audit=json.loads((H/'model_audit.json').read_text());assert audit['returncode']==0 and json.loads(audit['stdout'])['pass']
baseline=(PREV/'Receiver.h').read_text()
assert (H/'Receiver.h').read_text()==baseline.replace(
    'return noiseModel::Diagonal::Variances(q);','return noiseModel::Diagonal::Variances(0.1*q);')
for name in ['Live.cpp','Trace.h']:assert sha(H/name)==sha(PREV/name)
assert sha(R/'live_trace')==json.loads((H/'binary.json').read_text())['sha256']
containers=json.loads((H/'container_closeout.json').read_text())
assert containers['owned_controller_exit_code']==0
assert not containers['owned_receiver_build_or_validation_jobs_present_before_stop']
assert all(not x['Running'] and x['Pid']==0 and not x['OOMKilled'] for x in containers['containers'].values())
results=json.loads((H/'results.json').read_text())
assert results['not_promoted']
assert results['scientific_pass']==all(all(g.values()) for g in results['scientific_gates'].values())
assert results['control_qc_half_reproduced'] and results['control_qc_quarter_reproduced']
csv_path_checks={};output_pins={}
TP=np.array([.308837802,-.038312997,-.234944116])
for mode in ['off','on']:
    gate=json.loads((H/f'{mode}_validation.json').read_text());assert gate['pass_']
    assert json.loads((H/f'{mode}_finish.json').read_text())['returncode']==0
    for product in ['online','finalized']:
        path=R/mode/f'{product}_states.csv'
        with open(path) as f:rows=list(csv.DictReader(f))
        with open(P/f'full_{mode}_receiver'/'receiver'/path.name) as f:old=list(csv.DictReader(f))
        assert len(rows)==len(old)==4282
        for i,(r,b) in enumerate(zip(rows,old)):
            assert int(r['state_index'])==i
            assert all(r[k]==b[k] for k in ['state_index','timestamp_ns','availability_ns'])
            assert all(np.isfinite(float(x)) for x in r.values())
        p=np.array([[float(r[k]) for k in ['px','py','pz']] for r in rows])
        q=np.array([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in rows])
        prism=p+Rotation.from_quat(q).apply(TP)
        length=float(np.linalg.norm(np.diff(prism,axis=0),axis=1).sum())
        expected=results['full'][f'{mode}_qc_tenth_{product}']['path_m']
        assert abs(length-expected)<1e-8,(mode,product,length,expected)
        csv_path_checks[mode+'_'+product]=dict(independent_CSV_path_m=length,difference_from_TUM_score_m=abs(length-expected))
        for p in [path,R/mode/f'{product}_trajectory.tum']:output_pins[str(p)]=sha(p)
    for p in [R/mode/'summary.json',R/mode/'stream_progress.csv',R/f'{mode}_trace'/'trace_meta.csv',R/f'{mode}_trace'/'trace_states.csv']:
        output_pins[str(p)]=sha(p)
    # Verify exact observer pair identities, not merely a numerical summary.
    with open(R/f'{mode}_trace'/'trace_meta.csv') as f:meta=list(csv.DictReader(f))
    assert [int(x['snapshot']) for x in meta]==list(range(1,len(meta)+1))
    for i,x in enumerate(meta):
        if not x['stage'].endswith('_before'):continue
        y=meta[i+1];assert y['stage']==x['stage'].replace('_before','_after')
        for k in ['event','arrival_ns','source','action','from','to','revision','start_ns','end_ns']:
            assert x[k]==y[k],(mode,i,k)

artifacts={str(p):sha(p) for p in H.iterdir() if p.is_file() and p.name!='closeout.json'}
receipt=dict(pass_=True,scientific_pass=results['scientific_pass'],decision='test_completed_not_promoted',
    original_and_analysis_pins_verified=len(pins),replay_complete=complete,
    source_capture=False,one_model_change='Qc covariance multiplier0.1',
    source_payload_action_and_timing_conservation=True,original_live_adapter_unchanged=True,
    no_source_covariance_change=True,containers_restored=True,figures_visually_checked=['comparison.png'],
    csv_path_checks=csv_path_checks,no_stationarity_or_prior_conditioning_certificate=True,
    remaining_free_gib=shutil.disk_usage(R).free/2**30,
    artifact_sha256=artifacts,candidate_output_sha256=output_pins)
with open(H/'closeout.json','x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if k not in ['artifact_sha256','candidate_output_sha256']},indent=2))
