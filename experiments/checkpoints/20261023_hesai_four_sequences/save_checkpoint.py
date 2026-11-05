"""Copy compact completed-run evidence; retain large immutable captures in place.
Exclusive creation. Never launches a run, validation, evaluation, or git command.
"""
from pathlib import Path
import hashlib,json,shutil

H=Path(__file__).resolve().parent
E=H.parent.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
SPEC=[('ETH1','eth1_awake_full_cpk23_20261021','gnssfgo_eth1_awake_full_cpk23_20261021','results.json'),
 ('EIG-2','eig2_full_cpk24_20261021','gnssfgo_eig2_full_cpk24_20261021','results_eof_review.json'),
 ('SPX-3','spx3_stationary_full_cpk28_20261023','gnssfgo_spx3_stationary_full_cpk28_20261023','results.json'),
 ('LEICA-2','leica2_full_cpk29_20261023','gnssfgo_leica2_full_cpk29_20261023','results.json')]
assert not (H/'manifest.json').exists() and not (H/'snapshot').exists()
files={};cases={}
def keep(p,rel):
    if not p.is_file():return
    before=p.stat();assert before.st_size<32*1024**2,p
    target=H/'snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
    with p.open('rb') as src,target.open('xb') as dst:shutil.copyfileobj(src,dst)
    with target.open('rb') as f:sha=hashlib.file_digest(f,'sha256').hexdigest()
    with p.open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==sha
    assert (before.st_size,before.st_mtime_ns)==(p.stat().st_size,p.stat().st_mtime_ns)
    files[str(target.relative_to(H))]=dict(original=str(p),bytes=before.st_size,sha256=sha)
for seq,exp,run,result in SPEC:
    src=E/exp;prepared=W/'runs'/run/'prepared'
    assert json.loads((src/result).read_text())['passed']
    close=json.loads((src/'closeout.json').read_text());assert close['passed']
    cases[seq]=dict(experiment=str(src),prepared=str(prepared),result=str(src/result),
        data_roots=[str(prepared/f'full_{m}_{kind}') for m in ['off','on'] for kind in ['source','receiver','control']],
        note='Large journals, bags, binary dependencies, maps and full logs retained at original paths; not copied.')
    for p in src.iterdir():
        if p.suffix in ['.txt','.json','.py','.diff']:keep(p,Path(seq)/'reporting'/p.name)
    for p in (src/'eof_review').rglob('*'):
        if p.suffix in ['.txt','.json','.py']:keep(p,Path(seq)/'reporting/eof_review'/p.relative_to(src/'eof_review'))
    for p in prepared.iterdir():
        if p.suffix in ['.json','.py','.sh','.env','.launch','.txt']:keep(p,Path(seq)/'prepared'/p.name)
    for sub in ['input_profiles','transport_tools']:
        for p in (prepared/sub).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts:keep(p,Path(seq)/'prepared'/p.relative_to(prepared))
    for mode in ['off','on']:
        s=prepared/f'full_{mode}_source';r=prepared/f'full_{mode}_receiver/receiver'
        for name in ['effective_mode_check.json','candidate_validation.json','source_coverage.json',
                     'local_provenance.json','run_metadata.txt','resolved_glim_params.yaml',
                     'resolved_kimera_params.yaml','glim_odometry.csv','kimera_odometry.csv']:
            keep(s/name,Path(seq)/mode/'source'/name)
        for sub in ['glim_config','kimera_profile']:
            for p in (s/sub).rglob('*'):
                if p.is_file():keep(p,Path(seq)/mode/'source'/p.relative_to(s))
        for name in ['summary.json','online_trajectory.tum','finalized_trajectory.tum']:
            keep(r/name,Path(seq)/mode/'receiver'/name)
        keep(prepared/f'full_{mode}_control/complete.json',Path(seq)/mode/'capture_complete.json')
for name in ['PLAN.txt','REPORT.txt','analyze.py','timing_v2.json','audit_spikes.py','audit_spikes.json']:
    keep(E/'four_sequence_timing_cpt1_20261023'/name,Path('timing')/name)
with (H/'manifest.json').open('x') as f:
    json.dump(dict(status='saved_local_checkpoint',cases=cases,files=files,
        bytes=sum(x['bytes'] for x in files.values()),scope='Compact copy plus integrity manifest; not offsite/full binary/data backup'),f,indent=2)
print(json.dumps(dict(cases=len(cases),files=len(files),megabytes=sum(x['bytes'] for x in files.values())/1e6)))
