"""Final read-only evidence checks; write one new closeout receipt."""
import csv, hashlib, json, shutil
from pathlib import Path
import numpy as np

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_receiver_trace_cpv13_20261026')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(2**20),b''): h.update(b)
    return h.hexdigest()

pins=json.loads((H/'input_pins.json').read_text())
for mode in ['off','on']: pins.update(json.loads((H/f'{mode}_analysis_pins.json').read_text()))
for path,expected in pins.items(): assert sha(path)==expected,path
for mode in ['off','on']:
    parity=json.loads((H/f'{mode}_parity.json').read_text());assert parity['pass_']
    for name,expected in parity['byte_identical_outputs'].items():
        assert sha(R/mode/name)==sha(P/f'full_{mode}_receiver'/'receiver'/name)==expected
    analysis=json.loads((H/f'{mode}_analysis.json').read_text())
    assert analysis['checks']['cycles']==100
    assert analysis['checks']['publication_vs_original_max_m']==0
    assert analysis['checks']['cycle_vector_identity_max_m']==0
    cycles=json.loads((H/f'{mode}_cycles.json').read_text())
    total=0.;components=set(cycles[0]['vectors'])-{'prediction','support_add_adjustment'}
    for c in cycles:
        v=sum((np.array(c['vectors'][g]) for g in components),np.zeros(3))
        assert np.linalg.norm(v-np.array(c['actual']))<1e-10
        total+=np.linalg.norm(c['actual'])
    old=json.loads((H.parent/'leica2_fusion_jitter_cpv12_20261026/results.json').read_text())
    assert abs(total-old['modes'][mode]['selected']['online']['path_m'])<1e-10
    # Fresh chronological observer pairs, each exact event/source identity.
    with open(R/f'{mode}_trace'/'trace_meta.csv') as f: rows=list(csv.DictReader(f))
    assert [int(x['snapshot']) for x in rows]==list(range(1,len(rows)+1))
    for i,x in enumerate(rows):
        if x['stage'].endswith('_before'):
            y=rows[i+1];assert y['stage']==x['stage'].replace('_before','_after')
            for key in ['event','arrival_ns','source','action','from','to','revision','start_ns','end_ns']:
                assert x[key]==y[key]
    assert len(rows)==analysis['snapshots']
complete=json.loads((H/'replay_complete.json').read_text());assert complete['pass_']
container=json.loads((H/'container_closeout.json').read_text())
assert all(not x['Running'] and x['Pid']==0 for x in container['containers'].values())
assert sha(H/'Live.cpp')==sha(H.parent/'receiver_finalization_cpk19_20261021/Live.cpp')
assert sha(R/'live_trace')==json.loads((H/'binary.json').read_text())['sha256']
artifacts={str(p):sha(p) for p in H.iterdir() if p.is_file() and p.name!='closeout.json'}
result=dict(pass_=True,original_and_analysis_pins_verified=len(pins),replay_complete=complete,
            source_capture=False,model_weights_changed=False,original_output_exact=True,
            original_live_adapter_unchanged=True,containers_restored=True,
            figures_visually_checked=['update_trace.png','state_age.png'],
            no_stationarity_or_prior_conditioning_certificate=True,
            remaining_free_gib=shutil.disk_usage(R).free/2**30,artifact_sha256=artifacts)
with open(H/'closeout.json','x') as f: json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='artifact_sha256'},indent=2))
