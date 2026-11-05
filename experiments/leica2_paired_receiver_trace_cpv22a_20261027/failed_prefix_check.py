"""Preserve/check original CP-V22 observer-crash prefix without rerunning it."""
import csv,hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent
F=H.parent/'leica2_paired_receiver_trace_cpv22_20261027'
R=Path('/home/yeranis/results/gnssfgo_leica2_paired_receiver_trace_cpv22_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_full_cpk29_20261023/prepared/full_off_receiver/receiver')
result={'outputs':{},'input_sha256':{}}
def read(p):
    b=p.read_bytes();result['input_sha256'][str(p)]=hashlib.sha256(b).hexdigest();return b
for name in ('online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum'):
    partial=read(R/'hesai'/name);original=read(P/name)
    assert original.startswith(partial),name
    result['outputs'][name]={'exact_original_prefix':True,'lines':len(partial.splitlines())}
aa=list(csv.DictReader(read(R/'hesai/stream_progress.csv').decode().splitlines()))
bb=list(csv.DictReader(read(P/'stream_progress.csv').decode().splitlines()))
for i,(x,y) in enumerate(zip(aa,bb)):
    for k in x:
        if k not in ('processed_wall_ns','service_seconds','rss_kib'):assert x[k]==y[k],(i,k)
assert not (R/'velodyne').exists()
old=(F/'Diagnostics.h').read_text();new=(H/'Diagnostics.h').read_text()
assert new==old.replace('auto& files=AuditFiles(root);std::vector<Matrix> J;',
    'Require(bool(factor),"null numerical audit factor");\n  auto& files=AuditFiles(root);std::vector<Matrix> J(factor->keys().size());')
result.update(completed_event_rows_matching=len(aa),velodyne_not_started=True,observer_only_delta_verified=True,
              original_model_failure=False,cause='New observer passed an empty Jacobian vector to generic GTSAM NoiseModelFactor6; output vector must be pre-sized.')
with (H/'failed_prefix_check.json').open('x') as f:json.dump(result,f,indent=2)
print({k:v for k,v in result.items() if k!='input_sha256'})
