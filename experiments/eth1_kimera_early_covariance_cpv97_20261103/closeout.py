"""Supplementary source-contract clarification and immutable output checks."""
from pathlib import Path
import hashlib,json
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared'
assert not (H/'closeout.json').exists()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(n,o):
    with (H/n).open('x') as f:json.dump(o,f,indent=2);f.write('\n')
v=json.loads((H/'verification.json').read_text())
for p,h in v['output_sha256'].items():assert sha(p)==h,p
for p,h in json.loads((H/'inputs_sha256.json').read_text()).items():assert sha(p)==h,p
d=json.loads((H/'analysis.json').read_text())
manifest=json.loads((P/'protected_manifest.json').read_text())
checks={}
for rel in ['src/cbs/src/bpsam/bpsam.cpp','runs/retention_readiness_cpv73_20261031/VioBackend.cpp']:
    p=str(W/rel);assert sha(p)==manifest[p];checks[p]=manifest[p]
counts={}
for lab in ('old','new'):
    s=d[lab]['samples']['1']
    counts[lab]=sum(g['count'] for g in s['global_groups'].values() if g['category']!='cbs_external')
assert counts==dict(old=878,new=937)
save('source_review.json',dict(passed=True,pinned_sources=checks,
    clarification='The pinned source increments candidate_factor_count only for nonnull retained factors, then commits marginalization_graph_factor_count_. The conservative empty-slot warning in analysis.json is resolved: these are live LOCAL factor counts.',
    matched_sample1_live_counts=counts,
    current_frontend_source_not_individually_manifest_pinned=True,
    actual_frontend_bias_use_and_complete_smart_measurements_unavailable=True,
    additional_inputs_sha256={str(p):sha(p) for p in [P/'protected_manifest.json',
        P/'full_on_source/kimera_profile/BackendParams.yaml',Path(__file__),H/'REPORT.txt']}))
files=['PROTOCOL.txt','analyze.py','analysis.json','inputs_sha256.json','verification.json',
       'REPORT.txt','source_review.json','closeout.py']
save('closeout.json',dict(passed=True,source_runs=0,optimizer_runs=0,receiver_runs=0,
    old_terminal_outputs_verified=221,new_terminal_outputs_verified=426,
    source_review_only_not_causal_ablation=True,
    output_sha256={str(H/n):sha(H/n) for n in files}))
print('CP97 saved-data audit closeout passed; no source, receiver or optimizer run.')
