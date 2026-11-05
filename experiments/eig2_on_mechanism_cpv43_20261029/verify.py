"""Verify CP-V43 report inputs and arithmetic, without running an estimator."""
import datetime
import json
import sys
import numpy as np
from scipy.linalg import expm
sys.dont_write_bytecode=True
import analyze as a

assert not (a.H/'verification.json').exists()
main=json.loads((a.H/'analysis.json').read_text())
means=json.loads((a.H/'transport_means.json').read_text())
inputs={**main['input_sha256'],**means['input_sha256']}
for p,want in inputs.items():assert a.sha(p)==want,p
old=json.loads((a.H42/'complete.json').read_text())['output_sha256']
for p,want in old.items():assert a.sha(p)==want,p
rng=np.random.default_rng(43);maximum=0.0
for _ in range(100):
    x=rng.normal(size=6);x[:3]*=.2
    hat=np.zeros((4,4));hat[:3,:3]=a.skew(x[:3]);hat[:3,3]=x[3:]
    maximum=max(maximum,float(np.max(abs(a.exp(x)-expm(hat)))))
    assert np.max(abs(a.log(a.exp(x))-x))<1e-10
    G=a.exp(rng.normal(size=6));X=a.exp(rng.normal(size=6));Y=a.exp(rng.normal(size=6))
    assert np.max(abs(a.inv(G@X)@(G@Y)-a.inv(X)@Y))<1e-10
assert maximum<1e-10
events=json.loads((a.H/'edge_events.json').read_text())
assert len(events)==1744 and len({(x['k'],*x['receiver']) for x in events})==1744
assert len({x['k'] for x in events})==1735
assert main['modes']['on']['first_marginalization']['k']==26
assert main['modes']['on']['first_peer_retirement']['k']==30
assert main['modes']['off']['first_peer_retirement'] is None
assert not main['modes']['on']['recovery_events']
for k in [22,24,25]:
    e=next(x for x in events if x['k']==k)
    assert e['lifecycle']['marginal_keys']==e['lifecycle']['retiring_factors']==0
assert next(x for x in events if x['k']==1016)['total_cost_post']>5*next(x for x in events if x['k']==1016)['total_cost_pre']
outputs={str(p):a.sha(p) for p in a.H.iterdir() if p.is_file()}
a.save('verification.json',dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
    analyzed_inputs_unchanged=len(inputs),original_cpv42_outputs_unchanged=len(old),
    independent_SE3_matrix_exponential_cases=100,maximum_error=maximum,
    joins_checked=1744,unique_updates=1735,source_replays=0,optimizer_runs=0,
    covariance_analysis=False,estimator_changes=False,
    verification_scope='Analyzed inputs and all CP-V42 completed report outputs; not a repeat of the 6253-file campaign or four-sequence validation.',
    output_sha256=outputs))
print(json.dumps(dict(passed=True,analyzed_inputs=len(inputs),original_outputs=len(old),geometry_cases=100,maximum_error=maximum),indent=2))
