import csv,hashlib,json,math
from pathlib import Path
ROOT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
SAVED=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_live60_20261011/prepared/live60_on_receiver/receiver')
def data(p):
    excluded={'publication_convergence.csv':{'seconds'},'revision_events.csv':{'update_seconds'},'stream_progress.csv':{'wall_unix_ns'}}
    if p.name in excluded:
        with p.open() as f:return [{k:v for k,v in r.items() if k not in excluded[p.name]} for r in csv.DictReader(f)]
    if p.name=='summary.json':return {k:v for k,v in json.loads(p.read_text()).items() if k not in ['wall_seconds','revision_update_seconds']}
    return hashlib.sha256(p.read_bytes()).hexdigest()
def comparison(a,b):return {p.name:data(p)==data(b/p.name) for p in SAVED.iterdir() if p.is_file() for p in [a/p.name]}
unpinned=comparison(ROOT/'reference_unpinned',SAVED)
pinned=comparison(ROOT/'reference_pinned',ROOT/'off1')
original_failure=comparison(ROOT/'off1',SAVED)
movement={}
for name in ['online_states.csv','states.csv']:
    with (SAVED/name).open() as f:a=list(csv.DictReader(f))
    with (ROOT/'off1'/name).open() as f:b=list(csv.DictReader(f))
    assert len(a)==len(b)
    movement[name]={}
    for label,fields in [('position',['px','py','pz']),('velocity',['vx','vy','vz']),('omega',['wx','wy','wz']),('quaternion',['qx','qy','qz','qw'])]:
        vals=[math.sqrt(sum((float(x[k])-float(y[k]))**2 for k in fields)) for x,y in zip(a,b)]
        movement[name][label]=dict(maximum=max(vals),rms=math.sqrt(sum(v*v for v in vals)/len(vals)))
payload=dict(passed=all(unpinned.values()) and all(pinned.values()),
    unpinned_reference_vs_saved=unpinned,pinned_reference_vs_cache_off=pinned,
    failed_original_comparison=original_failure,pinned_vs_unpinned_state_differences=movement,
    note='Exact gate unchanged. Pinned and unpinned oracles distinguished; original failure retained.')
with (ROOT/'environment_control.json').open('x') as f:json.dump(payload,f,indent=2)
print(json.dumps(payload,indent=2));assert payload['passed']
