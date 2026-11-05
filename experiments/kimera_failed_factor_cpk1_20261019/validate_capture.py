"""One retained-output validation; no estimator, no scoring or tuning."""
import csv,io,json,math
from collections import defaultdict
from pathlib import Path
import numpy as np
import rosbag,yaml
R=Path('/workspace/cbs_gtsam4.3/runs/kimera_failed_factor_cpk1_20261019')
U=R.parent/'fixed_lag_stopping_fl4u_20261017/off_capture'
D=R/'off_capture'
assert (D/'diagnostic_captured.json').exists() and not (D/'failed.txt').exists()
assert not (D/'validation.json').exists()
def messages(path):
    out=[]
    with rosbag.Bag(str(path)) as bag:
        for _,m,_ in bag.read_messages(topics=['/kimera_vio_ros/odometry']):
            b=io.BytesIO();m.serialize(b);out.append((m.header.stamp.to_nsec(),b.getvalue()))
    return out
actual,reference=messages(D/'outputs.bag'),messages(U/'outputs.bag')
a,b=[np.loadtxt(p/'acceptance.csv',delimiter=',') for p in (D,U)]
obj,oldobj=[np.loadtxt(p/'objective.csv',delimiter=',') for p in (D,U)]
cols=[0,1,2,3,4,5,7,8]
parity=dict(count=len(actual),reference_count=len(reference),native_byte_identical=actual==reference,
    acceptance_exact=bool(a.shape==b.shape and np.array_equal(a[:,cols],b[:,cols])),
    objective_exact=bool(obj.shape==oldobj.shape and np.array_equal(obj,oldobj)))
assert len(actual)==len(reference)==531 and all(parity[k] for k in ('native_byte_identical','acceptance_exact','objective_exact')),parity
params=yaml.safe_load((D/'resolved_params.yaml').read_text())
assert params['cbs_belief_receive_enable'] is False and params['cbs_belief_publish_enable'] is True
assert json.loads((D/'preflight.json').read_text())['passed']
def rows(path):
    out=defaultdict(list)
    for r in csv.reader(path.open()):out[r[0]].append(r[1:])
    return out
base,previous=rows(D/'failed_decision.txt'),rows(U/'failed_decision.txt')
assert base==previous,'Original failed decision/probes changed'
new=rows(D/'factor_derivatives.txt')
assert new['done']==[['1']]
decision=[float(x) for x in new['decision'][0]]
assert decision[0]==531 and new['repeat'][0]==new['decision'][0][1:2]
factors=[]
for r in new['factor']:
    f=dict(slot=int(r[0]),family=r[1],keys=int(r[2]),cost=float(r[3]),analytic=float(r[4]),status=int(r[5]),
        fd=[float(r[i]) for i in (6,9,12)],validity=[int(r[i]) for i in (7,8,10,11,13,14)])
    f['gap']=f['fd'][1]-f['analytic']
    if f['family']=='smart_stereo':
        f.update(fresh_cost=float(r[15]),fresh_fd=float(r[16]),fresh_validity=[int(r[i]) for i in (17,18,19)])
    assert all(math.isfinite(f[k]) for k in ('cost','analytic','gap'))
    factors.append(f)
families={}
for name in sorted({f['family'] for f in factors}):
    subset=[f for f in factors if f['family']==name]
    families[name]=dict(count=len(subset),cost=sum(f['cost'] for f in subset),analytic=sum(f['analytic'] for f in subset),
        fd=[sum(f['fd'][j] for f in subset) for j in range(3)],gap=sum(f['gap'] for f in subset),
        sum_absolute_gap=sum(abs(f['gap']) for f in subset),
        validity_changes=sum(any(s!=f['status'] for s in f['validity']) for f in subset))
    if name=='smart_stereo':families[name].update(fresh_cost=sum(f['fresh_cost'] for f in subset),fresh_fd=sum(f['fresh_fd'] for f in subset))
sums=[float(x) for x in new['sums'][0]]
assert abs(sums[0]-decision[1])<1e-9 and abs(sums[1]-decision[2])<1e-8
original_derivative=float(base['direction'][0][0])
assert abs(sums[1]-original_derivative)<1e-8
log=(D/'kimera.log').read_text();assert log.count('FL4Q no decreasing current-chart LM step')==1
assert not any(s in log for s in ('CPK1','FL4U','Smoother recovery','Persistent BPSAM recovery'))
report=dict(passed=True,parity=parity,original_failure_preserved=True,original_probe_identical=True,
    native_span_s=(actual[-1][0]-actual[0][0])*1e-9,decision=decision,sums=sums,families=families,
    largest_gap_factors=sorted(factors,key=lambda f:abs(f['gap']),reverse=True)[:25],
    validities_changed=sum(any(s!=f['status'] for s in f['validity']) for f in factors),
    acceptance_changed=False,solver_changed=False)
with (D/'validation.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
