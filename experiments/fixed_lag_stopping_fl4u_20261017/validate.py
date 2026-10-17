"""One read-only parity and failed-decision analysis; never retries estimation."""
import io,json,re
from pathlib import Path
import numpy as np
import rosbag,yaml
R=Path(__file__).resolve().parent
D=R/'off_capture'
T=R.parent/'gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared/live120_off_source'
assert (D/'diagnostic_captured.json').exists() and not (D/'failed.txt').exists()
assert not (D/'validation.json').exists()
def messages(path):
    result=[]
    with rosbag.Bag(str(path)) as bag:
        for _,m,_ in bag.read_messages(topics=['/kimera_vio_ros/odometry']):
            b=io.BytesIO();m.serialize(b)
            result.append((m.header.stamp.to_nsec(),b.getvalue()))
    return result
actual=messages(D/'outputs.bag');reference=messages(T/'recordings/cbs.bag')
a=np.loadtxt(D/'acceptance.csv',delimiter=',');b=np.loadtxt(T/'acceptance.csv',delimiter=',')
obj=np.loadtxt(D/'objective.csv',delimiter=',');oldobj=np.loadtxt(T/'objective.csv',delimiter=',')
cols=[0,1,2,3,4,5,7,8]
parity=dict(native_count=len(actual),reference_count=len(reference),
    all_native_messages_byte_identical=actual==reference,
    all_nontiming_acceptance_exact=bool(a.shape==b.shape and np.array_equal(a[:,cols],b[:,cols])),
    all_objective_rows_exact=bool(obj.shape==oldobj.shape and np.array_equal(obj,oldobj)))
with (D/'parity.json').open('x') as f:json.dump(parity,f,indent=2)
assert len(actual)==len(reference)==531 and actual==reference,parity
assert parity['all_nontiming_acceptance_exact'] and parity['all_objective_rows_exact'],parity
assert np.array_equal(a[:,0],np.arange(531)) and np.all(np.isfinite(a)) and np.all(np.isfinite(obj))
params=yaml.safe_load((D/'resolved_params.yaml').read_text())
assert params['cbs_belief_receive_enable'] is False
assert params['cbs_belief_publish_enable'] is True
assert json.loads((D/'preflight.json').read_text())['passed']
rows={}
for line in (D/'failed_decision.txt').read_text().splitlines():
    v=line.split(',');rows.setdefault(v[0],[]).append([float(x) for x in v[1:]])
assert rows['done']==[[1.0]] and rows['finite']==[[1.0,1.0]]
assert all(np.isfinite(v).all() for groups in rows.values() for v in groups)
decision=rows['decision'][0];assert decision[0]==531
assert rows['repeat_cost'][0]==decision[1:3], 'fresh objective changed during read-only probes'
log=(D/'kimera.log').read_text()
assert log.count('FL4Q no decreasing current-chart LM step')==1
assert not any(s in log for s in ['FL4U','FL4Q fallback wall budget exceeded','Smoother recovery','Persistent BPSAM recovery'])
assert all(row[0]<=10 for row in rows['trial'])
tol=decision[5];analytic=rows.get('direction',[[0,0]])[0][0]
descent=[v for v in rows.get('probe',[]) if v[4]<-tol and v[3]<0]
evidence=dict(valid_same_failed_update=True,parity=parity,failed_update=531,
    native_span_s=(actual[-1][0]-actual[0][0])*1e-9,decision=decision,trial_columns=['outer_iteration','fresh_cost','LM_cached_cost','lambda'],
    trials=rows['trial'],last=rows['last'][0],
    model_columns=['dimension','gradient_norm','diagonal_scaled_gradient_norm','diagonal_scaled_gradient_inf','scaled_H_eigen_min','scaled_H_eigen_max'],
    model=rows['model'][0],direction=rows.get('direction'),
    probe_columns=['alpha','fresh_plus_cost','fresh_minus_cost','central_derivative','plus_minus_initial_cost'],probes=rows.get('probe',[]),
    original_failure_retained=True,all_probes_read_only=True,central_derivative_and_decrease_exist=bool(descent and analytic<0),
    objective_repeat_exact=True,original_solver_changed=False,acceptance_rule_changed=False,
    classification='NOT_CONVERGED_DESCENT_DEMONSTRATED' if descent and analytic<0 else 'INSUFFICIENT_EVIDENCE_FOR_STATIONARITY',
    warning='No conclusion about global convergence, production repair, or covariance/accuracy performance.')
with (D/'validation.json').open('x') as f:json.dump(evidence,f,indent=2)
print(json.dumps(evidence,indent=2))
