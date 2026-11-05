"""CP64 independent SE3/Jacobian/assembly and response analysis."""
import collections,hashlib,json
from pathlib import Path
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'
inputs={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):inputs[str(p)]=sha(p);return json.loads(p.read_text())
def lines(p):inputs[str(p)]=sha(p);return [json.loads(l) for l in p.read_text().splitlines()]
def mat(m):return np.asarray(m['data']).reshape(m['rows'],m['cols'])
def rel(a,b):return float(np.linalg.norm(np.asarray(a)-b)/(1+np.linalg.norm(b)))
def skew(v):
    a,b,c=v;return np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
def E(v):
    m=np.zeros((4,4));m[:3,:3]=skew(v[:3]);m[:3,3]=v[3:];return expm(m)
def L(t):
    w=Rotation.from_matrix(t[:3,:3]).as_rotvec();a=np.linalg.norm(w);wx=skew(w)
    aa=(1-np.cos(a))/a**2 if a>1e-4 else .5-a*a/24+a**4/720
    bb=(a-np.sin(a))/a**3 if a>1e-4 else 1/6-a*a/120+a**4/5040
    v=np.eye(3)+aa*wx+bb*wx@wx
    return np.r_[w,np.linalg.solve(v,t[:3,3])]
def inv(t):
    q=np.eye(4);q[:3,:3]=t[:3,:3].T;q[:3,3]=-q[:3,:3]@t[:3,3];return q
def T(v):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(np.asarray(v)[[4,5,6,3]]).as_matrix();t[:3,3]=v[:3];return t
def error(r,poses):
    z=mat(r['mean'])
    if r['slot']==929:return L(inv(z)@inv(poses[0])@poses[1])
    return -L(inv(poses[0])@z)
checks=collections.defaultdict(list)
def check(name,e,limit):checks[name].append(dict(error=float(e),limit=limit,passed=bool(e<limit)))
def fd(r,poses,eps):
    result=np.zeros_like(mat(r['J_exact']))
    for i in range(len(poses)):
        for j in range(6):
            d=np.zeros(6);d[j]=eps;plus=list(poses);minus=list(poses)
            plus[i]=poses[i]@E(d);minus[i]=poses[i]@E(-d)
            result[:,6*i+j]=(error(r,plus)-error(r,minus))/(2*eps)
    return result
def changes(base,result):
    per={}
    for k in base.keys()&result.keys():
        if k[0]=='x':
            bt=T(base[k]);rt=T(result[k])
            per[k]=dict(position_m=float(np.linalg.norm(rt[:3,3]-bt[:3,3])),
                        rotation_deg=float(Rotation.from_matrix(bt[:3,:3].T@rt[:3,:3]).magnitude()*180/np.pi))
        elif k[0]=='v':per[k]=dict(velocity_m_s=float(np.linalg.norm(np.asarray(result[k])-base[k])))
        elif k[0]=='b':per[k]=dict(accel_bias_m_s2=float(np.linalg.norm(np.asarray(result[k][:3])-base[k][:3])),
                                 gyro_bias_rad_s=float(np.linalg.norm(np.asarray(result[k][3:])-base[k][3:])))
        else:raise ValueError(k)
    fields=sorted({m for v in per.values() for m in v})
    maximum={f:max(v[f] for v in per.values() if f in v) for f in fields}
    return dict(per_state=per,maximum=maximum)
native=read(P/'result.json')
assert native['checkpoint_linear_solves']==6 and native['fixture_linear_solves']==0
records=lines(P/'derivatives.jsonl');assert len(records)==10
bylabel={r['label']:r for r in records}
for r in records:
    assert r['passed']
    j=mat(r['J_exact']);jn=mat(r['J_native']);R=mat(r['R']);res=np.asarray(r['residual']);poses=[mat(x) for x in r['poses']]
    manual=error(r,poses)
    check('manual_SE3_residual',rel(manual,res),1e-8)
    check('manual_SE3_cost',rel(.5*np.linalg.norm(R@manual)**2,r['native_cost']),1e-8)
    check('physical_cost_identity',rel(r['exact_cost'],r['native_cost']),1e-12)
    for name,J in [('exact',j),('native',jn)]:
        ab=np.column_stack((R@J,-R@res));check('target_'+name+'_Gaussian',rel(mat(r['H_'+name]),ab.T@ab),1e-10)
    for nfd in r['FD']:
        eps=nfd['eps'];jf=fd(r,poses,eps)
        check('native_raw_FD',nfd['raw_relative'],1e-6);check('native_white_FD',nfd['white_relative'],1e-6)
        check('independent_raw_FD',rel(jf,j),1e-6);check('independent_white_FD',rel(R@jf,R@j),1e-6)
        if 'slope_relative' in nfd:
            check('native_directional_FD',nfd['slope_relative'],1e-6)
            direction=np.asarray(r['direction']);slope=float(res@R.T@R@jf@direction)
            check('independent_directional',rel(slope,nfd['slope_predicted']),1e-6)
baseline=read(P/'baseline.json')
check('ordinary_equation',baseline['ordinary_equations'],1e-12);check('refresh_equation',baseline['refresh_equations'],1e-12)
check('ordinary_baseline_state',baseline['ordinary_state'],1e-6);check('refresh_baseline_state',baseline['refresh_state'],1e-5)
previous=read(P/'previous.json');initial=read(P/'initial.json')
captured={p:read(P/('captured_'+p+'.json')) for p in ['ordinary','refresh']}
results={};systems={};costs={};metrics={}
for arm in ['A','B','C']:
    for phase in ['ordinary','refresh']:
        label=arm+'_'+phase;sys=read(P/(label+'_system.json'));start=read(P/(label+'_start.json'));end=read(P/(label+'_result.json'))
        results[label]=end;hh=mat(sys['H']);h0=mat(sys['native_same_base_H']);d=np.asarray(sys['delta']);n=len(d)
        order=sys['order'];offset=0;indices={}
        for k in order:
            dim=3 if k[0]=='v' else 6;indices[k]=list(range(offset,offset+dim));dd=d[offset:offset+dim];offset+=dim
            if k[0]=='x':check('independent_pose_retract',rel(T(end[k]),T(start[k])@E(dd)),1e-8)
            else:check('independent_vector_retract',rel(end[k],np.asarray(start[k])+dd),1e-8)
        check('native_backward',sys['backward'],1e-14)
        backward=np.linalg.norm(hh[:-1,:-1]@d-hh[:-1,-1])/(1+np.linalg.norm(hh[:-1,-1])+np.linalg.norm(hh[:-1,:-1])*np.linalg.norm(d))
        check('independent_backward',backward,1e-14)
        assert sys['non_target_difference']==0
        expected=h0.copy()
        if arm!='A':
            target=bylabel[label];inds=[i for k in target['keys'] for i in indices[k]]+[n]
            expected[np.ix_(inds,inds)]+=mat(target['H_exact'])-mat(target['H_native'])
        check('target_only_full_system_assembly',rel(hh,expected),1e-10)
        sys_summary=dict(backward=sys['backward'],independent_backward=float(backward),used_valid=sys['used_valid'],
                         used_valid_slots=sys['used_valid_slots'],changed_slot=sys['changed_slot'])
        systems[label]=sys_summary
        rows=lines(P/(label+'_costs.jsonl'));assert len(rows)==356
        groups=collections.defaultdict(float);statuses=collections.Counter()
        for row in rows:
            groups[row['group']]+=row['cost'];check('native_cost_residual',row['residual_parity'],1e-9)
            if row['group']=='vision':statuses[row['status']]+=1
        costs[label]=dict(groups=dict(groups),total=sum(groups.values()),vision_statuses=dict(statuses),
                          valid_slots=[r['slot'] for r in rows if r['group']=='vision' and r['status']==0])
        metrics[label]=dict(old_state_revision=changes(previous,end),all_state_from_initial=changes(initial,end),
                            actual_step=changes(start,end),versus_original=changes(results['A_'+phase],end))
    if arm!='A':
        a=metrics['A_refresh']['old_state_revision']['maximum'];b=metrics[arm+'_refresh']['old_state_revision']['maximum']
        metrics[arm+'_refresh']['reduction_percent']={k:100*(a[k]-b[k])/a[k] for k in a}
        metrics[arm+'_refresh']['under_one_percent_on_both_primary_reductions']=bool(all(metrics[arm+'_refresh']['reduction_percent'][k]<1 for k in ['rotation_deg','gyro_bias_rad_s']))
# Baseline costs checked against prior preserved CP63 native saved endpoints.
prior=read(H.parent/'eig2_directional_cpv63_20261030/analysis.json')
for phase in ['ordinary','refresh']:
    for g,v in costs['A_'+phase]['groups'].items():check('prior_CP63_baseline_cost',rel(v,prior['phases'][phase]['groups'][g]['native']['a100']),1e-8)
# Reference control only: no CP52 graph rerun or new no-CBS solution.
control=read(H.parent/'eig2_first_factor_cpv52_20261030/analysis.json')
summary={k:dict(count=len(v),max_error=max(x['error'] for x in v),limit=v[0]['limit'],passed=all(x['passed'] for x in v)) for k,v in checks.items()}
out=dict(completed=True,numerical_checks_passed=all(v['passed'] for v in summary.values()),checkpoint_linear_solves=6,
         derivative_records=10,checks=summary,systems=systems,costs=costs,metrics=metrics,
         controls='CP52 existing saved no-CBS control, not rerun',input_sha256=inputs)
inputs[str(Path(__file__))]=sha(__file__)
def convert(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
with (H/'analysis.json').open('x') as f:json.dump(out,f,indent=2,default=convert,allow_nan=False)
print(json.dumps(dict(numerical_checks_passed=out['numerical_checks_passed'],checks=summary,
    final_response={arm:metrics[arm+'_refresh'] for arm in ['A','B','C']},
    final_costs={arm:costs[arm+'_refresh'] for arm in ['A','B','C']}),indent=2,default=convert))

