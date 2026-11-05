"""CP63 independent fixed-weight directional audit; no optimization."""
from pathlib import Path
import collections
import hashlib
import json
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

H=Path(__file__).resolve().parent
A=H/'attempt_vector_api'
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'attempt_vector_api/results'
inputs={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):
    inputs[str(p)]=sha(p);return json.loads(p.read_text())
def lines(p):
    inputs[str(p)]=sha(p);return [json.loads(x) for x in p.read_text().splitlines()]
def mat(m):return np.asarray(m['data']).reshape(m['rows'],m['cols'])
def rel(a,b):return float(np.linalg.norm(np.asarray(a)-b)/(1+np.linalg.norm(b)))
def transform(v):
    v=np.asarray(v);t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[[4,5,6,3]]).as_matrix();t[:3,3]=v[:3];return t
def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def block(k,j):return (0 if j<3 else 1) if k[0]=='x' else 2 if k[0]=='v' else (3 if j<3 else 4)
def masked(k,d,alpha,mask):
    d=np.asarray(d).copy()
    for j in range(len(d)):
        b=block(k,j);keep=mask=='full' or mask=='poses_only' and b<2 or mask=='without_gyro_bias' and b!=4 or mask=='without_accel_bias' and b!=3
        d[j]*=alpha if keep else 0
    return d
def point_cost(f,poses):
    ext=mat(f['body_P_camera']);point=np.asarray(f['point']);rs=[]
    for k,cal,z in zip(f['keys'],f['calibrations'],f['measurements']):
        t=poses[k]@ext;q=t[:3,:3].T@(point-t[:3,3])
        if q[2]<=0:return None
        fx,fy,s,cx,cy,b=cal;assert s==0
        rs.extend([fx*q[0]/q[2]+cx-z[0],0 if z[1] is None else fx*(q[0]-b)/q[2]+cx-z[1],fy*q[1]/q[2]+cy-z[2]])
    return .5*float(np.dot(rs,rs))/f['sigma']**2
checks=collections.defaultdict(list)
def check(name,error,limit):
    checks[name].append(dict(error=float(error),limit=limit,passed=bool(error<limit)))
def model(h,d):return float(.5*h[-1,-1]-d@h[:-1,-1]+.5*d@h[:-1,:-1]@d)
all_phases={}
assert read(P/'result.json')['optimizer_calls']==0
assert read(A/'extraction_complete.json')['optimizer_calls']==0
for phase in ['ordinary','refresh']:
    fs={f['slot']:f for f in lines(P/(phase+'_factors.jsonl'))}
    rows=lines(P/(phase+'_samples.jsonl'))
    samples=collections.defaultdict(dict)
    for row in rows:samples[row['case']][row['slot']]=row
    base=read(P/(phase+'_base.json'));target=read(P/(phase+'_target.json'));system=read(P/(phase+'_system.json'))
    order=system['order'];delta=system['delta'];n=sum(len(delta[k]) for k in order)
    idx={};o=0
    for k in order:idx[k]=list(range(o,o+len(delta[k])));o+=len(delta[k])
    global_h=mat(system['H']);assembled=np.zeros_like(global_h)
    check('captured_equations',system['captured_equation_relative'],1e-12)
    check('saved_endpoint',system['endpoint_tangent'],1e-8)
    groups={}
    for slot,f in fs.items():
        keys=f['keys'];h=mat(f['H']);d=np.concatenate([delta[k] for k in keys]);b=np.zeros((len(d),5));o=0
        for k in keys:
            for j in range(len(delta[k])):b[o+j,block(k,j)]=delta[k][j]
            o+=len(delta[k])
        slope=-b.T@h[:-1,-1];curve=b.T@h[:-1,:-1]@b
        check('component_delta',rel(d,f['delta']),1e-10);check('component_matrix',rel(b,mat(f['components'])),1e-10)
        check('component_slope',rel(slope,f['slope']),1e-10);check('component_curvature',rel(curve,mat(f['curvature'])),1e-10)
        check('component_slope_sum',rel(slope.sum(),-d@h[:-1,-1]),1e-10)
        check('component_curvature_sum',rel(curve.sum(),d@h[:-1,:-1]@d),1e-10)
        inds=[j for k in keys for j in idx[k]]+[n];assembled[np.ix_(inds,inds)]+=h
        g=groups.setdefault(f['group'],dict(count=0,slope=np.zeros(5),curvature=np.zeros((5,5)),native={},model={}))
        g['count']+=1;g['slope']+=slope;g['curvature']+=curve
        for case,ss in samples.items():
            row=ss[slot];dd=np.concatenate([masked(k,delta[k],row['alpha'],row['mask']) for k in keys])
            check('native_model_independent',rel(model(h,dd),row['native_model']),1e-10)
            check('native_residual',row['native_residual_relative'],1e-9)
            check('native_residual_independent',rel(.5*np.dot(row['whitened_residual'],row['whitened_residual']),row['native_cost']),1e-9)
            g['native'][case]=g['native'].get(case,0)+row['native_cost'];g['model'][case]=g['model'].get(case,0)+row['native_model']
            if 'point' in f:
                fw=mat(f['Fw']);bw=np.asarray(f['bw']);rr=fw@dd-bw
                check('fixed_model_independent',rel(.5*rr@rr,row['fixed_point_model']),1e-10)
        # Native nonvisual and UNELIMINATED visual derivatives are different checks.
        expected=-d@h[:-1,-1] if f['group']!='vision' else (-np.asarray(f['bw'])@mat(f['Fw'])@d if 'point' in f else None)
        if expected is not None:
            field='native_cost' if f['group']!='vision' else 'fixed_point_cost'
            for p,m,eps in [('p5','m5',1e-5),('p6','m6',1e-6)]:
                fd=(samples[p][slot][field]-samples[m][slot][field])/(2*eps)
                check(('nonvisual' if f['group']!='vision' else 'fixed_visual')+'_directional_FD_'+str(eps),rel(fd,expected),1e-6)
                f.setdefault('directional_checks',[]).append(dict(eps=eps,expected=float(expected),actual=float(fd),relative=rel(fd,expected),passed=rel(fd,expected)<1e-6))
    check('factor_assembly',rel(assembled,global_h),1e-10)
    case_summary={}
    for case,ss in samples.items():
        row=next(iter(ss.values()));x=read(P/(phase+'_'+case+'_values.json'));poses={}
        for k in order:
            dd=masked(k,delta[k],row['alpha'],row['mask'])
            if k[0]=='x':
                xi=np.zeros((4,4));xi[:3,:3]=skew(dd[:3]);xi[:3,3]=dd[3:]
                t=transform(base[k])@expm(xi);actual=transform(x[k]);check('scipy_se3_retract',rel(actual,t),1e-8);poses[k]=actual
            else:check('scipy_vector_retract',rel(x[k],np.asarray(base[k])+dd),1e-8)
        transitions=collections.defaultdict(lambda:dict(count=0,slots=[],base_cost=0,end_cost=0))
        native_total=0;fixed=0;fixed_model=0;invalid=[];native_visual=0;native_model=0
        for slot,f in fs.items():
            rr=ss[slot];native_total+=rr['native_cost'];native_model+=rr['native_model']
            if f['group']=='vision':
                native_visual+=rr['native_cost'];old=f['starting_status']==0;new=rr['native_status']==0
                label='retained' if old and new else 'lost' if old else 'new' if new else 'rejected_both'
                z=transitions[label];z['count']+=1;z['slots'].append(slot);z['base_cost']+=samples['a0'][slot]['native_cost'];z['end_cost']+=rr['native_cost']
            if 'point' in f:
                pc=point_cost(f,poses)
                check('manual_visual_validity',0 if (pc is not None)==rr['fixed_point_valid'] else 1,1e-8)
                if pc is None:invalid.append(slot)
                else:check('manual_visual_cost',rel(pc,rr['fixed_point_cost']),1e-8);fixed+=pc
                fixed_model+=rr['fixed_point_model']
        case_summary[case]=dict(alpha=row['alpha'],mask=row['mask'],native_total=native_total,native_model=native_model,native_visual=native_visual,
                                fixed_starting_point_cost=None if invalid else fixed,fixed_starting_point_model=fixed_model,fixed_point_invalid=invalid,
                                visual_transitions=dict(transitions))
    # Physical endpoint changes, not independent estimates or trajectory errors.
    changes={}
    for k in order:
        if k[0]=='x':
            bt=transform(base[k]);tt=transform(target[k])
            changes[k]=dict(position_m=float(np.linalg.norm(tt[:3,3]-bt[:3,3])),rotation_deg=float(Rotation.from_matrix(bt[:3,:3].T@tt[:3,:3]).magnitude()*180/np.pi))
        elif k[0]=='b':changes[k]=dict(accel_bias=float(np.linalg.norm(np.asarray(target[k][:3])-base[k][:3])),gyro_bias=float(np.linalg.norm(np.asarray(target[k][3:])-base[k][3:])))
        else:changes[k]=dict(velocity=float(np.linalg.norm(np.asarray(target[k])-base[k])))
    all_phases[phase]=dict(groups=groups,cases=case_summary,endpoint_changes=changes,
        directional_checks=[dict(slot=s,group=f['group'],checks=f['directional_checks']) for s,f in fs.items() if 'directional_checks' in f],
        factor_count=len(fs),starting_accepted=sum('point' in f for f in fs.values()))
summary={k:dict(count=len(v),max_error=max(x['error'] for x in v),limit=v[0]['limit'],passed=all(x['passed'] for x in v)) for k,v in checks.items()}
def convert(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
inputs[str(Path(__file__))]=sha(__file__)
result=dict(audit_completed=True,numerical_checks_passed=all(x['passed'] for x in summary.values()),optimizer_calls=0,
            checks=summary,phases=all_phases,input_sha256=inputs)
with (H/'analysis.json').open('x') as f:json.dump(result,f,indent=2,default=convert,allow_nan=False)
print(json.dumps(dict(numerical_checks_passed=result['numerical_checks_passed'],checks=summary),indent=2))

