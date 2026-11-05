"""Read-only explanation of CP63 failed gates; preserves original analysis/verdict."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'attempt_vector_api/results'
Q=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_first_factor_cpv52_20261030/attempt_compile1/results')
inputs={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):
    inputs[str(p)]=sha(p);return json.loads(p.read_text())
def lines(p):
    inputs[str(p)]=sha(p);return [json.loads(s) for s in p.read_text().splitlines()]
def mat(m):return np.asarray(m['data']).reshape(m['rows'],m['cols'])
def T(v):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(np.asarray(v)[[4,5,6,3]]).as_matrix();t[:3,3]=v[:3];return t
def skew(v):
    a,b,c=v;return np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
def E(v):
    m=np.zeros((4,4));m[:3,:3]=skew(v[:3]);m[:3,3]=v[3:];return expm(m)
def L(t):
    w=Rotation.from_matrix(t[:3,:3]).as_rotvec();a=np.linalg.norm(w);wx=skew(w)
    v=np.eye(3)+((1-np.cos(a))/a**2 if a>1e-4 else .5-a*a/24+a**4/720)*wx+((a-np.sin(a))/a**3 if a>1e-4 else 1/6-a*a/120+a**4/5040)*wx@wx
    return np.r_[w,np.linalg.solve(v,t[:3,3])]
def inv(t):
    q=np.eye(4);q[:3,:3]=t[:3,:3].T;q[:3,3]=-q[:3,:3]@t[:3,3];return q
def local_delta(f,delta,row):
    ds=[]
    for k in f['keys']:
        d=np.asarray(delta[k]).copy()
        for j in range(len(d)):
            b=(0 if j<3 else 1) if k[0]=='x' else 2 if k[0]=='v' else 3 if j<3 else 4
            mask=row['mask'];keep=mask=='full' or mask=='poses_only' and b<2 or mask=='without_gyro_bias' and b!=4 or mask=='without_accel_bias' and b!=3
            d[j]*=row['alpha'] if keep else 0
        ds.extend(d)
    return np.asarray(ds)
output={'original_numerical_verdict_unchanged':read(H/'analysis.json')['numerical_checks_passed'],'phases':{}}
for phase in ['ordinary','refresh']:
    fs={f['slot']:f for f in lines(P/(phase+'_factors.jsonl'))};rows=lines(P/(phase+'_samples.jsonl'))
    base=read(P/(phase+'_base.json'));sys=read(P/(phase+'_system.json'));delta=sys['delta'];xs={}
    for row in rows:
        if row['case'] not in xs:xs[row['case']]=read(P/(phase+'_'+row['case']+'_values.json'))
    failed=[]
    for row in rows:
        f=fs[row['slot']];h=mat(f['H']);d=local_delta(f,delta,row);mod=.5*h[-1,-1]-d@h[:-1,-1]+.5*d@h[:-1,:-1]@d
        re=abs(mod-row['native_model'])/(1+abs(row['native_model']))
        if re>=1e-10:
            hh=h.astype(np.longdouble);dd=d.astype(np.longdouble)
            terms=[.5*hh[-1,-1],-dd@hh[:-1,-1],.5*dd@hh[:-1,:-1]@dd];high=sum(terms)
            failed.append(dict(slot=row['slot'],group=f['group'],case=row['case'],native=row['native_model'],numpy=float(mod),longdouble=float(high),
                absolute_native_numpy=float(abs(mod-row['native_model'])),relative_native_numpy=re,
                native_vs_longdouble=float(abs(high-row['native_model'])),numpy_vs_longdouble=float(abs(high-mod)),terms=[float(z) for z in terms]))
    # For native BetweenFactor J2=I and native Prior H=I, recover W/r directly
    # from recorded factor equations; reconstruct physical mean and evaluate
    # exact SE3 error independently, without modifying or solving any graph.
    exact={}
    for slot in [0,929]:
        f=fs[slot];h=mat(f['H']);keys=f['keys']
        w=h[-7:-1,-7:-1];g=h[-7:-1,-1];r=-np.linalg.solve(w,g)
        poses={k:T(base[k]) for k in keys}
        hx=inv(poses[keys[0]])@poses[keys[1]] if slot==929 else poses[keys[0]]
        mean=hx@E(-r)
        errors=[]
        for row in [r for r in rows if r['slot']==slot]:
            xx=xs[row['case']];hh=inv(T(xx[keys[0]]))@T(xx[keys[1]]) if slot==929 else T(xx[keys[0]])
            rr=L(inv(mean)@hh);cost=.5*rr@w@rr
            errors.append(dict(case=row['case'],independent_cost=float(cost),native_cost=row['native_cost'],
                               relative=float(abs(cost-row['native_cost'])/(1+row['native_cost']))))
        j_numeric=[]
        for eps in [1e-5,1e-6]:
            def er(sign):
                ts={k:poses[k]@E(sign*eps*np.asarray(delta[k])) for k in keys}
                tt=inv(ts[keys[0]])@ts[keys[1]] if slot==929 else ts[keys[0]]
                return L(inv(mean)@tt)
            slope=float(((er(1)-er(-1))/(2*eps))@w@r)
            j_numeric.append(dict(eps=eps,exact_chain_slope=slope))
        d=np.concatenate([delta[k] for k in keys])
        exact[slot]=dict(recovered_residual=r.tolist(),recovered_mean=mean.tolist(),weight_eigenvalues=np.linalg.eigvalsh(w).tolist(),
                        native_approximate_slope=float(-d@h[:-1,-1]),exact_chain=j_numeric,
                        independent_cost_max_relative=max(z['relative'] for z in errors),cases=errors)
    output['phases'][phase]=dict(native_model_discrepancies=failed,nonvisual_exact_chain=exact)
controls={}
for name in ['initial','B_ordinary','B_refresh','A_ordinary','A_refresh']:
    p=Q/(name+'_factors.csv');inputs[str(p)]=sha(p);groups={}
    for f in csv.DictReader(p.read_text().splitlines()):
        typ=f['type'];groups[typ]=groups.get(typ,0)+float(f['cost'])
    controls[name]=groups
output['existing_CP52_cost_controls']=controls
for p in [Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/devel/include/gtsam/config.h'),
          Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/devel/include/gtsam/slam/BetweenFactor.h'),
          Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/devel/include/gtsam/nonlinear/ExtendedPriorFactor.h')]:
    inputs[str(p)]=sha(p)
inputs[str(Path(__file__))]=sha(__file__);output['input_sha256']=inputs
with (H/'gate_explanation.json').open('x') as f:json.dump(output,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in output.items() if k!='input_sha256'},indent=2))

