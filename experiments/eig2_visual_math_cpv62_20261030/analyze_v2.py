"""Independent fixed-geometry visual math audit; not trajectory scoring."""
import hashlib
import json
from pathlib import Path
import warnings
import numpy as np
from scipy import linalg

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def mat(d):return np.array(d['data']).reshape(d['rows'],d['cols'])
def relative(a,b):return float(np.linalg.norm(a-b)/(1+np.linalg.norm(b)))
def skew(q):
    x,y,z=q
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def analytic(row):
    n=len(row['keys']);F=np.zeros((3*n,6*n));E=np.zeros((3*n,3));res=np.zeros(3*n)
    ext=mat(row['body_P_camera']);Rc=ext[:3,:3];tc=ext[:3,3];point=np.array(row['point'])
    for i,(pose,cal,z) in enumerate(zip(row['poses'],row['calibrations'],row['measurements'])):
        pose=mat(pose);R=pose[:3,:3];t=pose[:3,3];fx,fy,s,cx,cy,base=cal
        assert s==0
        body=R.T@(point-t);q=Rc.T@(body-tc);x,y,depth=q
        assert depth>0
        predicted=np.array([fx*x/depth+cx,fx*(x-base)/depth+cx,fy*y/depth+cy])
        D=np.array([[fx/depth,0,-fx*x/depth**2],[fx/depth,0,-fx*(x-base)/depth**2],[0,fy/depth,-fy*y/depth**2]])
        if z[1] is None:D[1]=0;predicted[1]=0
        observed=np.array([z[0],0 if z[1] is None else z[1],z[2]])
        res[3*i:3*i+3]=predicted-observed
        F[3*i:3*i+3,6*i:6*i+6]=D@np.column_stack([Rc.T@skew(body),-Rc.T])
        E[3*i:3*i+3]=D@Rc.T@R.T
    return F,E,-res
def eliminate(F,E,b):
    U,s,_=np.linalg.svd(E,full_matrices=False)
    threshold=np.finfo(float).eps*max(E.shape)*s[0];rank=int(sum(s>threshold))
    if rank!=3:return None,dict(rank=rank,singular_values=s.tolist(),threshold=float(threshold))
    Fp=F-U@(U.T@F);bp=b-U@(U.T@b)
    A=np.column_stack([Fp,bp]);q=A.T@A
    raw=np.column_stack([F,b]);joint=raw.T@raw
    cross=E.T@raw
    with warnings.catch_warnings(record=True) as ws:
        warnings.simplefilter('always')
        reduction=cross.T@linalg.solve(E.T@E,cross,assume_a='pos')
    schur=joint-reduction
    return (q,schur),dict(rank=rank,singular_values=s.tolist(),E_condition=float(s[0]/s[-1]),
                          normal_condition=float((s[0]/s[-1])**2),normal_solve_warnings=[str(w.message) for w in ws],
                          constant_removed=float(np.linalg.norm(U.T@b)**2))
def delta_metrics(a,b,order):
    out={};off=0
    for k in order:
        dim=3 if k.startswith('v') else 6
        out[k]=(a[off:off+dim]-b[off:off+dim]).tolist();off+=dim
    poses=[v for k,v in out.items() if k.startswith('x')]
    return dict(mixed_tangent_norm=float(np.linalg.norm(a-b)),
                max_pose_rotation_increment_difference_deg=float(max(np.degrees(np.linalg.norm(v[:3])) for v in poses)),
                max_pose_translation_increment_difference_m=float(max(np.linalg.norm(v[3:]) for v in poses)),
                max_velocity_difference_m_s=float(max(np.linalg.norm(v) for k,v in out.items() if k.startswith('v'))),
                max_accel_bias_difference_m_s2=float(max(np.linalg.norm(v[:3]) for k,v in out.items() if k.startswith('b'))),
                max_gyro_bias_difference_rad_s=float(max(np.linalg.norm(v[3:]) for k,v in out.items() if k.startswith('b'))),
                per_key=out)
def dense_solve(H,g):
    with warnings.catch_warnings(record=True) as ws:
        warnings.simplefilter('always');d=linalg.solve(H,g,assume_a='gen')
    backward=float(np.linalg.norm(H@d-g)/(1+np.linalg.norm(g)+np.linalg.norm(H)*np.linalg.norm(d)))
    return d,dict(backward=backward,warnings=[str(w.message) for w in ws])
def main():
    assert json.loads((H/'extraction_complete.json').read_text())['completed']
    inputs=[Path(__file__),H/'extraction_complete.json']+[p for p in P.iterdir() if p.is_file()]
    pins={str(p):sha(p) for p in inputs}
    results=[];systems={};phases={}
    for phase in ['initial','postordinary','postrefresh']:
        rows=[json.loads(l) for l in (P/(phase+'_factors.jsonl')).open()]
        assert len(rows)==338
        sys=json.loads((P/(phase+'_system.json')).read_text())
        hn=mat(sys['H']);n=hn.shape[0]-1;order=sys['order'];offset={};off=0
        for k in order:offset[k]=off;off+=3 if k.startswith('v') else 6
        assert n==off
        rebuilt=mat(sys['nonvisual_H']);explicit=rebuilt.copy();manual=rebuilt.copy()
        phase_results=[]
        for row in rows:
            idx=[offset[k]+j for k in row['keys'] for j in range(6)]+[n];sub=np.ix_(idx,idx)
            h=mat(row['native_H']);rebuilt[sub]+=h
            record=dict(phase=phase,slot=row['slot'],status=row['status'],accepted=row['status']==0)
            if row['status']!=0:
                record['passed']=bool(np.linalg.norm(h)==0);record['zero_gaussian_norm']=float(np.linalg.norm(h))
                results.append(record);phase_results.append(record);continue
            F,E,b=mat(row['F']),mat(row['E']),mat(row['b']).ravel()
            Fw,Ew,bw=mat(row['Fw']),mat(row['Ew']),mat(row['bw']).ravel()
            Fa,Ea,ba=analytic(row);sigma=row['sigma'];assert sigma==3
            record.update(analytic_F_relative=relative(Fa,F),analytic_E_relative=relative(Ea,E),
                          analytic_b_relative=relative(ba,b),whitening_F_relative=relative(Fw,F/sigma),
                          whitening_E_relative=relative(Ew,E/sigma),whitening_b_relative=relative(bw,b/sigma),
                          finite_differences=row['FD'],residual_relative=row['residual_relative'])
            em,info=eliminate(Fw,Ew,bw);ea,ia=eliminate(Fa/sigma,Ea/sigma,ba/sigma)
            record.update(point_elimination=info,analytic_point_elimination=ia)
            passed=(max(record[k] for k in ['analytic_F_relative','analytic_E_relative','analytic_b_relative',
                      'whitening_F_relative','whitening_E_relative','whitening_b_relative'])<1e-10 and
                    row['residual_relative']<1e-10 and all(r['F_relative']<1e-6 and r['E_relative']<1e-6 for r in row['FD']))
            if em is None or ea is None:
                record['passed']=False;record['reason']='unresolved landmark rank';explicit=None;manual=None
            else:
                q,schur=em;qa,_=ea;np_=h.shape[0]-1
                record.update(native_vs_explicit_H_relative=relative(q[:np_,:np_],h[:np_,:np_]),
                              native_vs_explicit_g_relative=relative(q[:np_,-1],h[:np_,-1]),
                              native_vs_normal_H_relative=relative(schur[:np_,:np_],h[:np_,:np_]),
                              native_vs_normal_g_relative=relative(schur[:np_,-1],h[:np_,-1]),
                              native_vs_analytic_H_relative=relative(qa[:np_,:np_],h[:np_,:np_]),
                              native_vs_analytic_g_relative=relative(qa[:np_,-1],h[:np_,-1]),
                              native_constant_minus_btb=float(h[-1,-1]-bw@bw),
                              constant_difference=float(h[-1,-1]-q[-1,-1]),
                              expected_constant_difference=info['constant_removed'])
                passed=passed and max(record[k] for k in ['native_vs_explicit_H_relative','native_vs_explicit_g_relative',
                    'native_vs_normal_H_relative','native_vs_normal_g_relative','native_vs_analytic_H_relative','native_vs_analytic_g_relative'])<1e-7
                passed=passed and abs(record['native_constant_minus_btb'])/(1+abs(h[-1,-1]))<1e-10
                passed=passed and abs(record['constant_difference']-info['constant_removed'])/(1+abs(h[-1,-1]))<1e-10
                record['passed']=passed
                if explicit is not None:
                    explicit[sub]+=q;manual[sub]+=qa
            results.append(record);phase_results.append(record)
        assert relative(rebuilt,hn)<1e-12,'native reconstruction'
        good=[r for r in phase_results if r['accepted']]
        summary=dict(smart=len(rows),accepted=len(good),rejected=len(rows)-len(good),
                     failures=[r['slot'] for r in phase_results if not r['passed']],
                     max_FD_F_relative=max(d['F_relative'] for r in good for d in r['finite_differences']),
                     max_FD_E_relative=max(d['E_relative'] for r in good for d in r['finite_differences']),
                     max_analytic_F_relative=max(r['analytic_F_relative'] for r in good),
                     max_analytic_E_relative=max(r['analytic_E_relative'] for r in good),
                     max_analytic_b_relative=max(r['analytic_b_relative'] for r in good),
                     max_point_E_condition=max(r['point_elimination'].get('E_condition',float('inf')) for r in good),
                     native_reconstruction_relative=relative(rebuilt,hn),
                     max_constant_difference=max(r.get('constant_difference',0) for r in good),
                     summed_constant_difference=sum(r.get('constant_difference',0) for r in good))
        if explicit is not None:
            summary.update(max_native_explicit_H_relative=max(r['native_vs_explicit_H_relative'] for r in good),
                           max_native_explicit_g_relative=max(r['native_vs_explicit_g_relative'] for r in good),
                           max_native_analytic_H_relative=max(r['native_vs_analytic_H_relative'] for r in good),
                           max_native_analytic_g_relative=max(r['native_vs_analytic_g_relative'] for r in good))
        phases[phase]=summary
        if phase!='postrefresh' and explicit is not None:
            d0,n0=dense_solve(hn[:n,:n],hn[:n,-1])
            d1,n1=dense_solve(explicit[:n,:n],explicit[:n,-1])
            d2,n2=dense_solve(manual[:n,:n],manual[:n,-1])
            native=np.array(sys['native_delta'])
            assert all(d['backward']<1e-14 for d in [n0,n1,n2])
            systems[phase]=dict(native_captured_equation_relative=sys['captured_equation_relative'],
                native_captured_state_tangent=sys['captured_state_tangent'],native_backward=sys['native_backward'],
                native_dense_condition=float(np.linalg.cond(hn[:n,:n])),
                global_explicit_H_relative=relative(explicit[:n,:n],hn[:n,:n]),
                global_explicit_g_relative=relative(explicit[:n,-1],hn[:n,-1]),
                global_analytic_H_relative=relative(manual[:n,:n],hn[:n,:n]),
                global_analytic_g_relative=relative(manual[:n,-1],hn[:n,-1]),
                dense_native_vs_native_solver=delta_metrics(d0,native,order),
                explicit_vs_native_same_dense_solver=delta_metrics(d1,d0,order),
                analytic_vs_native_same_dense_solver=delta_metrics(d2,d0,order),
                solver_checks=[n0,n1,n2],native_dense_delta=d0.tolist(),explicit_dense_delta=d1.tolist(),analytic_dense_delta=d2.tolist())
    passed=all(r['passed'] for r in results) and len(systems)==2
    for p,h in pins.items():assert sha(p)==h,p
    report=dict(audit_completed=True,numerical_checks_passed=passed,suspect2_first_event_math_fault_found=not passed,
                phases=phases,systems=systems,records=results,input_sha256=pins,
                caveats=['Original native triangulated/cached points and accepted tracks, not CP61 local-frame candidate.',
                         'Constant term is intentionally unreduced in native smart Gaussian; compare Hessian/gradient separately.',
                         'Dense linear-system comparisons are not propagated nonlinear optimization or trajectory accuracy.',
                         'Passing checks apply to saved first event, not later marginalization, all geometries, or full CBS correctness.'])
    with (H/'analysis_v2.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False,default=lambda v: v.item() if isinstance(v,np.generic) else (_ for _ in ()).throw(TypeError(type(v).__name__)))
    print('NUMERICAL_PASS',passed)
    for p,d in phases.items():print(p,json.dumps(d))
    for p,d in systems.items():
        print(p,'GLOBAL', {k:v for k,v in d.items() if isinstance(v,(int,float))})
        for name in ['dense_native_vs_native_solver','explicit_vs_native_same_dense_solver','analytic_vs_native_same_dense_solver']:
            print(p,name,{k:v for k,v in d[name].items() if k!='per_key'})
    print('Failures',[(r['phase'],r['slot']) for r in results if not r['passed']])
if __name__=='__main__':main()
