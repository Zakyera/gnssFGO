"""Read saved covariance matrices; no estimator or receiver is executed."""
import csv, datetime, hashlib, importlib.util, json, sys
from pathlib import Path
import numpy as np
from scipy.linalg import eigvalsh

sys.dont_write_bytecode = True
H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
H105 = E/'eth1_fusion_diagnosis_cpv105_20261104'
P = {'OFF': W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared',
     'ON': W/'runs/eth1_prior_fix_live_cpv103_20261103/prepared'}
PINS = {}
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def pin(p):
    p = Path(p); PINS[str(p)] = sha(p); return p
def read(p): return json.loads(pin(p).read_text())
def save(name, obj):
    with (H/name).open('x') as f: json.dump(obj, f, indent=2, allow_nan=False)
def stats(v):
    v = np.asarray(v, float).ravel(); assert len(v) and np.isfinite(v).all()
    return dict(n=len(v), min=float(v.min()), p05=float(np.percentile(v,5)),
        median=float(np.median(v)), mean=float(v.mean()), p95=float(np.percentile(v,95)), max=float(v.max()))
def trace(C, start): return np.trace(C[...,start:start+3,start:start+3],axis1=-2,axis2=-1)
def matrix_comparison(new, old):
    ev = np.array([eigvalsh(a,b) for a,b in zip(new,old)])
    return dict(generalized_eigenvalues=stats(ev),
        ordered_eigenvalue_medians=np.median(ev,axis=0).tolist(),
        fraction_all_directions_contract=float(np.mean(ev.max(axis=1)<1)),
        fraction_some_direction_expands=float(np.mean(ev.max(axis=1)>1)),
        geometric_variance_ratio=stats(np.exp(np.mean(np.log(ev),axis=1))),
        rotation_trace_ratio=stats(trace(new,0)/trace(old,0)),
        translation_trace_ratio=stats(trace(new,3)/trace(old,3)))

assert not (H/'analysis.json').exists()
old = read(H105/'belief_analysis.json')
Z = np.load(pin(H105/'belief_cohorts.npz'))
epoch = read(P['ON']/'input_audit.json')['epoch_ns']
stages = ('first','age2.5','age4.5','terminal')
matrix = {}
for role in ('glim_hesai','kimera_front'):
    matrix[role] = {}
    for mode in P:
        get = lambda s: Z[role+'_'+mode+'_'+s+'_covariance']
        matrix[role][mode] = {a+'_to_'+b:matrix_comparison(get(b),get(a))
            for a,b in [('first','age2.5'),('age2.5','terminal'),('first','terminal')]}
    matrix[role]['ON_over_OFF'] = {s:matrix_comparison(
        Z[role+'_ON_'+s+'_covariance'],Z[role+'_OFF_'+s+'_covariance']) for s in stages}

cal = read(P['ON']/'transport_tools/frame_calibration.json')
T = np.array(cal['T_alphasense_cpt7']); Ri = T[:3,:3].T; ti = -Ri@T[:3,3]
def hat(v):
    x,y,z=v; return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
A = np.zeros((6,6)); A[:3,:3]=Ri; A[3:,3:]=Ri; A[3:,:3]=hat(ti)@Ri
ends = np.rint(Z['kimera_front_end_s']*1e9).astype(np.int64)+epoch
cohort = set(map(int,ends)); assert len(cohort)==1454
arrays = {}; accounting = {}; native_results = {}
for mode, root in P.items():
    sroot = root/('full_'+mode.lower()+'_source')
    rroot = root/('full_'+mode.lower()+'_receiver')
    updates = {}
    with pin(sroot/'kimera_odometry.csv').open() as f:
        for row in csv.DictReader(f):
            u=int(row['field.header.seq']); assert u not in updates
            updates[u]=int(row['field.header.stamp'])
    native = {}; retires = {}; edge_retires = {}; local_counts = {}
    with pin(sroot/'estimators.log').open() as f:
        for line in f:
            if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
                z=line.split('KIMERA_CBS_REVISION_EDGE_ROW,',1)[1].strip().split(',')
                assert len(z)==64, len(z)
                u,a,b,rev=map(int,[z[0],z[2],z[3],z[6]])
                native.setdefault((a,b),[]).append((u,rev,np.array(z[28:],float).reshape(6,6)))
            elif 'KIMERA_CBS_REVISION_POSE_RETIRE_ROW,' in line:
                z=line.split('KIMERA_CBS_REVISION_POSE_RETIRE_ROW,',1)[1].strip().split(',')
                retires.setdefault(int(z[0]),[]).append(int(z[1]))
            elif 'KIMERA_CBS_REVISION_EDGE_RETIRE_ROW,' in line:
                z=line.split('KIMERA_CBS_REVISION_EDGE_RETIRE_ROW,',1)[1].strip().split(',')
                edge_retires[(int(z[1]),int(z[2]))]=int(z[0])
            elif 'KIMERA_CBS_OUTGOING_TIMING_ROW,' in line:
                z=line.split('KIMERA_CBS_OUTGOING_TIMING_ROW,',1)[1].strip().split(',')
                local_counts[int(z[0])]=int(z[5])
    first_retire=min(retires)
    terminal = {}
    for path in sorted((rroot/'receiver/archive/ids/kimera_front').glob('*/*')):
        f=pin(path).read_text().split(); assert len(f)==59
        terminal[int(f[4])] = dict(revision=int(f[1]),ordinal=int(f[2]),arrival=int(f[5]))
    grouped={}; matches=0; maxerr=0.; expired=0; cursor={}
    with pin(rroot/'events.csv').open() as f:
        for z in csv.reader(f):
            if z[0]!='B' or z[2]!='kimera_front':continue
            assert len(z)==52
            a,b,rev=map(int,[z[6],z[7],z[5]])
            key=(a,b); index=cursor.get(key,0); cursor[key]=index+1
            u,native_rev,C=native[key][index]; D=np.array(z[16:],float).reshape(6,6)
            rel=np.linalg.norm(A@C@A.T-D)/np.linalg.norm(D)
            maxerr=max(maxerr,float(rel)); matches+=1
            end=int(z[9]); arrival=int(z[1])
            if end not in cohort:continue
            term=terminal[end]
            if rev>term['revision']:
                expired+=1;continue
            grouped.setdefault(end,[]).append(dict(u=u,a=a,b=b,revision=rev,
                source_age=(updates[u]-end)*1e-9,arrival_age=(arrival-end)*1e-9,
                wire=D,native=C,arrival=arrival))
    assert matches==sum(map(len,native.values())) and maxerr<1e-12
    assert set(grouped)==cohort
    # Revalidate terminal and age2.5 arrays against saved CP105, by exact payload.
    for i,end in enumerate(ends):
        recs=grouped[int(end)]
        assert np.max(abs(recs[-1]['wire']-Z['kimera_front_'+mode+'_terminal_covariance'][i]))==0
        middle=[r for r in recs if r['arrival_age']<=2.5][-1]
        assert np.max(abs(middle['wire']-Z['kimera_front_'+mode+'_age2.5_covariance'][i]))==0
    transitions=[]; per_edge=[]
    for end in ends:
        recs=grouped[int(end)]
        tr=np.array([trace(r['native'],3) for r in recs]); age=np.array([r['source_age'] for r in recs])
        ratios=tr[1:]/tr[:-1]; j=int(np.argmin(ratios))+1
        last_age=recs[-1]['source_age']
        per_edge.append([end, len(recs), recs[j]['source_age'],ratios[j-1],
            last_age-recs[j]['source_age'],tr[-1]/tr[0],
            trace(recs[-1]['wire'],3)/trace(recs[0]['wire'],3),
            recs[j]['u'], int(recs[j]['u'] in retires)])
        for k in range(1,len(recs)):
            transitions.append([recs[k]['source_age'],ratios[k-1],
                int(recs[k]['u'] in retires), recs[k]['u'],int(recs[k]['u']<first_retire),
                last_age-recs[k]['source_age']])
    per_edge=np.array(per_edge); transitions=np.array(transitions)
    curve=[]
    for age in np.arange(0,4.81,.2):
        selected=[]
        for end in ends:
            recs=grouped[int(end)]
            at=[r for r in recs if r['source_age']<=age+1e-6]
            if at and at[-1]['source_age']>=age-.25:
                selected.append((at[-1],recs[0]))
        curve.append(dict(cutoff_source_age_s=float(age),n=len(selected),
            native_translation_trace_mean=float(np.mean([trace(r['native'],3) for r,_ in selected])),
            native_translation_trace_median=float(np.median([trace(r['native'],3) for r,_ in selected])),
            median_relative_to_first=float(np.median([trace(r['native'],3)/trace(f['native'],3) for r,f in selected]))))
    late=[]
    for end in ends:
        recs=grouped[int(end)]; middle=[r for r in recs if r['arrival_age']<=2.5][-1]
        late.append((middle,recs[-1]))
    late_native=np.array([trace(b['native'],3)/trace(a['native'],3) for a,b in late])
    late_wire=np.array([trace(b['wire'],3)/trace(a['wire'],3) for a,b in late])
    groups={}
    for name,idx in [('all',np.ones(len(transitions),bool)),
                     ('before_first_pose_retirement',transitions[:,4]==1),
                     ('retirement_update',transitions[:,2]==1),
                     ('no_retirement_update',transitions[:,2]==0),
                     ('last_0.4s_of_saved_edge_life',transitions[:,5]<=.400001)]:
        if not idx.any():continue
        x=transitions[idx]
        groups[name]=dict(n=len(x),per_step_native_translation_ratio=stats(x[:,1]),
            fraction_over_half_drop=float(np.mean(x[:,1]<.5)),
            fraction_increasing=float(np.mean(x[:,1]>1)))
    native_results[mode]=dict(first_pose_retirement_update=first_retire,
        first_pose_retirement_time_s=(updates[first_retire]-epoch)*1e-9,
        pose_retirement_updates=len(retires),source_updates=len(updates),
        late_native_translation_ratio=stats(late_native),late_CPT7_translation_ratio=stats(late_wire),
        strongest_step_ratio=stats(per_edge[:,3]),strongest_step_source_age_s=stats(per_edge[:,2]),
        strongest_step_time_before_last_saved_version_s=stats(per_edge[:,4]),
        fraction_strongest_step_on_retirement_update=float(np.mean(per_edge[:,8])),
        transitions=groups,age_curve=curve)
    accounting[mode]=dict(native_versions=sum(map(len,native.values())),receiver_versions=matches,
        max_native_to_received_covariance_relative_error=maxerr,
        selected_edges=len(grouped),expired_versions_excluded_in_selected_cohort=expired,
        terminal_exact_array_checks=len(ends),age2p5_exact_array_checks=len(ends))
    arrays[mode+'_per_edge']=per_edge;arrays[mode+'_transitions']=transitions

# Independent empirical discrepancy diagnostics on fixed CP105 endpoint cohorts.
# This tests model/reference mismatch, not independent trials or true calibration.
discrepancy={}
for role in ('kimera_front','glim_hesai'):
    discrepancy[role]={}
    times=Z[role+'_end_s']; adjacent=np.abs(np.diff(times)-(.2 if role=='kimera_front' else .1))<.01
    for mode in P:
        out={}
        for stage in stages:
            C=Z[role+'_'+mode+'_'+stage+'_covariance'];e=Z[role+'_'+mode+'_'+stage+'_residual']
            row={}
            for name,start in [('rotation',0),('translation',3)]:
                c=C[:,start:start+3,start:start+3];r=e[:,start:start+3];tr=trace(C,start)
                q=np.einsum('ni,ni->n',r,np.linalg.solve(c,r[...,None])[...,0])
                centered=r-r.mean(axis=0); empirical=centered.T@centered/len(r)
                avg=c.mean(axis=0);eig=eigvalsh(empirical,avg)
                row[name]=dict(mean_variance=float(tr.mean()),RMS_reference_discrepancy=float(np.sqrt(np.mean(np.sum(r*r,axis=1)))),
                    RMS_reported_norm=float(np.sqrt(tr.mean())),mean_residual=r.mean(axis=0).tolist(),
                    centered_reference_scatter_over_mean_reported_covariance_eigenvalues=eig.tolist(),
                    nominal95_inclusion_fraction=float(np.mean(q<=7.814727903251179)),
                    adjacent_interval_residual_axis_correlations=[float(np.corrcoef(r[:-1,i][adjacent],r[1:,i][adjacent])[0,1]) for i in range(3)],
                    adjacent_pairs=int(adjacent.sum()))
            out[stage]=row
        discrepancy[role][mode]=out
for p,v in PINS.items():assert sha(p)==v,p
np.savez_compressed(H/'maturation_arrays.npz',**arrays)
save('analysis.json',dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),
    accounting=accounting,matrix_comparison=matrix,native_maturation=native_results,
    reference_discrepancy=discrepancy,input_sha256=PINS,
    caveats=['Historical OFF and corrected ON are different realizations/backends.',
      'Pose retirement association is confounded with routine source updates; no marginal information decomposition recorded.',
      'Age curves may have different n per cutoff; matrices and discrepancy stage comparisons retain the identical CP105 cohort.',
      'Reference residuals are dependent and IE-TC shares CPT7; no NEES/calibration or true-error independence claim.']))
print(json.dumps(dict(passed=True,accounting=accounting,native_maturation={m:{k:v for k,v in d.items() if k!='age_curve'} for m,d in native_results.items()}),indent=2))
