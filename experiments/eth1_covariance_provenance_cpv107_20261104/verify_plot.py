"""Independent matrix arithmetic checks and static evidence figure."""
import hashlib,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;E=H.parent
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
d=read(H/'analysis.json');assert d['passed']
assert not (H/'verification.json').exists()
Z=np.load(E/'eth1_fusion_diagnosis_cpv105_20261104/belief_cohorts.npz')
pins={str(H/'analysis.json'):sha(H/'analysis.json'),str(H/'insertions.json'):sha(H/'insertions.json')}
maximum=0.;checks=0
for role in ('kimera_front','glim_hesai'):
    for mode in ('OFF','ON'):
        for a,b in [('first','age2.5'),('age2.5','terminal'),('first','terminal')]:
            X=Z[role+'_'+mode+'_'+a+'_covariance'];Y=Z[role+'_'+mode+'_'+b+'_covariance']
            eig=[]
            for x,y in zip(X,Y):
                L=np.linalg.cholesky(x);Li=np.linalg.inv(L);ev=np.linalg.eigvalsh(Li@y@Li.T)
                eig.append(ev);checks+=1
            eig=np.array(eig);saved=d['matrix_comparison'][role][mode][a+'_to_'+b]
            err=np.max(abs(np.median(eig,axis=0)-saved['ordered_eigenvalue_medians']))
            maximum=max(maximum,float(err));assert err<1e-11
            assert abs(np.mean(eig.max(axis=1)<1)-saved['fraction_all_directions_contract'])<1e-12
            q=np.exp(np.mean(np.log(eig),axis=1))
            assert abs(np.median(q)-saved['geometric_variance_ratio']['median'])<1e-11
    for mode in ('OFF','ON'):
        for stage in ('first','age2.5','age4.5','terminal'):
            C=Z[role+'_'+mode+'_'+stage+'_covariance'];e=Z[role+'_'+mode+'_'+stage+'_residual']
            for start,name in [(0,'rotation'),(3,'translation')]:
                c=C[:,start:start+3,start:start+3];r=e[:,start:start+3]
                q=np.sum(np.linalg.solve(np.linalg.cholesky(c),r[...,None])[...,0]**2,axis=1)
                saved=d['reference_discrepancy'][role][mode][stage][name]
                assert abs(np.mean(q<=7.814727903251179)-saved['nominal95_inclusion_fraction'])<1e-12
                assert abs(np.sqrt(np.sum(c[:,[0,1,2],[0,1,2]])/len(c))-saved['RMS_reported_norm'])<1e-12
for mode in ('OFF','ON'):
    x=np.load(H/'maturation_arrays.npz')[mode+'_transitions']
    n=d['native_maturation'][mode]
    assert len(x)==n['transitions']['all']['n']
    assert int((x[:,2]==0).sum())==253
    assert abs(np.median(x[x[:,5]<=.400001,1])-n['transitions']['last_0.4s_of_saved_edge_life']['per_step_native_translation_ratio']['median'])<1e-12

plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
colors={'OFF':'#376ea6','ON':'#d26722'}
for mode,color in colors.items():
    curve=d['native_maturation'][mode]['age_curve']
    ax[0,0].plot([r['cutoff_source_age_s'] for r in curve],[r['median_relative_to_first'] for r in curve],color=color,label=mode,lw=2)
    stages=('first','age2.5','age4.5','terminal')
    age=[np.median(Z['kimera_front_'+mode+'_'+s+'_arrival_age']) for s in stages]
    rows=[d['reference_discrepancy']['kimera_front'][mode][s]['translation'] for s in stages]
    ax[0,1].plot(age,[r['RMS_reference_discrepancy']*1000 for r in rows],'-o',color=color,label=mode+' reference discrepancy')
    ax[0,1].plot(age,[r['RMS_reported_norm']*1000 for r in rows],'--s',color=color,label=mode+' reported norm')
    ev=d['matrix_comparison']['kimera_front'][mode]['age2.5_to_terminal']['ordered_eigenvalue_medians']
    ax[1,0].plot(range(1,7),ev,'-o',color=color,label=mode)
    x=np.load(H/'maturation_arrays.npz')[mode+'_transitions'];x=x[x[:,5]<=.400001,1]
    ax[1,1].hist(x,bins=np.linspace(.65,1.25,50),density=True,histtype='step',linewidth=2,color=color,label=mode+' (n='+str(len(x))+')')
ax[0,0].set(title='Native Alpha covariance keeps tightening late',xlabel='Belief age at native source update (s)',ylabel='Median translation trace / first trace',yscale='log')
ax[0,0].legend();ax[0,0].grid(alpha=.2)
ax[0,1].set(title='Error plateaus; reported uncertainty keeps falling',xlabel='Median observed arrival age (s)',ylabel='Translation tangent norm (mm)')
ax[0,1].legend(fontsize=8);ax[0,1].grid(alpha=.2)
ax[1,0].set(title='Full-matrix late contraction is direction-dependent',xlabel='Ordered generalized covariance eigenvalue',ylabel='Terminal / age-2.5s covariance ratio',xticks=range(1,7))
ax[1,0].axhline(1,color='grey',ls=':',lw=1);ax[1,0].legend();ax[1,0].grid(alpha=.2)
ax[1,1].set(title='Last 0.4s of each saved belief lifetime',xlabel='Native translation trace / preceding version',ylabel='Density')
ax[1,1].axvline(1,color='grey',ls=':',lw=1);ax[1,1].legend(fontsize=8)
fig.suptitle('CP-V107 · Kimera covariance maturation, saved ETH1 data',fontsize=16)
fig.supxlabel('1,454 matched physical intervals; age-curve counts vary slightly. OFF=CP95; ON=corrected CP103.\nIE-TC discrepancy is not independent true error or a calibration score. No new estimator run.',fontsize=9)
fig.savefig(H/'covariance_maturation.png',dpi=170);plt.close(fig)
pins[str(H/'maturation_arrays.npz')]=sha(H/'maturation_arrays.npz')
pins[str(E/'eth1_fusion_diagnosis_cpv105_20261104/belief_cohorts.npz')]=sha(E/'eth1_fusion_diagnosis_cpv105_20261104/belief_cohorts.npz')
with (H/'verification.json').open('x') as f:json.dump(dict(passed=True,independent_generalized_matrix_checks=checks,
    max_ordered_eigenvalue_median_error=maximum,Cholesky_whitening_checked=True,
    lifecycle_summary_array_checks=True,input_sha256=pins),f,indent=2)
print(json.dumps(dict(passed=True,checks=checks,max_error=maximum)))
