"""Independent saved-array checks and static scientific figure; no replay."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.linalg import eigh
from scipy.stats import chi2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent
assert not (H/'verification.json').exists()
r=json.loads((H/'analysis.json').read_text())
data=np.load(H/'selected_records.npz',allow_pickle=False)
checks={}
worst=0.
for role in ['GLIM','Kimera']:
    for stage in ['wire_first','age0','age1','age2','age4']:
        prefix=role+'_'+stage+'_'
        assert np.array_equal(data[prefix+'OFF__endpoint_ns'],data[prefix+'ON__endpoint_ns'])
        assert np.array_equal(data[prefix+'OFF__update_ns'],data[prefix+'ON__update_ns'])
        for mode in ['OFF','ON']:
            key=prefix+mode
            e=data[key+'__error'];C=data[key+'__covariance'];q=data[key+'__q']
            vals,vecs=np.linalg.eigh(C)
            assert vals.min()>0 and np.isfinite(C).all()
            wh=np.einsum('nji,nj->ni',vecs,e)/np.sqrt(vals)
            q_eig=np.sum(wh**2,axis=1)
            discrepancy=float(np.max(abs(q-q_eig)/(1+q)))
            worst=max(worst,discrepancy)
            assert discrepancy<1e-7
            for name,sl in [('rotation',slice(0,3)),('translation',slice(3,6))]:
                tr=np.trace(C[:,sl,sl],axis1=1,axis2=2)
                norm2=np.sum(e[:,sl]**2,axis=1)
                s=r['sources'][role][stage][mode][name]
                assert np.isclose(np.mean(norm2)/np.mean(tr),s['squared_error_over_variance'],rtol=1e-12)
            expected_n=r['sources'][role][stage][mode]['n']
            assert len(q)==expected_n
            checks[key]=dict(n=len(q),minimum_eigenvalue=float(vals.min()),eigen_whitening_error=discrepancy)
        if stage.startswith('age'):
            assert np.array_equal(data[prefix+'OFF__endpoint_ns'],data[role+'_age0_OFF__endpoint_ns'])
            age=(data[prefix+'OFF__update_ns']-data[prefix+'OFF__endpoint_ns'][:,1])*1e-9
            target=float(stage[3:])
            assert age.min()>=target and age.max()<=target+.250000001

fig,axes=plt.subplots(2,2,figsize=(11,7.6),layout='constrained')
for col,role in enumerate(['GLIM','Kimera']):
    for row,(block,unit,metric,scale) in enumerate([
        ('translation','mm','translation_m',1000),('rotation','degrees','rotation_deg',1)]):
        ax=axes[row,col]
        for mode,color in [('OFF','#777777'),('ON','#007f97')]:
            ss=[r['sources'][role]['age'+str(a)][mode] for a in [0,1,2,4]]
            uncertainty=[s[block]['reported_norm_rms'] for s in ss]
            error=[s['evo'][metric]['rmse']*scale for s in ss]
            ax.plot([0,1,2,4],error,'o-',color=color,label=mode+' observed discrepancy')
            ax.plot([0,1,2,4],uncertainty,'s--',color=color,label=mode+' reported uncertainty')
        ax.set_yscale('log');ax.set_xticks([0,1,2,4]);ax.grid(alpha=.2,which='both')
        ax.set_title(role+' | '+block);ax.set_ylabel('RMS norm ('+unit+')')
        ax.set_xlabel('Approximate age of the same belief (seconds)')
        ax.legend(fontsize=8,loc='best')
fig.suptitle('ETH1 split-IMU: uncertainty shrinkage versus relative error\nSame edges and source-update times in OFF and ON',fontsize=14)
fig.text(.5,-.035,'Reported uncertainty = sqrt(mean covariance-block trace), not a 95% bound.\n'
         'Observed discrepancy uses EVO at actual belief intervals (~0.2 s). IE-TC is not independent error-free ground truth.',
         ha='center',fontsize=9)
fig.savefig(H/'covariance_revision_age.png',dpi=170,bbox_inches='tight')
plt.close(fig)

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert all(sha(p)==h for p,h in r['input_hashes'].items())
terminal={}
for mode,name in [('OFF','eth1_alpha_bias_rw_full_off_cpv89_20261102'),
                  ('ON','eth1_split_imu_persistent_on_cpv91_20261102_attempt1')]:
    c=json.loads((H.parent/name/'complete.json').read_text())
    assert all(sha(p)==h for p,h in c['output_sha256'].items())
    terminal[mode]=len(c['output_sha256'])
out=dict(passed=True,checks=checks,eigen_whitening_relative_error_max=worst,
         exact_OFF_ON_endpoints_and_update_times=True,same_age_cohort=True,
         inputs_unchanged=len(r['input_hashes']),immutable_terminal_files_unchanged=terminal,
         file_sha256={p.name:sha(p) for p in [H/'analyze.py',H/'PROTOCOL.txt',H/'analysis.json',
            H/'selected_records.npz',H/'covariance_revision_age.png',Path(__file__)]},
         caveat='Diagnostic arithmetic validation, not covariance calibration or estimator independence.')
with (H/'verification.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in out.items() if k not in ('checks','file_sha256')},indent=2))
