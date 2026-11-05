"""Compact retained-data presentation. Never changes input artifacts."""
import json
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.dont_write_bytecode=True
import analyze as a

extra={}
fig,axes=plt.subplots(2,3,figsize=(14,8),layout='constrained')
for col,seq in enumerate(a.CASES):
    x=json.loads((a.HERE/f'{seq}_results.json').read_text())
    extra[seq]={}
    for row,(part,error,sigma) in enumerate([
        ('Translation','rho_error_mm','translation_sigma_norm_mm'),
        ('Rotation','rotation_error_deg','rotation_sigma_norm_deg')]):
        ax=axes[row,col]
        for source,color in [('glim_hesai','#16784b'),('kimera_front','#cd7700')]:
            rr=[r for r in x['records'] if r['source']==source and r['stage']=='first']
            for mode,marker in [('off','o'),('on','x')]:
                xx=np.asarray([r[mode][sigma] for r in rr])
                yy=np.asarray([r[mode][error] for r in rr])
                ax.scatter(xx,yy,s=8,alpha=.45,color=color,marker=marker,
                           label=f"{source.split('_')[0].upper()} {mode.upper()}",rasterized=True)
        ax.set_xscale('log');ax.set_yscale('log');ax.grid(alpha=.2)
        lo=min(ax.get_xlim()[0],ax.get_ylim()[0]);hi=max(ax.get_xlim()[1],ax.get_ylim()[1])
        ax.plot([lo,hi],[lo,hi],color='gray',ls='--',lw=1,label='equal norm scale')
        unit='mm' if row==0 else 'deg'
        ax.set_xlabel(f'Reported sqrt(trace(block)) [{unit}]')
        ax.set_ylabel(f'Reference error norm [{unit}]')
        ax.set_title(f'{seq.upper()} — {part}')
    if seq in ['eig2','spx3']:
        old=json.loads((a.HERE.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017'/f'{seq}_results.json').read_text())
        lut={(r['receiver_from_ns'],r['receiver_to_ns']):r for r in old['records']}
        groups={}
        for comparison in ['off_pre','pre']:
            for channel,metric in [('translation','translation_m'),('rotation','rotation_deg')]:
                good,bad=[],[]
                for rec in x['injections']['G2K']['records']:
                    oldrec=lut[rec['receiver_from_ns'],rec['receiver_to_ns']]
                    # The existing FL4y names are inspected instead of replacing
                    # internal pre-update poses with native/future trajectories.
                    errors=oldrec['errors']
                    sender=next(k for k in errors if 'aligned' in k and 'oracle' not in k)
                    receiver=next(k for k in errors if k==comparison or k.endswith('_'+comparison))
                    is_good=errors[sender][metric]<errors[receiver][metric]
                    (good if is_good else bad).append(rec['aligned'][channel+'_trace_'+('rad2' if channel=='rotation' else 'm2')])
                groups[comparison+'_'+channel]=dict(better_count=len(good),worse_count=len(bad),
                    median_variance_when_better=float(np.median(good)),
                    median_variance_when_worse=float(np.median(bad)),
                    worse_over_better_variance=float(np.median(bad)/np.median(good)))
        extra[seq]['G2K_sender_better_or_worse_than_receiver']=groups
axes[0,0].legend(fontsize=8,loc='best')
fig.suptitle('First-publication relative beliefs (~0.20 s); saved OFF/ON runs\n'
             'IE-TC reference shares CPT7. Dashed line is a norm scale, not a 95% boundary. ETH1: matched pre-stop interval.',fontsize=11)
plot=a.HERE/'uncertainty_vs_reference_error.png'
assert not plot.exists()
fig.savefig(plot,dpi=155)
plt.close(fig)
a.write(a.HERE/'injection_quality_groups.json',extra)
print(json.dumps(extra,indent=2))
