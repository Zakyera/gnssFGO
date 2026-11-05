"""Supplemental diagnostics and static scientific plots; no optimization."""
import csv,json
from pathlib import Path
import numpy as np
from scipy.linalg import eigvalsh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from localize import H,S,P,pin,read,save,sha,PINS,runs,stats
def main():
    assert not (H/'findings.json').exists()
    d=read(H/'localization.json');b=read(H/'belief_analysis.json');revisions=read(S/'evaluation/revision_events.json');r=read(S/'results.json')
    a=np.load(pin(H/'localized.npz'));z=np.load(pin(S/'evaluation/comparison.npz'));bc=np.load(pin(H/'belief_cohorts.npz'))
    t=a['time_s'];v=a['valid'];onset={};init={};sep={}
    for mode in ('off','on'):
        with pin(P/f'full_{mode}_receiver/receiver/online_states.csv').open() as f:
            init[mode]=next(csv.DictReader(f))
    assert all(init['on'][x]==init['off'][x] for x in ('timestamp_ns','px','py','pz','qx','qy','qz','qw'))
    for role in ('fusion_online','fusion_delayed'):
        raw=np.linalg.norm(z['on_'+role+'_position']-z['off_'+role+'_position'],axis=1);raw[~v]=np.nan;sep[role]=raw
        onset[role]={}
        for threshold in (.01,.02,.05,.1):
            selected=[ix for ix in runs(v&(raw>threshold)) if t[ix[-1]]-t[ix[0]]>=5]
            onset[role][str(threshold)]=[dict(start_s=float(t[ix[0]]),end_s=float(t[ix[-1]])) for ix in selected]
    windows=[]
    for lo,hi in ((0,10),(50,70),(138,146),(170,178),(275,290),(298,304),(310,350)):
        ix=v&(t>=lo)&(t<hi);row=dict(start_s=lo,end_s=hi,n=int(ix.sum()),curves={},revision_peaks={})
        for key in r['common']:
            row['curves'][key]=dict(saved_global_error=stats(a[key+'_norm'][ix]),relative5_m=stats(a[key+'_relative5_m'][ix]))
        for mode in ('off','on'):
            rs=[e for e in revisions[mode] if lo<=e['time_s']<hi];row['revision_peaks'][mode]=max(rs,key=lambda e:e['previous_revision_m'])
        windows.append(row)
    late={};fullmatrix={}
    for role in ('glim_hesai','kimera_front'):
        late[role]={};fullmatrix[role]={}
        for mode in ('off','on'):
            c2=bc[f'{role}_{mode}_age2.5_covariance'];ct=bc[f'{role}_{mode}_terminal_covariance'];e2=bc[f'{role}_{mode}_age2.5_residual'];et=bc[f'{role}_{mode}_terminal_residual']
            vals=np.array([eigvalsh(y,x) for x,y in zip(c2,ct)])
            late[role][mode]=dict(paired_trace_ratio=stats(np.trace(ct[:,3:,3:],axis1=1,axis2=2)/np.trace(c2[:,3:,3:],axis1=1,axis2=2)),mean_trace_ratio=float(np.trace(ct[:,3:,3:],axis1=1,axis2=2).mean()/np.trace(c2[:,3:,3:],axis1=1,axis2=2).mean()),translation_reference_RMS_age2p5_m=float(np.sqrt(np.mean(np.sum(e2[:,3:]**2,axis=1)))),translation_reference_RMS_terminal_m=float(np.sqrt(np.mean(np.sum(et[:,3:]**2,axis=1)))),fraction_all_six_directions_contract=float(np.mean(vals.max(axis=1)<1-1e-8)))
        vals=np.array([eigvalsh(y,x) for x,y in zip(bc[f'{role}_off_terminal_covariance'],bc[f'{role}_on_terminal_covariance'])])
        fullmatrix[role]=dict(generalized_eigenvalues=stats(vals.ravel()),fraction_all_six_on_below_off=float(np.mean(vals.max(axis=1)<1-1e-8)))
    shares={}
    for role in ('fusion_online','fusion_delayed'):
        shares[role]={f'{lo}-{hi}s':sum(q['net_extra_energy_fraction'] for q in d['fusion'][role]['bins'] if lo<=q['start_s']<hi) for lo,hi in ((0,50),(50,140),(140,190),(190,280),(280,360))}
    for path,digest in PINS.items():assert sha(path)==digest
    save(H/'findings.json',dict(passed=True,same_initial_online_pose=init,onset_same_initial_frame=onset,windows=windows,late_maturation=late,full_matrix_on_off=fullmatrix,net_extra_energy_shares=shares,input_sha256=PINS,caveats=['Same-initial-frame separation is not an error against truth; no fitted alignment is used.','Rolling RMS uses51samples spanning5s; overlapping windows are not independent.','Cohort terminal means are actual last-accepted absorption versions, not Method1 trajectory differences.']))
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'figure.dpi':140})
    off='#1670a4';on='#d45a22';fig,ax=plt.subplots(3,1,figsize=(12,9),sharex=True,layout='constrained')
    fig.suptitle('EIG-2: where CBS-ON fusion loses accuracy',fontsize=17,fontweight='bold')
    for i,role in enumerate(('fusion_online','fusion_delayed')):
        for mode,color in [('off',off),('on',on)]:
            ax[i].plot(t,a[mode+'_'+role+'_norm'],color=color,alpha=.16,lw=.6)
            ax[i].plot(t,a[mode+'_'+role+'_rms5'],color=color,lw=1.8,label=f'CBS {mode.upper()} — 5 s RMS')
        ax[i].set_ylabel('Position error (m)');ax[i].set_title(('ONLINE: saved global SE(3) alignment','Seven-second-finalized: discrepancy persists')[i],loc='left')
        ax[i].legend(loc='upper right',fontsize=9,ncol=2);ax[i].set_ylim(0,.44 if i==0 else .29)
    ax[0].annotate('Largest sustained ON peak\n299–303 s',xy=(300.8,.2167),xytext=(235,.34),arrowprops={'arrowstyle':'->','color':'#333'},fontsize=10)
    ax[0].annotate('OFF has the larger\nsingle-point spike',xy=(144.1,.4051),xytext=(170,.34),arrowprops={'arrowstyle':'->','color':'#333'},fontsize=10)
    ax[2].plot(t,sep['fusion_online'],color='#7256a0',lw=.8,label='ONLINE OFF–ON separation')
    ax[2].plot(t,sep['fusion_delayed'],color='#404040',lw=1.2,label='Finalized OFF–ON separation')
    ax[2].axhline(.05,color='#888',ls='--',lw=.8);ax[2].set_ylabel('Separation (m)')
    ax[2].set_title('Same initial pose; no whole-trajectory fit — separation stays >5 cm from ~53 s',loc='left')
    ax[2].legend(loc='upper left',fontsize=9);ax[2].set_xlabel('Seconds from selected EIG-2 window start (bag offset +5.7 s)')
    for x in ax:
        x.set_xlim(0,355.02)
        for lo,hi in ((50,70),(138,146),(170,178),(275,310)):x.axvspan(lo,hi,color='#888',alpha=.075)
    fig.savefig(H/'fusion_error_timeline.png');plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(12,6.5),sharex=True,layout='constrained')
    fig.suptitle('Kimera: matched physical intervals expose late covariance tightening',fontsize=15,fontweight='bold')
    tm=bc['kimera_front_end_s'];centers=np.arange(5,350,10)
    for mode,color in [('off',off),('on',on)]:
        c=bc[f'kimera_front_{mode}_terminal_covariance'];c2=bc[f'kimera_front_{mode}_age2.5_covariance'];e=bc[f'kimera_front_{mode}_terminal_residual']
        ratio=np.trace(c[:,3:,3:],axis1=1,axis2=2)/np.trace(c2[:,3:,3:],axis1=1,axis2=2)
        yy=[];ee=[]
        for mid in centers:
            ix=(tm>=mid-5)&(tm<mid+5);yy.append(np.median(ratio[ix]) if ix.any() else np.nan);ee.append(1e3*np.sqrt(np.mean(np.sum(e[ix,3:]**2,axis=1))) if ix.any() else np.nan)
        ax[0].plot(centers,yy,marker='.',color=color,label=mode.upper());ax[1].plot(centers,ee,marker='.',color=color,label=mode.upper())
    ax[0].axhline(1,color='#888',ls='--');ax[0].set_ylabel('Median trace ratio\nterminal / age 2.5 s');ax[0].legend(ncol=2);ax[0].set_ylim(0,2.1)
    ax[1].set_yscale('log');ax[1].set_ylabel('Terminal relative residual\nRMS vs IE-TC (mm, log scale)');ax[1].set_xlabel('Belief endpoint time from selected window start (s)')
    for x in ax:
        x.set_xlim(0,355.02)
        for lo,hi in ((50,70),(138,146),(170,178),(275,310)):x.axvspan(lo,hi,color='#888',alpha=.075)
    fig.savefig(H/'kimera_local_covariance.png');plt.close(fig)
    print(json.dumps(dict(onset=onset,late=late,extra_energy=shares),indent=2))
if __name__=='__main__':main()
