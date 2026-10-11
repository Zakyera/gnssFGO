"""Describe same-cohort late revisions and draw two static audit figures."""
from pathlib import Path
import argparse, hashlib, json, sys
sys.dont_write_bytecode=True
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.stats import spearmanr

def stats(x):
    x=np.asarray(x,float)
    return dict(n=len(x),median=float(np.median(x)),p05=float(np.quantile(x,.05)),p95=float(np.quantile(x,.95)))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);args=ap.parse_args()
    root=Path(args.root); out=root/'presentation';out.mkdir(exist_ok=False)
    summary=json.loads((root/'summary.json').read_text())
    rows=json.loads((root/'selected_records.json').read_text())
    lookup={(r['mode'],r['source'],r['from_ns'],r['to_ns'],r['age']):r for r in rows}
    groups={}; max_correlation_difference=0.
    for mode in ('off','on'):
        for source in ('glim_hesai','kimera_front'):
            keys=[k[:4] for k in lookup if k[0]==mode and k[1]==source and k[4]=='2']
            keyname=f'{mode}_{source}';groups[keyname]={}
            for ch,part in enumerate(('rotation','rho')):
                byage={}
                for age in ('first','1','2','cutoff'):
                    a=[lookup[(*k,age)] for k in keys]; first=[lookup[(*k,'first')] for k in keys]
                    s=np.array([r['sigma'][ch] for r in a]);e=np.array([r['error'][ch] for r in a])
                    sf=np.array([r['sigma'][ch] for r in first]);ef=np.array([r['error'][ch] for r in first])
                    byage[age]=dict(sigma=stats(s),error=stats(e),sigma_over_first=stats(s/sf),error_over_first=stats(e/ef))
                prev=[lookup[(*k,'2')] for k in keys];last=[lookup[(*k,'cutoff')] for k in keys]
                s=np.array([r['sigma'][ch] for r in last])/np.array([r['sigma'][ch] for r in prev])
                e=np.array([r['error'][ch] for r in last])/np.array([r['error'][ch] for r in prev])
                groups[keyname][part]=dict(ages=byage,two_to_cutoff=dict(sigma_ratio=stats(s),error_ratio=stats(e),
                    normalized_error_ratio=stats(e/s),sigma_contracts_at_least_5pct=float(np.mean(s<=.95)),
                    sigma_contracts_5pct_and_error_not_improve_5pct=float(np.mean((s<=.95)&(e>=.95))),
                    fraction_error_improves_5pct=float(np.mean(e<=.95))))
                selected=[r for r in rows if r['mode']==mode and r['source']==source and r['age']=='cutoff']
                actual=spearmanr([r['sigma'][ch] for r in selected],[r['error'][ch] for r in selected]).statistic
                expected=summary['groups'][f'{mode}_{source}_cutoff'][part]['ranking']['spearman']
                max_correlation_difference=max(max_correlation_difference,abs(actual-expected))
    assert max_correlation_difference<1e-12
    report=dict(same_age2_cohort=groups,independent_scipy_spearman_max_difference=max_correlation_difference)
    with (out/'late_revisions.json').open('x') as f:json.dump(report,f,indent=2)
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
    fig,axs=plt.subplots(2,2,figsize=(11.8,7.5),layout='constrained')
    for i,source in enumerate(('glim_hesai','kimera_front')):
        for j,part in enumerate(('rotation','rho')):
            ax=axs[i,j]; scale=180/np.pi if j==0 else 1000
            for mode,col in [('off','#2369b0'),('on','#c84a27')]:
                g=summary['groups'][f'{mode}_{source}_cutoff'][part]; q=g['ranking']['quintiles']
                ax.plot(range(1,6),[v['error']['median']*scale for v in q],'-o',color=col,label=f'CBS {mode.upper()}')
            ax.set(title=f'{"GLIM" if i==0 else "Kimera"}: {part}',xticks=range(1,6),
                   xlabel='Reported uncertainty quintile (1 = smallest)',
                   ylabel='Median reference discrepancy '+('degrees' if j==0 else 'mm'))
            ax.legend()
    fig.suptitle('Does higher reported uncertainty identify larger relative errors?\nLatest received beliefs; descriptive IE-TC comparison, not covariance calibration',fontsize=13)
    fig.savefig(out/'uncertainty_ranking.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(11.8,7.5),layout='constrained')
    for i,source in enumerate(('glim_hesai','kimera_front')):
        for j,part in enumerate(('rotation','rho')):
            ax=axs[i,j]
            for mode,style in [('off','--'),('on','-')]:
                g=groups[f'{mode}_{source}'][part]['ages']
                for field,col,label in [('sigma_over_first','#2369b0','uncertainty'),('error_over_first','#c84a27','discrepancy')]:
                    ax.plot(range(4),[g[a][field]['median'] for a in ('first','1','2','cutoff')],style+'o',color=col,label=f'{mode.upper()} {label}')
            ax.set(title=f'{"GLIM" if i==0 else "Kimera"}: {part}',xticks=range(4),xticklabels=['First','Age 1 s','Age 2 s','Last received'],
                   ylabel='Median per-belief ratio to first version',ylim=(0,1.1))
            ax.legend(fontsize=8)
    fig.suptitle('Revisions: reported sqrt(trace) and reference discrepancy\nIdentical age-2-qualified beliefs at every column; last-received age varies',fontsize=13)
    fig.savefig(out/'revision_evolution.png',dpi=160);plt.close(fig)
    print(json.dumps({k:{p:v[p]['two_to_cutoff'] for p in ('rotation','rho')} for k,v in groups.items()},indent=2))

if __name__=='__main__':main()
