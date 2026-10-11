"""Summarize verified retained-log results; no estimator changes."""
import argparse,json,re,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def dump(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False)

def stats(x):
    x=np.asarray(x,float)
    return dict(n=len(x),median=float(np.median(x)),p05=float(np.quantile(x,.05)),
                p95=float(np.quantile(x,.95)),min=float(x.min()),max=float(x.max()))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('analysis');args=ap.parse_args()
    p=Path(args.analysis);out=p/'presentation';out.mkdir(exist_ok=False)
    S=json.loads((p/'summary.json').read_text());E=json.loads((p/'edges.json').read_text())
    U=json.loads((p/'updates.json').read_text());T=json.loads((p/'steps.json').read_text())
    matched=[(e['i'],e['j']) for e in E['off'] if e['retired']]
    assert matched==[(e['i'],e['j']) for e in E['on'] if e['retired']] and len(matched)==556
    onset={};data={};flags={};code={}
    for mode in ('off','on'):
        es=[e for e in E[mode] if e['retired']]
        # All these physical intervals have 25 versions, avoiding changing cohorts.
        assert all(len(e['evolution'])==25 for e in es)
        changes=np.array([[np.array(b['sigma'])/a['sigma'] for a,b in zip(e['evolution'],e['evolution'][1:])] for e in es])
        assert changes.shape==(556,24,2)
        data[mode]=changes
        onset[mode]={}
        for u in (24,25,26,27,28,29,30):
            row=next(x for x in U[mode] if x['u']==u)
            # Same 15 oldest-eligible physical edges at u27 and u28, both modes.
            selected=[x for x in T[mode] if x['u']==u and 3<=x['i']<=17]
            onset[mode][u]=dict(update=row,selected_edges=[(x['i'],x['j']) for x in selected],
                rotation=stats([x['sigma_ratio'][0] for x in selected]),
                rho=stats([x['sigma_ratio'][1] for x in selected]))
        logs=[Path(x) for x in json.loads((p/'inputs_before.json').read_text()) if 'roslaunch-' in x and f'{"full_off" if mode=="off" else "bounded120_on"}_source' in x]
        assert len(logs)==1
        relevant=['cbs_odom_factor_mode','cbs_odom_covariance_mode','cbs_live_revision_enable',
            'cbs_use_persistent_bpsam_sidecar_for_belief_covariance','cbs_active_factor_diagnostic_enable',
            'cbs_active_factor_detail_diagnostic_enable','cbs_relative_covariance_matrix_dump_enable',
            'kimera_factor_graph_audit_enable','kimera_bad_update_audit_dir','cbs_health_aware_enable']
        txt=logs[0].read_text()
        flags[mode]={k:sorted(set(re.findall('--'+k+r"=([^'\s,]*)",txt))) for k in relevant}
    sourceon={e['i']:e for e in E['on'] if e['retired'] and e['first_u']>=40}
    order=sorted(sourceon,key=lambda i:sourceon[i]['rotation']['last_over_age2'])
    ids=[order[0],order[len(order)//2],order[-1]]
    reps={str(i):{m:next(e for e in E[m] if e['i']==i) for m in E} for i in ids}
    last={m:{part:dict(root_ratio=stats(data[m][:,-1,j]),
        fraction_drop5=float(np.mean(data[m][:,-1,j]<.95))) for j,part in enumerate(('rotation','rho'))} for m in data}
    brief=dict(matched_uncensored_edges=556,onset=onset,representative_edge_start_ids=ids,
        selection='strongest, median, weakest ON late rotation contraction among retired edges first published at u>=40; no reference-error selection',
        last_step=last,retained_launch_flags=flags)
    dump(out/'detail.json',brief);dump(out/'representative_edges.json',reps)
    fig,ax=plt.subplots(2,2,figsize=(12.5,8.5),constrained_layout=True)
    colors={'off':'#3478b4','on':'#d16a26'}
    for j,part in enumerate(('rotation','rho')):
        for mode,a in data.items():
            med=np.median(a[:,:,j],axis=0);lo,hi=np.quantile(a[:,:,j],[.25,.75],axis=0)
            ax[0,j].plot(np.arange(1,25),100*(1-med),label='CBS-'+mode.upper(),color=colors[mode])
            ax[0,j].fill_between(np.arange(1,25),100*(1-hi),100*(1-lo),alpha=.13,color=colors[mode])
        ax[0,j].set(xlabel='Updates since first publication (last valid = 24)',ylabel='Uncertainty decrease per update (%)',
                    title=('Rotation' if j==0 else 'Translation tangent (rho)')+' — same 556 retired beliefs')
        ax[0,j].axhline(0,color='gray',lw=.8);ax[0,j].grid(alpha=.2);ax[0,j].legend()
        for mode in ('off','on'):
            u=[x for x in U[mode] if 20<=x['u']<=40]
            ax[1,j].plot([x['u'] for x in u],[100*(1-x[part]['late_median_ratio']) for x in u],'.-',label='CBS-'+mode.upper(),color=colors[mode])
        ax[1,j].axvline(26,color='gray',ls=':',label='First pose retirement (x0)')
        ax[1,j].axvline(28,color='#684d96',ls='--',label='First CBS-connected endpoint retirement (x2)')
        ax[1,j].axhline(0,color='gray',lw=.8);ax[1,j].grid(alpha=.2)
        ax[1,j].set(xlabel='Backend update ID',ylabel='Median late-belief uncertainty decrease (%)',title='Onset: late = after 2 s since first arrival')
    ax[1,0].legend(fontsize=7,loc='upper right');ax[1,1].legend(fontsize=7,loc='lower right')
    fig.suptitle('ETH1 retained Kimera logs: covariance contraction follows window age\nUncertainty = sqrt(trace of 3x3 marginal); shaded range = interquartile, not confidence interval',fontsize=13)
    fig.savefig(out/'window_age_and_onset.png',dpi=150,metadata={'Software':'retained-log audit'})
    plt.close(fig)
    print(json.dumps(dict(last_step=last,representative_ids=ids,flags=flags),indent=2))

if __name__=='__main__':main()
