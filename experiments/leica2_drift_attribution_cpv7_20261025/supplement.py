"""Directional check and same-start horizon control; saved data only."""
import gzip
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import spearmanr
import analyze as a


def rms(v): return float(np.sqrt(np.mean(np.square(v))))


def main():
    out=a.H/'outputs';dest=out/'directional_checks.json';assert not dest.exists()
    hashes={str(p):a.g.sha(p) for p in (out/'metrics.json',out/'time_series.npz',out/'peer_records.jsonl.gz',Path(__file__),a.H/'analyze.py')}
    m=json.loads((out/'metrics.json').read_text());z=np.load(out/'time_series.npz')
    valid=z['valid'];time=z['time_s'];ref=a.g.Reference(a.GT)
    # Fix the START cohort to the strict20s continuous support mask for all horizons.
    starts=a.intervals(valid,200)
    horizon={};localization={};orientation={}
    for mode in a.MODES:
        for role in a.ROLES:
            key=mode+'_'+role;v={}
            for h in (1,5,10,20):
                rr=z[key+f'_rpe_{h}s'];keep=np.isin(rr[:,0].astype(int),starts)
                assert keep.sum()==len(starts)
                v[h]=dict(n=int(keep.sum()),translation_rms_m=rms(rr[keep,2]),rotation_rms_deg=rms(rr[keep,3]))
            horizon[mode+'/'+role]=v
        angles=z[mode+'_kimera_angles']
        orientation[mode]=dict(roll_rms_deg=rms(angles[valid,0]),pitch_rms_deg=rms(angles[valid,1]),heading_rms_deg=rms(angles[valid,2]))
        pieces=np.split(np.flatnonzero(valid),np.flatnonzero(np.diff(np.flatnonzero(valid))>1)+1)
        bins=[]
        for t in range(0,390,30):
            # Signed change accumulated only WITHIN supported segments, no fabricated gap steps.
            changes=[];spans=[]
            for piece in pieces:
                ii=piece[(time[piece]>=t)&(time[piece]<t+30)]
                if len(ii)>1:
                    changes.append(float(angles[ii[-1],2]-angles[ii[0],2]));spans.append(float(time[ii[-1]]-time[ii[0]]))
            bins.append(dict(start_s=t,supported_span_s=sum(spans),heading_change_deg=sum(changes)))
        localization[mode]=bins
    rr=[json.loads(line) for line in gzip.open(out/'peer_records.jsonl.gz','rt')]
    look={mode:{(x['a'],x['b']):x for x in rr if x['mode']==mode} for mode in a.MODES[1:]}
    keys=set(look['original_on'])&set(look['rotation10_on'])
    direction={};bin_data={}
    for mode in a.MODES[1:]:
        prepared=[]
        for key in sorted(keys):
            x=look[mode][key];ra=ref.at(x['a']);rb=ref.at(x['b'])
            r=dict(time_s=x['time_s'])
            for name,mufield,cf in [('incoming','incoming_mu','applied_cov'),('pre','pre_mu','pre_cov'),('post','post_mu',None)]:
                mu=np.asarray(x[mufield]);T=a.g.exp(mu)
                pred=ra[:3,:3]@T[:3,:3]
                diff=R.from_matrix(pred@rb[:3,:3].T).as_rotvec()*180/np.pi
                # Explicit Euler heading confirms world-Z log component is not a naming artifact.
                yaw=(np.arctan2(pred[1,0],pred[0,0])-np.arctan2(rb[1,0],rb[0,0])+np.pi)%(2*np.pi)-np.pi
                r[name]=dict(z_deg=float(diff[2]),tilt_deg=float(np.linalg.norm(diff[:2])),heading_deg=float(yaw*180/np.pi))
                if cf:
                    # Right-local marginal angular uncertainty expressed in nominal predicted ENU axes.
                    # This is not a fully propagated residual covariance / NEES claim.
                    C=np.asarray(x[cf])[:3,:3];W=pred@C@pred.T
                    assert np.isclose(np.trace(W),np.trace(C),rtol=1e-12)
                    r[name].update(z_reported_std_deg=float(np.sqrt(W[2,2])*180/np.pi),
                                   tilt_reported_norm_deg=float(np.sqrt(W[0,0]+W[1,1])*180/np.pi))
                assert np.allclose(diff,x[name]['world_rotation_xyz_deg'],atol=1e-10)
            prepared.append(r)
        def summary(rows):
            result=dict(n=len(rows))
            for name in ('incoming','pre','post'):
                result[name]=dict(heading_rms_deg=rms([x[name]['heading_deg'] for x in rows]),
                                  vertical_component_rms_deg=rms([x[name]['z_deg'] for x in rows]),
                                  tilt_component_rms_deg=rms([x[name]['tilt_deg'] for x in rows]),
                                  signed_heading_mean_deg=float(np.mean([x[name]['heading_deg'] for x in rows])))
                if name!='post':
                    result[name].update(vertical_reported_std_rms_deg=rms([x[name]['z_reported_std_deg'] for x in rows]),
                                        tilt_reported_norm_rms_deg=rms([x[name]['tilt_reported_norm_deg'] for x in rows]),
                                        vertical_covariance_error_rank=float(spearmanr([x[name]['z_reported_std_deg'] for x in rows],[abs(x[name]['z_deg']) for x in rows]).statistic),
                                        tilt_covariance_error_rank=float(spearmanr([x[name]['tilt_reported_norm_deg'] for x in rows],[x[name]['tilt_deg'] for x in rows]).statistic))
            result['incoming_better_fraction']={dim:float(np.mean([abs(x['incoming'][dim])<abs(x['pre'][dim]) for x in rows])) for dim in ('heading_deg','tilt_deg')}
            return result
        direction[mode]=summary(prepared)
        bin_data[mode]=[dict(start_s=t,summary=summary([x for x in prepared if t<=x['time_s']<t+30])) for t in range(0,390,30) if any(t<=x['time_s']<t+30 for x in prepared)]
    # A failed single-step/outlier hypothesis can be assessed descriptively; no deletion.
    concentration={}
    for mode in a.MODES:
        ang=z[mode+'_kimera_angles'][:,2]
        ii=np.flatnonzero(valid[:-1]&valid[1:]);delta=ang[ii+1]-ang[ii]
        count=max(1,int(np.ceil(.01*len(delta))));order=np.argsort(np.abs(delta))[-count:]
        concentration[mode]=dict(n=len(delta),largest_abs_100ms_change_deg=float(np.abs(delta).max()),
                                 top1percent_abs_change_fraction=float(np.abs(delta[order]).sum()/np.abs(delta).sum()),
                                 top1percent_signed_change_deg=float(delta[order].sum()),
                                 all_supported_signed_change_deg=float(delta.sum()))
    assert all(a.g.sha(Path(p))==h for p,h in hashes.items())
    result=dict(passed=True,same_start_horizon_control=horizon,orientation_rms=orientation,
                heading_time_bins=localization,paired_directional=direction,paired_directional_bins=bin_data,
                increment_concentration=concentration,input_sha256=hashes,
                limits='Post-hoc directional explanation, not a fitted policy or isolated causal ablation. ENU Z heading; angular covariance projection is not residual covariance propagation. No cross-time covariance inferred.')
    a.save(dest,result)
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4.8))
    labels=['Incoming GLIM','Kimera pre-update']
    for ax,field,title in zip(axes,['tilt_component_rms_deg','heading_rms_deg'],['Tilt error (two horizontal rotation components)','Heading error (about vertical)']):
        x=np.arange(2)
        for j,mode in enumerate(a.MODES[1:]):
            vals=[direction[mode][name][field] for name in ('incoming','pre')]
            bars=ax.bar(x+(j-.5)*.34,vals,.34,color=a.COLORS[mode],label=a.NAMES[mode])
            ax.bar_label(bars,fmt='%.3f',padding=3,fontsize=9)
        ax.set_xticks(x,labels);ax.set_ylabel('Error RMS (degrees)');ax.set_title(title,fontsize=10)
        ax.set_ylim(0,ax.get_ylim()[1]*1.15);ax.grid(axis='y',alpha=.2)
    axes[0].legend(fontsize=8)
    fig.suptitle('Same 1,384 receiver intervals: GLIM rotation is not uniformly worse')
    fig.text(.04,.02,'Exact aligned beliefs and verified pre-states; shared-CPT7 reference. Separate marginal error components, not independent factors.',fontsize=9)
    fig.tight_layout(rect=(0,.07,1,.94));fig.savefig(out/'05_directional_errors.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in result.items() if k in ('paired_directional','orientation_rms','increment_concentration','same_start_horizon_control')},indent=2))


if __name__=='__main__':main()
