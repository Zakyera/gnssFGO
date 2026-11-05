"""Paired trace presentation and delayed-age diagnostic; no replays."""
import json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
C=H.parent/'leica2_sensor_jitter_audit_cpv21_20261027'
norm=lambda x:np.linalg.norm(x,axis=-1)
cache=np.load(C/'timeseries.npz')
fig,axes=plt.subplots(3,2,figsize=(14,11),constrained_layout=True)
out={'cases':{},'input_sha256':{}}
def pin(p):
    with p.open('rb') as f:out['input_sha256'][str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
for col,name in enumerate(('hesai','velodyne')):
    a=json.loads(pin(H/f'{name}_analysis.json').read_text())
    cycles=json.loads(pin(H/f'{name}_cycles.json').read_text())
    windows=json.loads(pin(H/f'{name}_windows.json').read_text())
    pub=np.load(pin(H/f'{name}_publication_states.npz'))
    t=cache[name+'_time_s'];op=cache[name+'_online'];fp=cache[name+'_finalized'];gp=cache[name+'_reference']
    sample_t=np.array([c['time_s'] for c in cycles])
    colors={f'glim_{name}_insert':'#198b53','kimera_front_insert':'#bd7b00',
            f'glim_{name}_replace':'#83b985','kimera_front_replace':'#d9b67c',
            'retire':'#7544a5','support_add_adjustment':'#444444'}
    for g,color in colors.items():
        axes[0,col].plot(sample_t,[100*norm(c['vectors'][g]) for c in cycles],label=g.replace('_',' '),color=color,lw=.8)
    axes[0,col].set(title=name.upper()+' CBS OFF',ylabel='Net update movement per cycle (cm)')
    for pts,label,color in [(op,'Connected ONLINE','#d12e44'),(gp,'IE-TC reference','#222222')]:
        axes[1,col].plot(t[1:],100*norm(np.diff(pts,axis=0)),label=label,color=color,lw=.8)
    axes[1,col].plot([w['time_s'] for w in windows],[100*w['simultaneous_last_edge_m'] for w in windows],label='Newest edge in current window',color='#3d80c9',lw=.8)
    axes[1,col].set(ylabel='100 ms edge length (cm)')
    for k,label,color in [('stitched_online_window_path_m','Stitched ONLINE','#d12e44'),
                          ('simultaneous_window_path_m','Simultaneous revised window','#3d80c9'),
                          ('reference_window_path_m','IE-TC reference','#222222')]:
        axes[2,col].plot([w['time_s'] for w in windows],[w[k] for w in windows],label=label,color=color,lw=.9)
    axes[2,col].set(ylabel='Path in same retained interval (m)',xlabel='Seconds from capture start')
    for ax in axes[:,col]:ax.set_xlim(0,60);ax.grid(alpha=.2);ax.legend(fontsize=8)
    positions={(int(new),int(i)):p for new,i,p in zip(pub['newest'],pub['index'],pub['points'])}
    ages=(0,1,2,3,5,10)
    idx=np.array([i for i in range(len(t)) if 0<t[i]<=59 and all((i+age,i) in positions for age in ages)])
    assert len(idx)>580 and np.all(np.diff(idx)==1)
    products={}
    for age in ages:
        x=np.array([positions[(i+age,i)] for i in idx]);ref=gp[idx]
        if age==0:assert np.max(norm(x-op[idx]))<1e-10
        delta=np.diff(x,axis=0)-np.diff(ref,axis=0)
        products[str(age/10)]={'rows':len(x),'path_m':float(norm(np.diff(x,axis=0)).sum()),
                              'step_vector_error_rms_m':float(np.sqrt(np.mean(norm(delta)**2))),
                              'position_rmse_fixed_fit_m':float(np.sqrt(np.mean(norm(x-ref)**2)))}
    cos=[]
    for c in cycles:
        if c['counts'].get('kimera_front_insert',0) and c['counts'].get(f'glim_{name}_insert',0):
            g=np.array(c['vectors'][f'glim_{name}_insert']);k=np.array(c['vectors']['kimera_front_insert'])
            if norm(g)*norm(k)>1e-15:cos.append(float(g@k/(norm(g)*norm(k))))
    out['cases'][name]={'age_cohort_times_s':[float(t[idx[0]]),float(t[idx[-1]])],
       'reference_path_m':float(norm(np.diff(gp[idx],axis=0)).sum()),'age_products':products,
       'mixed_cycle_source_update_cosine':{'n':len(cos),'median':float(np.median(cos)),'opposing_fraction':float(np.mean(np.array(cos)<0))},
       'mean_simultaneous_window_length_m':float(np.mean([w['simultaneous_window_path_m'] for w in windows if w['time_s']>7.5])),
       'mean_reference_window_length_m':float(np.mean([w['reference_window_path_m'] for w in windows if w['time_s']>7.5])),
       'mean_stitched_window_length_m':float(np.mean([w['stitched_online_window_path_m'] for w in windows if w['time_s']>7.5]))}
# Shared axis scales across sensor columns, without hiding maxima.
for row in range(3):
    upper=0.
    for ax in axes[row]:
        for line in ax.lines:
            x,y=np.asarray(line.get_xdata()),np.asarray(line.get_ydata())
            upper=max(upper,float(y[(x>=0)&(x<=60)].max()))
    for ax in axes[row]:ax.set_ylim(0,upper*1.05)
fig.suptitle('Same receiver, unchanged settings: first-minute operation trace\nExact original-output parity required. Component magnitudes are not additive path-length shares.')
fig.savefig(H/'paired_update_trace.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(12,4.5),constrained_layout=True)
for col,name in enumerate(('hesai','velodyne')):
    z=out['cases'][name]['age_products'];x=np.array([float(k) for k in z])
    axs[col].plot(x,[100*z[str(v)]['step_vector_error_rms_m'] for v in x],'o-',color='#d43f57')
    axs[col].set(title=name.upper()+' CBS OFF',xlabel='Additional age when the SAME state is read (s)',ylabel='100 ms displacement-vector RMS (cm)',ylim=(0,6));axs[col].grid(alpha=.2)
fig.suptitle('Delayed correction is not an improvement of age-zero output\nSame physical states, original rigid fits; no smoothing or changed weights')
fig.savefig(H/'paired_state_age.png',dpi=150);plt.close(fig)
pin(Path(__file__));pin(C/'timeseries.npz')
with (H/'comparison.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out['cases'],indent=2))
