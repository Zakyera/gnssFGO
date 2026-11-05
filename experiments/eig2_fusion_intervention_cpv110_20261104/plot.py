"""Static scientific intervention plots; no solver or scoring calls."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from common import *
assert read(H/'report_verification.json')['passed']
assert not (H/'regional_intervention.png').exists()
r=read(H/'results.json');z=np.load(H/'evaluation/comparison.npz');epoch=read(P/'input_audit.json')['epoch_ns'];t=(z['timestamp_ns']-epoch)*1e-9;valid=z['common_valid']
def smooth(x):
    out=np.full(len(x),np.nan);ids=np.flatnonzero(valid)
    for ix in np.split(ids,np.flatnonzero(np.diff(ids)>1)+1):
        if len(ix)>=51:out[ix[25:-25]]=np.sqrt(np.convolve(x[ix]**2,np.ones(51)/51,'valid'))
    return out
colors={'baseline':'#404040','hold_cov':'#1479b8','hold_mean':'#d7691d','hold_both':'#6e53a3'}
labels={'baseline':'Original ON','hold_cov':'Hold late covariance','hold_mean':'Hold late means','hold_both':'Hold both'}
plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'figure.dpi':145})
fig,axes=plt.subplots(2,2,figsize=(12,7.5),layout='constrained')
fig.suptitle('EIG-2 saved-stream test: late mean versus covariance revisions',fontsize=15,fontweight='bold')
for row,product in enumerate(('online','delayed')):
    for col,(lo,hi) in enumerate(((45,85),(265,335))):
        ax=axes[row,col]
        for mode in MODES:
            error=np.linalg.norm(z[mode+'_'+product+'_frozen_error'],axis=1)
            ax.plot(t,smooth(error),color=colors[mode],label=labels[mode],lw=1.6)
        ax.set_xlim(lo,hi);ax.axvspan(50 if col==0 else 275,70 if col==0 else 310,color='#888',alpha=.12)
        ax.set_title(('ONLINE' if row==0 else 'Seven-second-finalized')+f' | {lo}–{hi} s',loc='left')
        ax.set_xlabel('Seconds from selected window start');ax.set_ylabel('Position-error 5 s RMS (m)')
axes[0,0].legend(fontsize=8)
fig.supxlabel('Frozen original-ON alignment for every curve; shading = eligible belief endpoint windows. No new live capture.',fontsize=9)
fig.savefig(H/'regional_intervention.png');plt.close(fig)
fig,ax=plt.subplots(figsize=(12,4.5),layout='constrained')
for mode in MODES:
    error=np.linalg.norm(z[mode+'_online_own_error'],axis=1)
    label=f"{labels[mode]}: ATE {r['common'][mode+'_online']['ATE_RMSE_m']:.4f} m"
    ax.plot(t,smooth(error),color=colors[mode],lw=1.5,label=label)
for lo,hi in WINDOWS:ax.axvspan(lo,hi,color='#888',alpha=.1)
ax.set_xlim(0,355.02);ax.set_title('ONLINE fusion — each curve uses its own whole-sequence SE(3) alignment',loc='left');ax.set_xlabel('Seconds from selected EIG-2 window start');ax.set_ylabel('Position-error 5 s RMS (m)');ax.legend(fontsize=9,ncol=2)
fig.savefig(H/'whole_trajectory_errors.png');plt.close(fig)
