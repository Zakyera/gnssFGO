"""Static scientific display of saved direct-EVO results; no new scores."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from evo.tools import file_interface
from common import *
assert read(H/'report_verification.json')['passed']
r=read(H/'results.json');data=np.load(H/'evaluation/comparison.npz');valid=data['common_valid']
epoch=read(P/'input_audit.json')['epoch_ns'];t=(data['timestamp_ns'][valid]-epoch)*1e-9
g=data['reference_position'][valid];cuts=np.split(np.arange(len(t)),np.flatnonzero(np.diff(t)>.10000001)+1)
fig,axs=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
colors={'baseline':'#d37522','frozen_kimera':'#176b99'}
for col,product in enumerate(('online','delayed')):
    ax=axs[0,col]
    for i,ii in enumerate(cuts):ax.plot(g[ii,0],g[ii,1],color='#444444',lw=1.6,label='IE-TC reference' if i==0 else None)
    for mode in colors:
        key=mode+'_'+product;align=r['common'][key]['alignment']
        pos=data[key+'_position'][valid]@np.asarray(align['rotation']).T+align['translation']
        label='Original covariance revisions' if mode=='baseline' else 'First-accepted Kimera covariance held'
        for i,ii in enumerate(cuts):ax.plot(pos[ii,0],pos[ii,1],lw=1.0,alpha=.85,color=colors[mode],label=label if i==0 else None)
        result=file_interface.load_res_file(H/'evaluation/common'/(key+'_ape.zip'))
        errors=result.np_arrays['error_array']
        for i,ii in enumerate(cuts):axs[1,col].plot(t[ii],errors[ii],lw=.8,color=colors[mode],label=label if i==0 else None)
    ax.set_title('ONLINE' if product=='online' else 'DELAYED: 7-second fixed-lag finalized')
    ax.set_xlabel('Aligned reference X (m)');ax.set_ylabel('Aligned reference Y (m)');ax.axis('equal');ax.grid(alpha=.2)
    axs[1,col].set_xlabel('ETH1 elapsed time (s)');axs[1,col].set_ylabel('EVO aligned position error (m)');axs[1,col].grid(alpha=.2)
axs[0,0].legend(fontsize=8,loc='best')
fig.suptitle('CP-V106: saved ON stream, covariance-only intervention\nSame 2,966-sample mask; independent SE(3) alignment per curve; gaps retained',fontsize=12)
fig.savefig(H/'covariance_intervention.png',dpi=160);plt.close(fig)
