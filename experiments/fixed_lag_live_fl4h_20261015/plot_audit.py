"""Display retained observations only; do not rerun an estimator."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/'
          'gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/audit_fl4h')
d=json.loads((ROOT/'mechanism_summary.json').read_text())
details=json.loads((ROOT/'factor_details_v2.json').read_text())
fig,ax=plt.subplots(3,1,figsize=(11,9),sharex=True,constrained_layout=True)
for mode,color in [('off','#2471a3'),('on','#c0392b')]:
    rows=[r for r in d[mode]['rows'] if 'native_step_translation_error_m' in r]
    t=np.array([r['time'] for r in rows]);v=np.array([r['native_step_translation_error_m'] for r in rows])
    # Trailing 25-keyframe RMS, not an independently rescored trajectory.
    sm=np.array([np.sqrt(np.mean(v[max(0,i-24):i+1]**2)) for i in range(len(v))])
    ax[0].plot(t,sm,color=color,label=f'Kimera {mode.upper()}: trailing 25-edge RMS')
rows=[r for r in d['on']['rows'] if 'aligned_error_m' in r]
t=np.array([r['time'] for r in rows]);v=np.array([r['aligned_error_m'] for r in rows])
ax[0].plot(t,[np.sqrt(np.mean(v[max(0,i-24):i+1]**2)) for i in range(len(v))],color='#229954',label='Incoming aligned GLIM: same-interval error RMS')
ax[0].set(ylabel='Relative translation error (m)',title='Saved ETH1 120 s audit — relative errors against IE-TC, not trajectory ATE')
ax[0].legend(fontsize=9,loc='upper left')
for key,color,label in [('prior_injected_edge_error_m','#7f8c8d','Target edge before update'),('after_injected_edge_error_m','#c0392b','Target edge after update / recovery')]:
    ax[1].semilogy(t,[max(r[key],1e-5) for r in rows],'.',ms=3,color=color,label=label)
guards=sorted(set(g['time'] for g in details['guards'] if 'active_window_rollback,' in g['text']))
ax[1].plot(guards,[8]*len(guards),'v',color='black',ms=5,label='Rollback transaction (marker only)')
ax[1].set(ylabel='Target-edge error (m, log)');ax[1].legend(fontsize=9,loc='upper left')
samples=list(details['sampled_existing_family_costs'].values())
for family,color in [('imu','#884ea0'),('cbs_external','#229954'),('smart_stereo','#d68910')]:
    ss=[s for s in samples if family in s['existing']]
    ax[2].semilogy([s['time'] for s in ss],[s['existing'][family]['cost'] for s in ss],'.-',label=family,color=color)
ax[2].set(ylabel='Recorded family cost (log)',xlabel='Seconds from frozen bag clock 1727775007; all saved outputs included',title='Sampled existing-graph pre-update costs; NOT a full-objective convergence test')
ax[2].legend(fontsize=9,loc='upper left')
for a in ax:a.grid(True,alpha=.25);a.set_xlim(0,121)
target=ROOT/'failure_timeline.png';assert not target.exists();fig.savefig(target,dpi=160)
