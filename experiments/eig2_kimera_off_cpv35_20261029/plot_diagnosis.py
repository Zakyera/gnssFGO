"""Static scientific figure from retained costs/states; no trajectory edits."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
P=W/'runs/eig2_kimera_off_cpv35_20261029/capture'
a=np.loadtxt(S/'objective.csv',delimiter=',')
e={d['k']:d for d in json.loads((H.parent/'eig2_kimera_jitter_cpv34_20261029/velodyne_eig2_off_events.json').read_text())}
sn=json.loads((H/'snapshots_final.json').read_text())['updates']
ids=np.array(sorted(e));t=np.array([e[k]['time_s'] for k in ids]);valid=[r for r in sn if r['update'] in e]
st=np.array([e[r['update']]['time_s'] for r in valid]);ss=[r['stages']['after_solve'] for r in valid]
fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,layout='constrained')
axes[0].semilogy(t,np.maximum(a[ids,7],1e-4),label='Original OFF: native IMU cost',color='#9b1c31')
axes[0].semilogy(t,np.maximum(a[ids,9],1e-4),label='Original OFF: marginal-prior cost',color='#405b91')
axes[0].set_ylabel('Weighted cost\n(log scale)');axes[0].legend(loc='upper left')
axes[1].plot(t,[e[k]['previous_revision_m'] for k in ids],label='Revision of previous pose',color='#d04f38',lw=1)
axes[1].plot(t,[e[k]['same_update_motion_m'] for k in ids],label='Same-update latest motion',color='#26785d',lw=1)
axes[1].set_ylabel('Distance (m)');axes[1].legend(loc='upper left')
for part,color,label in [('position_m','#2564aa','Position residual (m)'),('velocity_mps','#ac5a13','Velocity residual (m/s)')]:
    axes[2].plot(st,[s['imu_residuals'][part]['max'] for s in ss],'o',ms=4,color=color,label=label)
axes[2].set_yscale('symlog',linthresh=1e-5)
axes[2].set_ylabel('Maximum native IMU\nresidual in window');axes[2].legend(loc='upper left')
axes[3].plot(st,[s['anchor_rotation_deg']['max'] for s in ss],'o',ms=4,label='Max pose rotation from its linearization anchor',color='#72579c')
axes[3].set_ylabel('Angle (degrees)');axes[3].set_xlabel('EIG-2 sensor time from original matched start (s)');axes[3].legend(loc='upper left')
for ax in axes:
    ax.axvspan(266,273,color='grey',alpha=.15);ax.axvline(300,color='black',ls='--',alpha=.4);ax.grid(alpha=.2)
axes[0].set_title('EIG-2 Kimera CBS OFF: output revisions and internal IMU inconsistency\nHesai and Velodyne originals identical; private diagnostic replays first 300 s')
fig.savefig(H/'off_diagnosis_v2.png',dpi=150)
print(H/'off_diagnosis_v2.png')
