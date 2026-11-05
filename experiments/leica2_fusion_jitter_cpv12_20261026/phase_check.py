"""Supplemental phase association and independent output checks; no replay."""
import csv
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation
from evo.core.geometry import umeyama_alignment
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import analyze as a

H=Path(__file__).resolve().parent
assert not (H/'phase_checks.json').exists()
r=json.loads((H/'results.json').read_text());z=np.load(H/'timeseries.npz')
pins={str(p):a.sha(p) for p in [Path(__file__),H/'results.json',H/'timeseries.npz',
 a.P/'protected_manifest.json',a.P/'receiver_cpk19.sh',
 Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/build_live.json'),
 Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live'),
 a.E/'fixed_lag_slots_fl3i_20261014/NativeSmoother.h',
 a.E/'velodyne_full_campaign_cpv4_20261024/leica2/RESULTS_REPORT.txt']}
manifest=json.loads((a.P/'protected_manifest.json').read_text())
method_pins={}
for p in pins:
 if p in manifest:
  assert pins[p]==manifest[p],p
  method_pins[p]=pins[p]
rh=a.E/'receiver_finalization_cpk19_20261021/Receiver.h'
assert a.sha(rh)==manifest[str(rh)]
method_pins[str(rh)]=a.sha(rh)
checks={'method_pins_match_original_capture':method_pins,'modes':{}}
TP=np.array([.308837802,-.038312997,-.234944116])
fig,axs=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
axis=None
for col,mode in enumerate(('off','on')):
 t=z[mode+'_time_s'];op=z[mode+'_online'];fp=z[mode+'_finalized'];gp=z[mode+'_reference']
 k=z[mode+'_applied_kimera_front'];g=z[mode+'_applied_glim_velodyne'];sel=(t>380)&(t<=390)
 both=sel&(k>0);go=sel&(k==0);err=op-fp
 assert both.sum()==50 and go.sum()==50 and np.all(g[sel]>0)
 phase_delta=err[both].mean(axis=0)-err[go].mean(axis=0)
 if axis is None: axis=phase_delta/np.linalg.norm(phase_delta)
 centered=err[sel]-err[sel].mean(axis=0)
 phase_residual=err[sel]-np.where((k[sel]>0)[:,None],err[both].mean(axis=0),err[go].mean(axis=0))
 phase_share=1-np.sum(phase_residual**2)/np.sum(centered**2)
 entry=dict(phase_mean_separation_m=float(np.linalg.norm(phase_delta)),
     glm_only_mean_offset_m=err[go].mean(axis=0).tolist(),both_mean_offset_m=err[both].mean(axis=0).tolist(),
     exact_phase_alternations=int(np.count_nonzero(np.diff((k[sel]>0).astype(int)))),
     centered_same_state_offset_variance_associated_with_phase_percent=float(100*phase_share),
     caution='Descriptive phase association, not fraction of error caused by Kimera; both factors and retirement coexist.',
     source_frontier_age_s={},example=[])
 epoch=1732549034915071665
 stamps=np.rint(t*1e9).astype(np.int64)+epoch
 for src in ('glim_velodyne','kimera_front'):
  age=(stamps-z[f'{mode}_latest_{src}'])*1e-9
  entry['source_frontier_age_s'][src]={'GLIM_only_cycle':a.stats(age[go]),'both_cycle':a.stats(age[both])}
 for i in np.flatnonzero((t>=383)&(t<383.65)):
  entry['example'].append(dict(time_s=float(t[i]),online_step_cm=float(a.norm(op[i]-op[i-1])*100),
    finalized_step_cm=float(a.norm(fp[i]-fp[i-1])*100),reference_step_cm=float(a.norm(gp[i]-gp[i-1])*100),
    online_displacement_cm=((op[i]-op[i-1])*100).tolist(),
    correction_later_cm=((fp[i]-op[i])*100).tolist(),glim_applied=int(g[i]),kimera_applied=int(k[i])))
 # Independent raw TUM + rotation-matrix lever arm, not CSV parser or prism helper.
 raw_paths={};fit_rmse={}
 for label,file in [('online','online_trajectory.tum'),('finalized','finalized_trajectory.tum')]:
  p=a.P/f'full_{mode}_receiver/receiver'/file;pins[str(p)]=a.sha(p)
  rows=np.loadtxt(p)
  R=Rotation.from_quat(rows[:,4:8]).as_matrix()
  prism=rows[:,1:4]+(R@TP[:,None]).squeeze(-1)
  raw_paths[label]=float(np.linalg.norm(np.diff(prism,axis=0),axis=1).sum())
  assert abs(raw_paths[label]-r['modes'][mode]['whole'][label]['path_m'])<1e-10
  rot,tr,scale=umeyama_alignment(prism.T,gp.T,with_scale=False);assert scale==1.
  fit_rmse[label]=float(np.sqrt(np.mean(np.sum((prism@rot.T+tr-gp)**2,axis=1))))
  assert np.max(np.abs(rows[:,1:4]))<100
 entry['independent_TUM_path_m']=raw_paths
 entry['full_saved_ATE_separate_SE3_fits_m']=fit_rmse
 checks['modes'][mode]=entry
 ss=(t>=383)&(t<384.05)
 axs[0,col].plot(t[ss],(err@axis)[ss]*100,color='#d33048',lw=1,label='ONLINE minus later-finalized point')
 axs[0,col].scatter(t[ss&(k==0)],(err@axis)[ss&(k==0)]*100,color='#aa7400',s=36,label='GLIM updates only')
 axs[0,col].scatter(t[ss&(k>0)],(err@axis)[ss&(k>0)]*100,color='#008e9b',s=36,label='GLIM + Kimera updates')
 axs[0,col].axhline(0,color='#00876c',lw=1)
 axs[0,col].set(title=f'CBS {mode.upper()}: alternating publication phases',ylabel='position offset along diagnostic axis (cm)')
 for p,color,label in [(op,'#d33048','ONLINE'),(fp,'#00876c','DELAYED finalized'),(gp,'#377bd1','IE-TC reference')]:
  axs[1,col].plot(t[1:][ss[1:]],a.norm(np.diff(p,axis=0))[ss[1:]]*100,color=color,label=label,marker='.',lw=1)
 axs[1,col].set(ylabel='distance in each 100 ms step (cm)',xlabel='seconds from capture start')
 for ax in axs[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(383,384)
fig.suptitle('The first-published point alternates with the update schedule\nDiagnostic axis fixed from OFF phase difference; no trajectory smoothing. Association does not establish cause.',fontsize=12)
fig.savefig(H/'publication_cycle.png',dpi=160);plt.close(fig)
checks['all_original_input_hashes_still_match']=all(a.sha(p)==h for p,h in r['input_sha256'].items())
checks['new_input_hashes_match']=all(a.sha(p)==h for p,h in pins.items())
assert checks['all_original_input_hashes_still_match'] and checks['new_input_hashes_match']
checks['input_sha256']=pins
(H/'phase_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
print(json.dumps({k:v for k,v in checks.items() if k!='input_sha256'},indent=2))
