"""Full saved-journal publication comparison; no estimator or data changes."""
import hashlib, importlib.util, json, sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H=Path(__file__).resolve().parent; E=H.parent
R=Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027')
spec=importlib.util.spec_from_file_location('score14',E/'leica2_motion_prior_cpv14_20261026/score.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
c,a=s.c,s.a
CP10=E/'first_published_comparison_cpv10_20261026/outputs'
CP11=E/'leica2_velodyne_rerun_cpv11_20261026'
EPOCH=1732549034915071665
N=lambda x:np.linalg.norm(x,axis=-1)
pins={}; arrays={};full={};common_scores={};checks={}
def pin(p):
    p=Path(p)
    with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
    return p
def save(p,obj):
    with p.open('x') as f:json.dump(obj,f,indent=2,allow_nan=False)

assert json.loads(pin(H/'reconstruction_checks.json').read_text())['cases']['on']['rows']==4282
old=json.loads(pin(CP10/'velodyne_leica2.json').read_text())
common=np.load(pin(CP10/'velodyne_leica2.npz'));grid=common['timestamp_ns'];valid=common['common_valid']
display=json.loads(pin(CP11/'data_checks.json').read_text())
pub=np.load(pin(H/'publication_products.npz'))
ref=a.g.Reference(pin(a.GT))
reference=np.asarray(ref.times,np.int64),np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]
for mode in ('off','on'):
    align=display['products'][mode+'_fusion_online']
    Q=np.asarray(align['alignment_rotation']);tr=np.asarray(align['alignment_translation_m'])
    anchor=np.asarray(display['origin_reference_enu_m'])
    for product,file in [('absolute','online_trajectory.tum'),('chained','chained_trajectory.tum'),('delayed_finalized','finalized_trajectory.tum')]:
        key=mode+'_'+product
        curve=a.load_curve(pin(R/mode/file));t,p,r=curve
        assert len(t)==4282 and np.all(np.diff(t)==100_000_000)
        assert np.array_equal(t,pub[mode+'_timestamp_ns'])
        gp,gr,ok=a.sample(reference,t,50_000_000);assert ok.all()
        ep,er=a.prism(p,r);gp,gr=a.prism(gp,gr)
        value,aligned,disp,speed,angular=s.metrics(ep,er,gp,gr,np.ones(len(t),bool))
        full[key]=value
        value['coverage']=c.coverage(t,np.ones(len(t),bool),EPOCH,428.75)
        value['coverage']['reference_supported_saved_rows']=int(ok.sum())
        value['coverage']['reference_supported_saved_percent']=100.
        fixed=ep@Q.T+tr
        fixed_error=N(fixed-gp)
        disp_error=(fixed-fixed[0])-(gp-gp[0])
        fixed_step=N(np.diff(fixed,axis=0)-np.diff(gp,axis=0))
        value['original_absolute_fit']={'position_error_m':s.stat(fixed_error),
            'endpoint_position_error_m':float(fixed_error[-1]),
            'endpoint_displacement_error_m':float(N(disp_error[-1])),
            'displacement100ms_error_m':s.stat(fixed_step),
            'alignment_rotation':Q.tolist(),'alignment_translation_m':tr.tolist()}
        value['independent_fit_endpoint_error_m']=float(N(aligned[-1]-gp[-1]))
        if product=='absolute':
            delta=abs(value['ATE_RMSE_m']-old['full_saved'][mode+'_fusion_online']['ATE_RMSE_m'])
            assert delta<1e-7;checks[mode+'_original_full_ATE_difference_m']=delta
        p1,r1,ok=a.sample(curve,grid,500_000_000);assert ok[valid].all()
        ep1,er1=a.prism(p1,r1)
        common_scores[key],_=c.metrics(ep1,er1,common['reference_position'],common['reference_rotation'],valid,grid)
        if product=='absolute':
            delta=abs(common_scores[key]['ATE_RMSE_m']-old['common'][mode+'_fusion_online']['ATE_RMSE_m'])
            assert delta<1e-7;checks[mode+'_original_common_ATE_difference_m']=delta
        arrays[key+'_display_p']=fixed-anchor
        arrays[key+'_position_error_m']=fixed_error
        arrays[key+'_displacement_error_m']=N(disp_error)
        arrays[key+'_step_error_m']=fixed_step
        arrays[key+'_independent_fit_p']=aligned-anchor
    arrays[mode+'_reference_p']=gp-anchor
    arrays[mode+'_time_s']=(t-EPOCH)*1e-9
    arrays[mode+'_timestamp_ns']=t
    D=pub[mode+'_graph_T_odom'];O=pub[mode+'_chained_body_T'];G=pub[mode+'_graph_body_T']
    arrays[mode+'_graph_chained_position_difference_m']=N(G[:,:3,3]-O[:,:3,3])
    arrays[mode+'_graph_chained_rotation_difference_deg']=np.rad2deg(Rotation.from_matrix(np.swapaxes(O[:,:3,:3],1,2)@G[:,:3,:3]).magnitude())
    checks[mode+'_graph_to_odom_rotation_orthogonality_max']=float(np.max(np.abs(np.swapaxes(D[:,:3,:3],1,2)@D[:,:3,:3]-np.eye(3))))
    assert checks[mode+'_graph_to_odom_rotation_orthogonality_max']<1e-9

changes={}
for mode in ('off','on'):
    b=full[mode+'_absolute'];n=full[mode+'_chained']
    changes[mode]={'path_change_percent':100*(n['path_m']/b['path_m']-1),
        'ATE_improvement_percent':100*(1-n['ATE_RMSE_m']/b['ATE_RMSE_m']),
        'RPE1_translation_improvement_percent':100*(1-n['horizons']['1.0']['translation_RMSE_m']/b['horizons']['1.0']['translation_RMSE_m']),
        'RPE1_rotation_improvement_percent':100*(1-n['horizons']['1.0']['rotation_RMSE_deg']/b['horizons']['1.0']['rotation_RMSE_deg']),
        'fixed_fit_100ms_displacement_improvement_percent':100*(1-n['original_absolute_fit']['displacement100ms_error_m']['rms']/b['original_absolute_fit']['displacement100ms_error_m']['rms'])}
result={'task':'CP-V23','full':full,'common':common_scores,'common_coverage':old['common_coverage'],
    'unchanged_native_full':{m+'_'+role:old['full_saved'][m+'_'+role] for m in ('off','on') for role in ('glim_initial','kimera')},
    'unchanged_native_common':{m+'_'+role:old['common'][m+'_'+role] for m in ('off','on') for role in ('glim_initial','kimera')},
    'absolute_to_chained_changes':changes,'baseline_checks':checks,'synthetic_checks':a.tests(),
    'display_alignment':'Original CP-V10 absolute fusion rigid no-scale fit, identically applied to both fusion products per mode.',
    'ATE_alignment':'Separate full-interval rigid no-scale fit for each product; common-mask scores use their own common-mask fit.',
    'limitations':['Offline receiver replay, not live co-running.','Chained output has no added state age but omits historical graph corrections.',
        'No new covariance or velocity claimed for chained odometry.','DELAYED finalized is not online.','IE-TC shares CPT7 and is not independent prism GT.'],
    'not_promoted':True}
for module in (s,c,c.b,a,a.g):pin(Path(module.__file__))
for p in (Path(__file__),H/'PROTOCOL.txt'):pin(p)
pins.update({str(k):v for k,v in a.HASHES.items()})
for p,digest in pins.items():
    with open(p,'rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==digest
save(H/'results.json',result);save(H/'scoring_pins.json',pins)
assert not (H/'comparison_arrays.npz').exists()
np.savez_compressed(H/'comparison_arrays.npz',**arrays)
fig,axes=plt.subplots(3,2,figsize=(14,11),constrained_layout=True)
for col,mode in enumerate(('off','on')):
    t=arrays[mode+'_time_s'];gp=arrays[mode+'_reference_p']
    for product,label,color in [('absolute','Absolute graph ONLINE','#e83d58'),('chained','Chained incremental','#8c50c8')]:
        key=mode+'_'+product;p=arrays[key+'_display_p'];step=arrays[key+'_step_error_m']
        axes[0,col].plot(t,arrays[key+'_position_error_m'],label=label,color=color,lw=1)
        axes[1,col].plot(t,np.r_[0,np.cumsum(N(np.diff(p,axis=0)))],label=label,color=color,lw=1.4)
        # Non-overlapping one-second RMS bins, diagnostic only; trajectories untouched.
        n=len(step)//10
        axes[2,col].plot(t[1:1+n*10].reshape(-1,10).mean(1),100*np.sqrt((step[:n*10]**2).reshape(-1,10).mean(1)),label=label,color=color,lw=1)
    axes[1,col].plot(t,np.r_[0,np.cumsum(N(np.diff(gp,axis=0)))],label='IE-TC reference',color='#267abe',lw=1.4)
    axes[0,col].set(title='CBS '+mode.upper(),ylabel='Position error (m), SAME original fit')
    axes[1,col].set(ylabel='Cumulative estimated / reference path (m)')
    axes[2,col].set(ylabel='100ms displacement-vector error\n1s RMS bins (cm)',xlabel='Physical time after capture start (s)')
    for ax in axes[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('LEICA-2 Velodyne | Original graph ONLINE vs causal chained increments\nOriginal Qc; identical graph results; saved-journal replay, no smoothing of trajectories',fontsize=13)
fig.savefig(H/'comparison.png',dpi=160);plt.close(fig)
print(json.dumps({'changes':changes,'full':{k:{f:v[f] for f in ('ATE_RMSE_m','path_m','reference_path_m','horizons','original_absolute_fit')} for k,v in full.items()}},indent=2))
