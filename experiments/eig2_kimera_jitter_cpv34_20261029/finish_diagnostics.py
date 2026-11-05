"""Read-only export/config checks, evidence plot and frozen-input verification."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from rosbags.highlevel import AnyReader
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import analyze_windows as w
from analyze_inputs import exp, between, difference, vec

H=w.H;V=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=V/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared'
w.pin(__file__);w.pin(H/'analyze_inputs.py');w.pin(w.__file__)
for f in ['window_results.json','input_results.json']:
    receipt=w.load(H/f)
    for p,s in receipt['input_sha256'].items():w.pin(p,s)
manifest=w.load(P/'protected_manifest.json')
source_files=[
 V/'runs/kimera_incremental_prior_cpk9_20261019/VioBackend.cpp',
 V/'runs/kimera_incremental_prior_cpk9_20261019/NativePriorRefresh.h',
 V/'runs/kimera_incremental_prior_cpk9_20261019/OriginalChartPrior.h',
 V/'runs/kimera_incremental_prior_cpk9_20261019/private_build/libkimera_vio.so',
 V/'runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp',
 V/'src/glim/src/glim/odometry/odometry_estimation_imu.cpp',
 V/'src/glim_ros1/src/glim_rosnode.cpp',
 V/'src/Kimera-VIO-ROS/src/KimeraVioRos.cpp',
 V/'src/Kimera-VIO-ROS/src/RosOnlineDataProvider.cpp',
 V/'src/Kimera-VIO/include/kimera-vio/imu-frontend/CbsNativeImuMeanBridge.h',
]
code_checks=[]
for p in source_files:
    expected=manifest.get(str(p));actual=w.sha(p)
    code_checks.append(dict(path=str(p),sha256=actual,pinned_in_original=expected is not None,
                            matches_original=actual==expected if expected else None))
    w.pin(p)
configs={}
for mode in ['off','on']:
    root=P/('full_'+mode+'_source')
    effective=w.load(root/'effective_mode_check.json')
    assert 'imu:=/boxi/cpt7/imu' in effective['kimera_argv']
    for name in ['resolved_glim_params.yaml','resolved_kimera_params.yaml','run_metadata.txt',
                 'experiment_runner_used.sh','grandtour_launch_used.launch','newer_college_reference_launch.launch',
                 'glim_config/config_ros.json','glim_config/config_sensors.json','kimera_profile/ImuParams.yaml',
                 'kimera_profile/BackendParams.yaml','kimera_profile/LeftCameraParams.yaml','kimera_profile/RightCameraParams.yaml',
                 'velodyne_validation.json','local_provenance.json']:
        w.pin(root/name)
    configs[mode]=dict(kimera_argv=effective['kimera_argv'],kimera=effective['kimera'],
                       glim=effective['glim'],candidate=effective['kimera_candidate_env'])
preflight=w.load(w.E/'velodyne_full_campaign_cpv4_20261024/eig2/bag_preflight.json')
root=P/'full_on_source'
logfiles=list((root/'ros_logs').glob('*/glim_rosnode-1-stdout.log'));assert len(logfiles)==1
glog=w.pin(logfiles[0]);exports={};max_export_t=0.;max_export_r=0.;export_count=0
with glog.open() as f:
    for line in f:
        if 'GLIM_CBS_EXPORT_ENDPOINT_ROW,' not in line:continue
        v=w.ANSI.sub('',line.split('GLIM_CBS_EXPORT_ENDPOINT_ROW,',1)[1]).strip().split(',')
        assert len(v)==22,len(v)
        key=(int(v[3]),int(v[4]))
        pa=(np.array(v[7:10],float),R.from_quat(np.array(v[10:14],float)).as_matrix())
        pb=(np.array(v[14:17],float),R.from_quat(np.array(v[17:21],float)).as_matrix())
        raw=vec(v[21]);err=difference(between(pa,pb),exp(raw))
        max_export_t=max(max_export_t,err[0]);max_export_r=max(max_export_r,err[1])
        exports.setdefault(key,[]).append(raw);export_count+=1
matched=0;max_export_payload=0.
with AnyReader([w.pin(root/'recordings/cbs.bag')]) as reader:
    cs=[c for c in reader.connections if c.topic=='/kimera/cbs/odom_belief_in']
    for c,ts,data in reader.messages(connections=cs):
        msg=reader.deserialize(data,c.msgtype)
        for b in msg.beliefs:
            cand=exports[(b.from_pose_index,b.to_pose_index)]
            err=min(float(np.max(abs(x-b.relative_mu))) for x in cand)
            max_export_payload=max(max_export_payload,err);matched+=1
assert max_export_t<1e-10 and max_export_r<1e-8 and max_export_payload<1e-12

# Deliberately small scientific diagnostic figure, not an aligned accuracy plot.
ws,*_=w.parse(root/'estimators.log');k=1097
ids=sorted(set(ws[k-1])&set(ws[k]))
old=np.array([ws[k-1][i][0] for i in ids]);new=np.array([ws[k][i][0] for i in ids])
dr=np.array([ws[k][i][1]@ws[k-1][i][1].T for i in ids]);rot=R.from_matrix(dr).mean().as_matrix()
tr=np.mean(new-old@rot.T,0);new_in_old=(new-tr)@rot
events=w.load(H/'velodyne_eig2_on_events.json');time=np.array([e['time_s'] for e in events]);sel=(time>218)&(time<222)
plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
fig,axes=plt.subplots(1,3,figsize=(15,4.5),layout='constrained')
origin=old.mean(0)
for pts,label,col in [(old,'Before update','#2878bd'),(new,'After update','#e28318')]:
    axes[0].plot(pts[:,0]-origin[0],pts[:,1]-origin[1],'.-',label=label,color=col)
axes[0].set(title='Same 25 active poses\nAbsolute window position',xlabel='X from old centroid (m)',ylabel='Y from old centroid (m)')
axes[0].axis('equal');axes[0].legend()
for pts,label,col in [(old,'Before', '#2878bd'),(new_in_old,'After, rigidly overlaid','#e28318')]:
    axes[1].plot(pts[:,0]-origin[0],pts[:,1]-origin[1],'.-',label=label,color=col)
axes[1].set(title='Remove ONE rigid window transform\nMaximum 3D shape residual: 2.24 cm',xlabel='X from old centroid (m)',ylabel='Y from old centroid (m)')
axes[1].axis('equal');axes[1].legend()
for key,label,col in [('previous_revision_m','Revision of previous pose','#cb4b43'),('same_update_motion_m','New motion within one solution','#25804b')]:
    values=np.array([e[key] for e in events]);axes[2].plot(time[sel],values[sel],'.-',label=label,color=col)
axes[2].axvline(219.798141556,color='#666',ls=':',lw=1)
axes[2].set(title='Why Method 1 has a spike\n6.34 m revision vs 0.164 m motion',xlabel='Seconds from capture start',ylabel='Displacement (m)')
axes[2].legend()
fig.suptitle('EIG-2 Velodyne / Kimera CBS-ON — saved MAIN states, update 1097',fontsize=14)
fig.savefig(H/'window_mechanism.png',dpi=160);plt.close(fig)

evo={}
for case in ['velodyne_eig2','hesai_eig2']:
    d=w.load(w.H32/(case+'.json'))
    evo[case]=dict(common_coverage=d['common_coverage'],reference_path_length_m=d['reference_path_length_m'],
                   RPE_pairs=d['RPE_pairs'],results={k:v for k,v in d['results'].items() if k.endswith('_kimera')})
    for v in evo[case]['results'].values():
        w.pin(v['ATE_evo_archive'])
        for r in v['RPE'].values():w.pin(r['evo_archive'])
assert all(w.sha(p)==s for p,s in w.PINS.items())
w.save(H/'verification.json',dict(passed=True,input_sha256=w.PINS,inputs_unchanged=True,
 code_identity_checks=code_checks,configurations=configs,
 calibration=preflight['calibration'],quiet_start_checks=preflight['quiet_start_checks'],
 glim_export=dict(endpoint_rows=export_count,incoming_matched=matched,
                  maximum_endpoint_vs_payload_translation_m=max_export_t,
                  maximum_endpoint_vs_payload_rotation_deg=max_export_r,
                  maximum_bag_vs_export_mu_abs=max_export_payload),evo_metrics=evo,
 figure_sha256=w.sha(H/'window_mechanism.png'),
 limits=['Wrapper and mean-bridge source files without original pins are code context, not historical byte identity.',
         'Saved logs do not contain full nonlinear costs, prior Hessians, solver gradients, caches, IMU packets or bias states for each update.',
         'No new source/receiver replay, build, tuning or production fix performed.']))
print(json.dumps(dict(passed=True,pins=len(w.PINS),code_identity=code_checks,export_checks=dict(rows=export_count,matched=matched,t=max_export_t,r=max_export_r,mu=max_export_payload)),indent=2))
