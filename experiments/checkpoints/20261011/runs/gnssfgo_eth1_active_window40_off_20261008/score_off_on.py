"""Existing evaluators, eight native/fused outputs, one exact common mask.

No estimation, adaptation, or use of a longer retrospective graph.
Run in the existing Noetic container after both cases validate.
"""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from tf.transformations import quaternion_from_matrix

ROOT=Path(__file__).resolve().parent
ON=ROOT.parent/'gnssfgo_eth1_active_window40_prepared_20261008'
TOOLS=Path('/workspace/cbs_gtsam4.3/src/cbsms/tools')
DATA=Path('/workspace/cbs_gtsam4.3/src/datasets/GrandTour')
sys.path.insert(0,str(TOOLS))
from score_grandtour_relative_pose import load_tum,sort_unique,interpolate_pose

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def save(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False)

sources={}
protected={}
clocks={}
validation={}
for mode,base in [('off',ROOT),('on',ON)]:
    vpath=base/('pilot_'+mode+'_validation.json')
    v=json.loads(vpath.read_text())
    assert v['passed']
    s=v['summary']
    assert s['receiver_model']=='source_wnoa' and s['imu_factor_count']==0
    assert s['source_model_noise']=={} and s['experimental_qc_diagonal']==[.00375]*3+[.015]*3
    assert s['pending_at_end']==s['revision_stale']==s['revision_duplicates']==0
    assert v['max_publication_gradient']<=1e-5
    validation[mode]=dict(tag=v['tag'],validation_sha256=sha(vpath),max_gradient=v['max_publication_gradient'],
       versions=v['versions'],active_ids=v['active_source_ids'],replacements=v['replacements'],
       live_output_age_s=v['live_output_age_s'],summary=s)
    source=base/('pilot_'+mode+'_source')
    recv=base/('pilot_'+mode+'_receiver')
    sources[mode]=(source,recv)
    protected[str(vpath)]=sha(vpath)
    journal=recv/'events.csv'
    protected[str(journal)]=sha(journal)
    clocks[mode]=[r[2:] for r in csv.reader(journal.open()) if r[0]=='I']
assert clocks['off']==clocks['on'], 'Raw sensor-clock payloads or coverage differ'
assert validation['off']['summary']['initial_timestamp_ns']==validation['on']['summary']['initial_timestamp_ns']
for directory in ['glim_config','kimera_profile']:
    trees={}
    for mode,(source,_) in sources.items():
        trees[mode]={str(p.relative_to(source/directory)):sha(p) for p in (source/directory).rglob('*') if p.is_file()}
    assert trees['off']==trees['on']

out=ROOT/'comparison'
out.mkdir(exist_ok=False)
raw=out/'original_cropped'; raw.mkdir()
grid=out/'common_grid'; grid.mkdir()
paths={}
full_stats={}
cut=int(clocks['off'][-1][0])*1e-9
for mode,(source,recv) in sources.items():
    originals={'native_glim':source/'odom_imu.txt','fused_online':recv/'receiver/online_trajectory.tum',
               'fused_final':recv/'receiver/trajectory.tum'}
    k=source/'kimera_odometry.csv'
    protected[str(k)]=sha(k)
    dest=raw/(mode+'_native_kimera.tum')
    subprocess.run(['python3',str(TOOLS/'export_odometry_csv_tum.py'),'--input',str(k),'--output',str(dest),
                    '--source-to-target','0','0','0','0','0','0','1'],check=True)
    originals['native_kimera']=dest
    for kind,path in originals.items():
        a=np.loadtxt(path)
        assert np.isfinite(a).all() and np.all(np.diff(a[:,0])>0)
        assert a[-1,0]<=cut+1e-6
        label=mode+'_'+kind
        protected[str(path)]=sha(path)
        full_stats[label]=dict(rows=len(a),duration_s=float(a[-1,0]-a[0,0]),
             start_s=float(a[0,0]),end_s=float(a[-1,0]),max_gap_s=float(np.diff(a[:,0]).max()),
             path_m=float(np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1).sum()))
        paths[label]=path
curves={name:sort_unique(*load_tum(path)) for name,path in paths.items()}
start=max(float(c[0][0]) for c in curves.values())
end=min(float(c[0][-1]) for c in curves.values())
candidate=start+np.arange(int(np.floor((end-start)/.1))+1)*.1
common=[]
rows={name:[] for name in paths}
for stamp in candidate:
    poses={name:interpolate_pose(*curve,stamp,.5) for name,curve in curves.items()}
    if any(pose is None for pose in poses.values()):continue
    common.append(float(stamp))
    for name,(position,rotation) in poses.items():
        mat=np.eye(4);mat[:3,:3]=rotation
        rows[name].append([stamp,*position,*quaternion_from_matrix(mat)])
assert len(common)>10
mask=dict(candidate_times=len(candidate),common_times=len(common),start=common[0],end=common[-1],
          sample_period_s=.1,max_interpolation_gap_s=.5,
          timestamp_sha256=hashlib.sha256(np.array(common,dtype='<f8').tobytes()).hexdigest())
for name,data in rows.items():np.savetxt(str(grid/(name+'.tum')),data,fmt='%.9f')
commands=[]
results={}
for name in paths:
    path=grid/(name+'.tum')
    prism=['python3',str(TOOLS/'score_grandtour_prism.py'),'--estimate',str(path),'--estimate-format','tum',
       '--source-frame','cpt7_imu','--reference-bag',str(DATA/'2024-10-01-11-29-55_ap20_prism_position.bag'),
       '--start-time-sec',str(start),'--end-time-sec',str(end),'--output',str(grid/(name+'_prism.json')),
       '--aligned-csv',str(grid/(name+'_prism_aligned.csv'))]
    rpe=['python3',str(TOOLS/'score_grandtour_relative_pose.py'),'--estimate',str(path),'--estimate-format','tum',
       '--reference-bag',str(DATA/'2024-10-01-11-29-55_cpt7_ie_tc.bag'),'--start-time-sec',str(start),
       '--end-time-sec',str(end),'--delta-sec','1.0','--output',str(grid/(name+'_rpe.json'))]
    for kind,cmd in [('prism',prism),('rpe',rpe)]:
        with (grid/(name+'_'+kind+'.log')).open('x') as log:
            subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        commands.append(cmd)
    p=json.loads((grid/(name+'_prism.json')).read_text());r=json.loads((grid/(name+'_rpe.json')).read_text())
    results[name]=dict(ATE_m=p['ape_rmse_m'],tRPE_1s_m=r['translation_error_m']['rmse'],
       rRPE_1s_deg=r['rotation_error_deg']['rmse'],full_path_m=full_stats[name]['path_m'],
       evaluation_path_m=p['estimate_full_path_m'],GT_overlap_path_m=p['matched_estimate_path_m'],
       GT_reference_path_m=p['matched_reference_path_m'],GT_poses=p['matched_poses'],
       GT_coverage_s=p['matched_elapsed_coverage_s'],RPE_pairs=r['valid_pairs'])
assert len({r['GT_poses'] for r in results.values()})==1
assert len({r['GT_coverage_s'] for r in results.values()})==1
assert len({r['RPE_pairs'] for r in results.values()})==1
for p,h in protected.items():assert sha(Path(p))==h,p
save(out/'summary.json',dict(mask=mask,results=results,full_trajectories=full_stats,
    validation=validation,raw_clock_samples_equal=len(clocks['off']),cutoff_s=cut,
    caveat='Fresh matched 40 s OFF capture vs completed 40 s temporary-factor ON capture; no longer-run final graphs used. ATE partial AP20 prism; RPE IE-TC shares CPT7.'))
save(out/'commands.json',commands)
save(out/'input_hashes.json',protected)
print(json.dumps(dict(mask=mask,results=results),indent=2))
