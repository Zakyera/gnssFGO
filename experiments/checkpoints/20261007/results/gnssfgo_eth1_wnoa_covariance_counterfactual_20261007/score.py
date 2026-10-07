"""Score the single stationary retrospective diagnostic on the frozen common mask."""
from pathlib import Path
import json,sys,subprocess,csv
import numpy as np
from tf.transformations import quaternion_from_matrix
root=Path(__file__).resolve().parent
base=Path('/base')
project=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
run=project/'runs/gnssfgo_eth1_live_wnoa_20261007'
sys.path.insert(0,str(project/'src/cbsms/tools'))
from score_grandtour_relative_pose import load_tum,sort_unique,interpolate_pose
assert json.loads((root/'complete.json').read_text())['passed']
assert json.loads((root/'on_means_off_cov/optimizer.json').read_text())['normalized_gradient']<=1e-5
out=root/'evaluation';out.mkdir(exist_ok=False)
previous=run/'retained120_online_off_on_evaluation'
grid=np.loadtxt(previous/'on_fused_online.tum')[:,0]
commands=json.loads((previous/'commands.json').read_text())
templates=[c for c in commands if 'on_fused_online.tum' in c[c.index('--estimate')+1]]
assert len(templates)==2
def mask(p):return [r['timestamp'] for r in csv.DictReader(p.open()) if r['matched_reference']=='1']
def pairs(p):return [(r['start_time_sec'],r['end_time_sec']) for r in csv.DictReader(p.open())]
curve=sort_unique(*load_tum(root/'on_means_off_cov/trajectory.tum'));data=[]
for t in grid:
    pose=interpolate_pose(*curve,t,.5);assert pose is not None
    p,r=pose;mat=np.eye(4);mat[:3,:3]=r;data.append([t,*p,*quaternion_from_matrix(mat)])
label='on_means_off_cov';np.savetxt(out/(label+'.tum'),data,fmt='%.9f');executed=[]
for template in templates:
    cmd=[s.replace('/workspace/cbs_gtsam4.3',str(project)) for s in template]
    kind='prism' if '--aligned-csv' in cmd else 'rpe'
    cmd[cmd.index('--estimate')+1]=str(out/(label+'.tum'));cmd[cmd.index('--output')+1]=str(out/(label+'_'+kind+'.json'))
    if kind=='prism':cmd[cmd.index('--aligned-csv')+1]=str(out/(label+'_aligned.csv'))
    else:cmd+=['--pairs-csv',str(out/(label+'_pairs.csv'))]
    with (out/(label+'_'+kind+'.log')).open('x') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
    executed.append(cmd)
assert mask(out/(label+'_aligned.csv'))==mask(base/'evaluation/on_fused_final_aligned.csv')
assert pairs(out/(label+'_pairs.csv'))==pairs(base/'evaluation/on_fused_final_pairs.csv')
a=json.loads((out/(label+'_prism.json')).read_text());r=json.loads((out/(label+'_rpe.json')).read_text())
assert a['matched_poses']==626 and r['valid_pairs']==1151
arr=np.array(data)
new={'ate_m':a['ape_rmse_m'],'translation_rpe_m':r['translation_error_m']['rmse'],'rotation_rpe_deg':r['rotation_error_deg']['rmse'],
    'common_grid_body_path_m':float(np.linalg.norm(np.diff(arr[:,1:4],axis=0),axis=1).sum()),
    'full_support_body_path_m':float(np.linalg.norm(np.diff(curve[1],axis=0),axis=1).sum()),
    'evaluation_prism_path_m':a['estimate_full_path_m'],'gt_overlap_estimate_prism_path_m':a['matched_estimate_path_m'],
    'gt_path_m':a['matched_reference_path_m'],'gt_coverage_s':a['matched_elapsed_coverage_s'],'matched_prism_poses':a['matched_poses'],'rpe_pairs':r['valid_pairs']}
results=json.loads((base/'metrics.json').read_text());results[label]=new
with (out/'commands.json').open('x') as f:json.dump(executed,f,indent=2)
with (root/'metrics.json').open('x') as f:json.dump(results,f,indent=2)
print(json.dumps(new,indent=2))
