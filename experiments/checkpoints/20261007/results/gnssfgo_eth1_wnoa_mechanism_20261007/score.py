"""Unchanged evaluators and existing common grid; score only newly matched final graphs."""
from pathlib import Path
import json,sys,subprocess,csv
import numpy as np
from tf.transformations import quaternion_from_matrix
root=Path(__file__).resolve().parent
project=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
run=project/'runs/gnssfgo_eth1_live_wnoa_20261007'
tools=project/'src/cbsms/tools';sys.path.insert(0,str(tools))
from score_grandtour_relative_pose import load_tum,sort_unique,interpolate_pose
out=root/'evaluation';out.mkdir(exist_ok=False)
previous=run/'retained120_online_off_on_evaluation'
results=json.loads((previous/'comparison.json').read_text())['results']
grid=np.loadtxt(previous/'on_fused_online.tum')[:,0]
commands=json.loads((previous/'commands.json').read_text());templates=[c for c in commands if 'on_fused_online.tum' in c[c.index('--estimate')+1]]
record=[]
def prism_mask(p):return [r['timestamp'] for r in csv.DictReader(p.open()) if r['matched_reference']=='1']
reference_mask=prism_mask(previous/'on_fused_online_prism_aligned.csv')
for mode in ['off','on']:
    assert json.loads((root/f'validation_{mode}.json').read_text())['passed']
    curve=sort_unique(*load_tum(root/(mode+'_wnoa/trajectory.tum')));data=[]
    for t in grid:
        pose=interpolate_pose(*curve,t,.5);assert pose is not None
        p,r=pose;mat=np.eye(4);mat[:3,:3]=r;data.append([t,*p,*quaternion_from_matrix(mat)])
    label=mode+'_fused_final';np.savetxt(out/(label+'.tum'),data,fmt='%.9f')
    for template in templates:
        cmd=[s.replace('/workspace/cbs_gtsam4.3',str(project)) for s in template]
        kind='prism' if '--aligned-csv' in cmd else 'rpe'
        cmd[cmd.index('--estimate')+1]=str(out/(label+'.tum'));cmd[cmd.index('--output')+1]=str(out/(label+'_'+kind+'.json'))
        if kind=='prism':cmd[cmd.index('--aligned-csv')+1]=str(out/(label+'_aligned.csv'))
        else:cmd+=['--pairs-csv',str(out/(label+'_pairs.csv'))]
        with (out/(label+'_'+kind+'.log')).open('x') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
        record.append(cmd)
    a=json.loads((out/(label+'_prism.json')).read_text());r=json.loads((out/(label+'_rpe.json')).read_text())
    assert prism_mask(out/(label+'_aligned.csv'))==reference_mask
    assert a['matched_poses']==626 and r['valid_pairs']==1151
    arr=np.array(data)
    results[label]={'ate_m':a['ape_rmse_m'],'translation_rpe_m':r['translation_error_m']['rmse'],'rotation_rpe_deg':r['rotation_error_deg']['rmse'],
        'common_grid_body_path_m':float(np.linalg.norm(np.diff(arr[:,1:4],axis=0),axis=1).sum()),'evaluation_prism_path_m':a['estimate_full_path_m'],
        'matched_prism_poses':a['matched_poses'],'gt_coverage_s':a['matched_elapsed_coverage_s'],'rpe_pairs':r['valid_pairs']}
    print(label,results[label],flush=True)
(out/'commands.json').write_text(json.dumps(record,indent=2));(root/'metrics.json').write_text(json.dumps(results,indent=2))
