#!/usr/bin/env python3
"""Use the unchanged Noetic evaluators and exact prior common evaluation mask."""
from pathlib import Path
import csv
import json
import subprocess
import sys
import numpy as np
from tf.transformations import quaternion_from_matrix

ROOT = Path('/audit')
BASE = Path('/baseline')
PROJECT = Path('/workspace/cbs_gtsam4.3')
sys.path.insert(0,str(PROJECT/'src/cbsms/tools'))
from score_grandtour_relative_pose import load_tum, sort_unique, interpolate_pose

def save(path, data):
    with path.open('x') as f:
        json.dump(data,f,indent=2,allow_nan=False)

def selected_mask(path):
    with path.open() as f:
        return [r['timestamp'] for r in csv.DictReader(f) if r['matched_reference']=='1']

def pair_mask(path):
    with path.open() as f:
        return [(r['start_time_sec'],r['end_time_sec']) for r in csv.DictReader(f)]

out=ROOT/'evaluation'
out.mkdir(exist_ok=False)
previous=PROJECT/'runs/gnssfgo_eth1_live_wnoa_20261007/retained120_online_off_on_evaluation'
grid=np.loadtxt(previous/'on_fused_online.tum')[:,0]
assert len(grid)==1161
reference_prism=selected_mask(previous/'on_fused_online_prism_aligned.csv')
reference_pairs=pair_mask(BASE/'evaluation/off_fused_final_pairs.csv')
templates=json.loads((BASE/'evaluation/commands.json').read_text())[:2]
record=[]
results=json.loads((BASE/'metrics.json').read_text())
results={name+'_x1' if '_fused_' in name else name:value for name,value in results.items()}

for mode in ('off','on'):
    assert json.loads((ROOT/f'{mode}_validation.json').read_text())['passed']
    for product,filename in [('online','online_trajectory.tum'),('final','trajectory.tum')]:
        curve=sort_unique(*load_tum(ROOT/(mode+'_x2')/filename))
        data=[]
        for t in grid:
            pose=interpolate_pose(*curve,t,.5)
            assert pose is not None
            p,r=pose
            mat=np.eye(4);mat[:3,:3]=r
            data.append([t,*p,*quaternion_from_matrix(mat)])
        label=mode+'_fused_'+product+'_x2'
        arr=np.array(data)
        np.savetxt(out/(label+'.tum'),arr,fmt='%.9f')
        for template in templates:
            cmd=[x.replace('/home/yeranis/repos/V4RL/cbs_gtsam4.3',str(PROJECT)) for x in template]
            kind='prism' if '--aligned-csv' in cmd else 'rpe'
            cmd[cmd.index('--estimate')+1]=str(out/(label+'.tum'))
            cmd[cmd.index('--output')+1]=str(out/(label+'_'+kind+'.json'))
            if kind=='prism':cmd[cmd.index('--aligned-csv')+1]=str(out/(label+'_aligned.csv'))
            else:cmd[cmd.index('--pairs-csv')+1]=str(out/(label+'_pairs.csv'))
            with (out/(label+'_'+kind+'.log')).open('x') as log:
                subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
            record.append(cmd)
        a=json.loads((out/(label+'_prism.json')).read_text())
        r=json.loads((out/(label+'_rpe.json')).read_text())
        assert selected_mask(out/(label+'_aligned.csv'))==reference_prism
        assert pair_mask(out/(label+'_pairs.csv'))==reference_pairs
        assert a['matched_poses']==626 and r['valid_pairs']==1151
        raw=np.loadtxt(ROOT/(mode+'_x2')/filename)
        results[label]=dict(ate_m=a['ape_rmse_m'],translation_rpe_m=r['translation_error_m']['rmse'],
            rotation_rpe_deg=r['rotation_error_deg']['rmse'],
            common_grid_body_path_m=float(np.linalg.norm(np.diff(arr[:,1:4],axis=0),axis=1).sum()),
            raw_full_body_path_m=float(np.linalg.norm(np.diff(raw[:,1:4],axis=0),axis=1).sum()),
            evaluation_prism_path_m=a['estimate_full_path_m'],gt_overlap_estimate_path_m=a['matched_estimate_path_m'],
            gt_overlap_reference_path_m=a['matched_reference_path_m'],matched_prism_poses=a['matched_poses'],
            gt_coverage_s=a['matched_elapsed_coverage_s'],rpe_pairs=r['valid_pairs'],
            common_grid_count=len(grid),common_grid_span_s=float(grid[-1]-grid[0]),exact_masks_verified=True)
        print(label,json.dumps(results[label]),flush=True)
save(out/'commands.json',record)
save(ROOT/'metrics.json',results)
