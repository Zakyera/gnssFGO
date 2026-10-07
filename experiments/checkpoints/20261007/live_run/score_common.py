#!/usr/bin/env python3
"""Common-grid evaluation only; retained trajectories and estimators untouched."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from tf.transformations import quaternion_from_matrix

root=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('phase',choices=['smoke','full'])
a=p.parse_args()
original=root/(a.phase+'_evaluation')
out=root/(a.phase+'_common_evaluation')
out.mkdir(exist_ok=False)
tools=Path('/workspace/cbs_gtsam4.3/src/cbsms/tools')
sys.path.insert(0,str(tools))
from score_grandtour_relative_pose import load_tum, sort_unique, interpolate_pose

paths={p.stem:p for p in sorted(original.glob('*.tum'))}
assert len(paths)==8
curves={name:sort_unique(*load_tum(path)) for name,path in paths.items()}
interval=json.loads((original/'common_interval.json').read_text())
start=interval['start']; end=interval['end']
count=int(np.floor((end-start)/.1))+1
data={name:[] for name in paths}; common=[]
for stamp in start+np.arange(count)*.1:
    poses={name:interpolate_pose(*curve,stamp,.5) for name,curve in curves.items()}
    if any(pose is None for pose in poses.values()):
        continue
    common.append(float(stamp))
    for name,(position,rotation) in poses.items():
        matrix=np.eye(4); matrix[:3,:3]=rotation
        data[name].append([stamp,*position,*quaternion_from_matrix(matrix)])
assert len(common)>10
for name,rows in data.items():
    np.savetxt(str(out/(name+'.tum')),rows,fmt='%.9f')
manifest=dict(candidate_times=count,common_times=len(common),start=common[0],end=common[-1],
              max_estimate_interpolation_gap_s=.5,sample_period_s=.1,
              timestamps_sha256=hashlib.sha256(np.array(common,dtype='<f8').tobytes()).hexdigest(),
              note='Offline scoring resampling with existing evaluator linear position / quaternion SLERP; not an estimator or causal-output change. Original full-path lengths retained separately.')
(out/'common_mask.json').write_text(json.dumps(manifest,indent=2))
commands=json.loads((original/'commands.json').read_text())
for old in commands:
    command=[arg.replace(str(original),str(out)) for arg in old]
    # Container-relative originals and path strings agree here.
    output=Path(command[command.index('--output')+1])
    with output.with_suffix('.log').open('x') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
metrics=[json.loads((out/(name+'_prism.json')).read_text()) for name in paths]
assert len({m['matched_poses'] for m in metrics})==1
assert len({m['matched_elapsed_coverage_s'] for m in metrics})==1
manifest['matched_prism_poses']=metrics[0]['matched_poses']
manifest['matched_prism_coverage_s']=metrics[0]['matched_elapsed_coverage_s']
(out/'validated_mask.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
