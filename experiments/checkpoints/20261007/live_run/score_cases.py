#!/usr/bin/env python3
"""Use the existing native-trajectory evaluators, never source-only FGO proxies."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('phase',choices=['smoke','full'])
args=parser.parse_args()
out=root/(args.phase+'_evaluation')
out.mkdir(exist_ok=False)
tools=Path('/workspace/cbs_gtsam4.3/src/cbsms/tools')
dataset=Path('/workspace/cbs_gtsam4.3/src/datasets/GrandTour')
mission='2024-10-01-11-29-55'
paths={}
for mode in ['off','on']:
    validation=json.loads((root/(args.phase+'_'+mode+'_validation.json')).read_text())
    assert validation['passed']
    tag=validation['tag']
    source=root/(tag+'_source')
    receiver=root/(tag+'_receiver')/'receiver'
    for name,original in [('native_glim',source/'odom_imu.txt'),
                          ('fused_online',receiver/'online_trajectory.tum'),
                          ('fused_final',receiver/'trajectory.tum')]:
        destination=out/(mode+'_'+name+'.tum')
        shutil.copyfile(original,destination)
        paths[mode+'_'+name]=destination
    destination=out/(mode+'_native_kimera.tum')
    subprocess.run(['python3',str(tools/'export_odometry_csv_tum.py'),'--input',str(source/'kimera_odometry.csv'),
      '--output',str(destination),'--source-to-target','0','0','0','0','0','0','1'],check=True)
    paths[mode+'_native_kimera']=destination
trajectories={label:np.loadtxt(path) for label,path in paths.items()}
start=max(float(a[0,0]) for a in trajectories.values())
end=min(float(a[-1,0]) for a in trajectories.values())
(out/'common_interval.json').write_text(json.dumps(dict(start=start,end=end,duration=end-start,
 note='Same physical interval, existing evaluator association and gap rules; prism coverage is partial, not full-window ground truth.'),indent=2))
commands=[]
for label,path in paths.items():
    prism=['python3',str(tools/'score_grandtour_prism.py'),'--estimate',str(path),'--estimate-format','tum',
      '--source-frame','cpt7_imu','--reference-bag',str(dataset/(mission+'_ap20_prism_position.bag')),
      '--start-time-sec',str(start),'--end-time-sec',str(end),'--output',str(out/(label+'_prism.json')),
      '--aligned-csv',str(out/(label+'_prism_aligned.csv'))]
    rpe=['python3',str(tools/'score_grandtour_relative_pose.py'),'--estimate',str(path),'--estimate-format','tum',
      '--reference-bag',str(dataset/(mission+'_cpt7_ie_tc.bag')),'--start-time-sec',str(start),
      '--end-time-sec',str(end),'--delta-sec','1.0','--output',str(out/(label+'_rpe.json'))]
    for kind,command in [('prism',prism),('rpe',rpe)]:
        with (out/(label+'_'+kind+'.log')).open('x') as f:
            subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,check=True)
        commands.append(command)
(out/'commands.json').write_text(json.dumps(commands,indent=2))
print('Scoring completed on common physical interval',start,end)
