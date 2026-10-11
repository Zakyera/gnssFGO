"""Quantify metric sensitivity to the retained repeat, not a new accuracy case."""
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from tf.transformations import quaternion_from_matrix

ROOT=Path(__file__).resolve().parent
CAPTURE=ROOT.parent/'source_huber_eth1_30s_opencv42_20261008'
TOOLS=Path('/workspace/cbs_gtsam4.3/src/cbsms/tools')
sys.path.insert(0,str(TOOLS))
from score_grandtour_relative_pose import load_tum, sort_unique, interpolate_pose

def main():
    original=ROOT/'matched30_common_evaluation'
    out=ROOT/'repeat_metric_sensitivity';out.mkdir(exist_ok=False)
    stamps=np.loadtxt(original/'on_huber_fused_online.tum')[:,0]
    commands=json.loads((ROOT/'matched30_evaluation/commands.json').read_text())
    results={}
    for method,filename in [('fused_online','online_trajectory.tum'),('fused_final','trajectory.tum')]:
        label='on_huber_'+method
        curve=sort_unique(*load_tum(CAPTURE/'on_huber_repeat'/filename))
        data=[]
        for stamp in stamps:
            pose=interpolate_pose(*curve,stamp,.5)
            assert pose is not None
            p,r=pose;matrix=np.eye(4);matrix[:3,:3]=r
            data.append([stamp,*p,*quaternion_from_matrix(matrix)])
        np.savetxt(out/(label+'.tum'),data,fmt='%.9f')
        selected=[c for c in commands if Path(c[c.index('--estimate')+1]).stem==label]
        assert len(selected)==2
        for old in selected:
            command=[v.replace(str(ROOT/'matched30_evaluation'),str(out)) for v in old]
            output=Path(command[command.index('--output')+1])
            with output.with_suffix('.log').open('x') as f:
                subprocess.run(command,check=True,stdout=f,stderr=subprocess.STDOUT)
        a=json.loads((original/(label+'_prism.json')).read_text())
        b=json.loads((out/(label+'_prism.json')).read_text())
        x=json.loads((original/(label+'_rpe.json')).read_text())
        y=json.loads((out/(label+'_rpe.json')).read_text())
        assert a['matched_poses']==b['matched_poses']==113 and x['valid_pairs']==y['valid_pairs']==251
        d=dict(ATE_m=abs(a['ape_rmse_m']-b['ape_rmse_m']),
               translation_RPE_m=abs(x['translation_error_m']['rmse']-y['translation_error_m']['rmse']),
               rotation_RPE_deg=abs(x['rotation_error_deg']['rmse']-y['rotation_error_deg']['rmse']))
        assert all(v<1e-7 for v in d.values()),d
        results[method]=d
    with (out/'metric_differences.json').open('x') as f:json.dump(results,f,indent=2)
    print(json.dumps(results,indent=2))

if __name__=='__main__':main()
