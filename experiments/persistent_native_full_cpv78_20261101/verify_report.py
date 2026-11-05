"""Independent archive and Method1 output cross-check; no scoring rerun."""
import json,re
from pathlib import Path
import numpy as np
from evo.tools import file_interface
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'replay'
r=json.loads((H/'results.json').read_text());assert r['passed']
archives=0;improvement={}
for section,folder in [('common','common'),('full_saved','full')]:
    for mode,v in r[section].items():
        f=H/'evaluation'/(folder if section=='common' else folder+'_'+mode)
        a=file_interface.load_res_file(f/(mode+'_ape.zip'))
        assert a.stats['rmse']==v['ATE_RMSE_m'];archives+=1
        for key,rr in v['RPE'].items():
            a=file_interface.load_res_file(Path(rr['archive']))
            assert a.stats==rr['statistics'] and len(a.np_arrays['error_array'])==rr['pairs'];archives+=1
    a,b=r[section]['temporary'],r[section]['persistent']
    improvement[section]=dict(ATE_percent=100*(1-b['ATE_RMSE_m']/a['ATE_RMSE_m']),
        translation_RPE_percent=100*(1-b['RPE']['translation_m']['statistics']['rmse']/a['RPE']['translation_m']['statistics']['rmse']),
        rotation_RPE_percent=100*(1-b['RPE']['rotation_deg']['statistics']['rmse']/a['RPE']['rotation_deg']['statistics']['rmse']),
        path_excess_percent={mode:100*(v['path_length_m']/v['reference_path_length_m']-1) for mode,v in r[section].items()})
def rows(mode):
    marker='KIMERA_BACKEND_POSE_PATH_ROW,'
    return np.array([list(map(float,re.sub(r'\x1b\[[0-9;]*m','',l.split(marker,1)[1]).split(',')))
        for l in (P/(mode+'_capture/estimators.log')).read_text().splitlines() if marker in l])
x,y=rows('temporary'),rows('persistent');n=min(len(x),len(y));assert x.shape[1]==y.shape[1]==24
out=dict(passed=True,EVO_archives_verified=archives,improvement=improvement,
    backend_pose_rows=dict(temporary=len(x),persistent=len(y)),
    exact_initial_rows_until_first_difference=next((i for i in range(n) if not np.array_equal(x[i],y[i])),n),
    native_policy_comparison_not_frozen_admission=True)
with (H/'report_verification.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
