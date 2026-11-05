"""Saved-artifact cross-checks; no optimizer or scoring rerun."""
import json,re,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from evo.tools import file_interface
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
P=W/H.name/'replay';r=json.loads((H/'results.json').read_text());assert r['passed']
marker='KIMERA_BACKEND_POSE_PATH_ROW,'
def poses(p):
    return np.array([list(map(float,re.sub(r'\x1b\[[0-9;]*m','',l.split(marker,1)[1]).split(',')))
        for l in p.read_text().splitlines() if marker in l])
old=poses(W/'eig2_retirement_capture_cpv66_20261031/prepared/full_on_source/estimators.log')
remove=poses(P/'remove_capture/estimators.log');retain=poses(P/'retain_capture/estimators.log')
assert old.shape==remove.shape==retain.shape==(298,24)
assert np.array_equal(remove[:30],retain[:30])
# Descriptive original-control reproduction, separate from A/B exact gates.
diff=np.max(abs(old-remove),axis=0)
assert np.array_equal(old[:,2:5],remove[:,2:5])
archives=0;improvement={}
for section,folder in [('common','common'),('full_saved','full')]:
    for mode,v in r[section].items():
        f=H/'evaluation'/(folder if section=='common' else folder+'_'+mode)
        a=file_interface.load_res_file(f/(mode+'_ape.zip'))
        assert a.stats['rmse']==v['ATE_RMSE_m'];archives+=1
        for key,rr in v['RPE'].items():
            a=file_interface.load_res_file(Path(rr['archive']))
            assert a.stats==rr['statistics'] and len(a.np_arrays['error_array'])==rr['pairs'];archives+=1
    a,b=r[section]['remove'],r[section]['retain']
    improvement[section]=dict(ATE_percent=100*(1-b['ATE_RMSE_m']/a['ATE_RMSE_m']),
        translation_RPE_percent=100*(1-b['RPE']['translation_m']['statistics']['rmse']/a['RPE']['translation_m']['statistics']['rmse']),
        rotation_RPE_percent=100*(1-b['RPE']['rotation_deg']['statistics']['rmse']/a['RPE']['rotation_deg']['statistics']['rmse']),
        path_excess_percent={mode:100*(v['path_length_m']/v['reference_path_length_m']-1) for mode,v in r[section].items()})
assert r['diagnostics']['retain']['absorption_boundaries']==268
out=dict(passed=True,EVO_archives_verified=archives,improvement=improvement,
    exact_A_B_first30_rows=True,original_CP66_remove_logged_positions_exact=True,
    original_CP66_remove_all_logged_fields_exact=False,original_control_max_field_difference=diff.tolist(),
    note='Original control quaternions differ by <=1.00000000003e-6 in six-significant-digit logs; no byte-exact full-output claim.')
with (H/'report_verification.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
