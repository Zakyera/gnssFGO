"""Independent CP-V27 log geometry, product identity and integrity closeout."""
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
import analyze as m
import numpy as np
from scipy.spatial.transform import Rotation as R

H=Path(__file__).resolve().parent
assert not (H/'verification.json').exists()
pins={}; cases={}; cross={}; max_rel=0.; max_yaw=0.; total=0
for row in m.c.b.ROWS:
    name=row[0]
    d=json.loads((H/(name+'.json')).read_text())
    saved=np.load(H/(name+'.npz'))
    for p,sha in d['input_sha256'].items():
        if p in pins: assert pins[p]==sha
        pins[p]=sha
    for mode,check in d['kimera_reconstruction_checks'].items():
        rows=[]
        with Path(check['source_log']).open() as f:
            for line in f:
                if 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:
                    # Parse all fields, including the separately logged graph-to-increment diagnostic.
                    rows.append([float(x) for x in m.ANSI.sub('',line.split('KIMERA_BACKEND_POSE_PATH_ROW,')[1]).strip().split(',')])
        v=np.asarray(rows)
        assert len(v)==check['rows']
        rg=R.from_quat(v[:,[6,7,8,5]]).as_matrix()
        ri=R.from_quat(v[:,[13,14,15,12]]).as_matrix()
        pg=v[:,2:5]; pi=v[:,9:12]
        assert np.array_equal(pg,saved[mode+'_kimera_absolute_body_position'])
        assert np.array_equal(rg,saved[mode+'_kimera_absolute_body_rotation'])
        rel=np.einsum('nji,nj->ni',rg,pi-pg)
        discrepancy=np.linalg.norm(rel-v[:,16:19],axis=1)
        pbound=np.linalg.norm(m.rounding_bound(v[:,2:5]),axis=1)+np.linalg.norm(m.rounding_bound(v[:,9:12]),axis=1)
        # Conservative source quaternion rounding bound already computed per case.
        bound=pbound+check['graph_rotation_rounding_bound_rad']*(np.linalg.norm(pi-pg,axis=1)+pbound)
        bound+=np.linalg.norm(m.rounding_bound(v[:,16:19]),axis=1)+1e-10
        assert np.all(discrepancy <= bound), (name,mode,float(np.max(discrepancy-bound)))
        norm_error=abs(np.linalg.norm(rel,axis=1)-v[:,19])
        assert np.all(norm_error <= bound+m.rounding_bound(v[:,19]))
        yawg=np.arctan2(rg[:,1,0],rg[:,0,0]); yawi=np.arctan2(ri[:,1,0],ri[:,0,0])
        wrap=lambda x:(x+np.pi)%(2*np.pi)-np.pi
        yawerr=np.max(np.abs(np.column_stack([wrap(yawg-v[:,20]),wrap(yawi-v[:,21]),wrap(yawi-yawg-v[:,22])])),axis=1)
        assert yawerr.max()<1e-4
        max_rel=max(max_rel,float(discrepancy.max()));max_yaw=max(max_yaw,float(yawerr.max()))
        total+=len(v)
    cases[name]=d
for seq in ('spx3','eig2','leica2'):
    h=np.load(H/('hesai_'+seq+'.npz')); v=np.load(H/('velodyne_'+seq+'.npz'))
    cross[seq]=all(np.array_equal(h['off_kimera_'+k],v['off_kimera_'+k]) for k in ('timestamp_ns','absolute_body_position','absolute_body_rotation'))
    assert cross[seq]
for p,sha in pins.items(): assert m.a.g.sha(Path(p))==sha,p
out=dict(passed=True,verified_inputs=len(pins),all_input_hashes_unchanged=True,
         kimera_absolute_rows=total,log_relative_pose_consistency_max_m=max_rel,
         log_yaw_consistency_max_rad=max_yaw,kimera_off_identical_between_sensors=cross,
         historical_products_reproduced=sum(len(d['unchanged_product_checks']) for d in cases.values()),
         historical_full_and_common_ATE_max_difference_m=max(v[k] for d in cases.values() for v in d['unchanged_product_checks'].values() for k in ('common_ATE_difference_m','full_ATE_difference_m')),
         kimera_pose_rounding_bound_max_m=max(x['graph_prism_position_rounding_bound_m'] for d in cases.values() for x in d['kimera_reconstruction_checks'].values()),
         rounded_incremental_control_ATE_max_difference_m=max(x[k] for d in cases.values() for x in d['kimera_reconstruction_checks'].values() for k in ('rounded_incremental_common_ATE_difference_m','rounded_incremental_full_ATE_difference_m')),
         evo_max_difference_m=max(v['evo_alignment_difference_m'] for d in cases.values() for block in ('common','full_saved') for v in d[block].values()),
         direct_RPE_max_difference=max(v['direct_RPE_check_max'] for d in cases.values() for v in d['common'].values()),
         input_sha256=pins)
m.a.save(H/'verification.json',out)
print(json.dumps({k:v for k,v in out.items() if k!='input_sha256'},indent=2))
