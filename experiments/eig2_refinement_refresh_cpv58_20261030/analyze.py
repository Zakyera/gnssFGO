"""Report CP58's two saved graph steps, not trajectory accuracy or stationarity."""
import collections
import hashlib
import json
from pathlib import Path
import numpy as np

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def rotation(v):
    w,x,y,z=v[3:];assert abs(w*w+x*x+y*y+z*z-1)<1e-10
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def angle(r):
    sine=np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])/2
    return float(np.degrees(np.arctan2(sine,(np.trace(r)-1)/2)))


def movement(a,b,keys):
    mats=[rotation(b[k])@rotation(a[k]).T for k in keys]
    u,_,vt=np.linalg.svd(sum(mats));fix=np.eye(3);fix[2,2]=np.linalg.det(u@vt);R=u@fix@vt
    details={}
    for k,rr in zip(keys,mats):
        n=k[1:]
        details[k]=dict(position_m=float(np.linalg.norm(b[k][:3]-a[k][:3])),
                        rotation_deg=angle(rr),velocity_m_s=float(np.linalg.norm(b['v'+n]-a['v'+n])),
                        accel_bias_m_s2=float(np.linalg.norm(b['b'+n][:3]-a['b'+n][:3])),
                        gyro_bias_rad_s=float(np.linalg.norm(b['b'+n][3:]-a['b'+n][3:])),
                        position_delta=(b[k][:3]-a[k][:3]).tolist(),
                        accel_bias_delta=(b['b'+n][:3]-a['b'+n][:3]).tolist(),
                        gyro_bias_delta=(b['b'+n][3:]-a['b'+n][3:]).tolist())
    return dict(common_rotation_deg=angle(R),common_tilt_deg=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),
                common_rotation=R.tolist(),maxima={key:max(row[key] for row in details.values()) for key in
                   ['position_m','rotation_deg','velocity_m_s','accel_bias_m_s2','gyro_bias_rad_s']},per_state=details)


def costs(rows):
    sums=collections.defaultdict(float)
    for r in rows:sums[r['type']]+=r['cost']
    imu=[r['residual'] for r in rows if r['type']=='imu']
    cbs=next(r for r in rows if r['type']=='CBS')
    return dict(costs=dict(sums),original_rule_smart_status=dict(collections.Counter(str(r['status']) for r in rows if r['type'].startswith('smart'))),
                max_imu_rotation_residual_deg=float(max(np.degrees(np.linalg.norm(v[:3])) for v in imu)),
                max_imu_position_residual_m=float(max(np.linalg.norm(v[3:6]) for v in imu)),
                max_imu_velocity_residual_m_s=float(max(np.linalg.norm(v[6:]) for v in imu)),
                CBS_rotation_residual_deg=float(np.degrees(np.linalg.norm(cbs['residual'][:3]))),
                CBS_translation_log_norm_m=float(np.linalg.norm(cbs['residual'][3:])))


def main():
    assert json.loads((H/'test_passed.json').read_text())['passed']
    inputs=[p for p in P.iterdir() if p.is_file()]+[H/'test_passed.json',Path(__file__),
            H.parent/'eig2_landmark_feasibility_cpv57_20261030/findings.json']
    pins={str(p):sha(p) for p in inputs}
    states={name:{k:np.array(v) for k,v in json.loads((P/(name+'.json')).read_text()).items()}
            for name in ['previous','pre_admission_initial','refresh_start','A_result','B_result','captured_result']}
    previous=states['previous'];start=states['refresh_start'];keys=sorted(k for k in previous if k.startswith('x'))
    assert keys==['x0','x1','x2','x3','x4','x5']
    for key in previous:assert np.array_equal(previous[key],states['pre_admission_initial'][key])
    for degree in [.01,1,10,45,100]:
        v=[0,0,0,np.cos(np.radians(degree)/2),np.sin(np.radians(degree)/2),0,0]
        assert abs(angle(rotation(v))-degree)<1e-12
    result=json.loads((P/'result.json').read_text());parity=json.loads((P/'original_parity.json').read_text())
    assert result['real_GN_steps']==2 and result['restored']==7 and result['untouched_max_difference']==0
    assert parity['equation_relative']<1e-12 and parity['state_tangent']<1e-6
    arms={}
    for arm in ['A','B']:
        final=states[arm+'_result']
        rows=[json.loads(l) for l in (P/(arm+'_original_rule_factors.jsonl')).open()]
        arms[arm]=dict(refresh=movement(start,final,keys),net=movement(previous,final,keys),
                        newest_refresh=movement(start,final,['x6']),original_rule_postevaluation=costs(rows))
    decisions=[json.loads(l) for l in (P/'decisions.jsonl').open()]
    c57=json.loads(inputs[-1].read_text())['records']
    lookup={r['slot']:r for r in c57 if r['phase']=='ordinary_estimate' and r['transform']=='identity'}
    restored=[r['slot'] for r in decisions if r['restored']]
    cohorts=dict(restored_slots=restored,full_stereo=sum(lookup[s]['stereo'] for s in restored),
                 distinct_geometries=len({lookup[s]['geometry_hash'] for s in restored}))
    changes={phase:{metric:100*(arms['A'][phase]['maxima'][metric]-arms['B'][phase]['maxima'][metric])/arms['A'][phase]['maxima'][metric]
                        for metric in arms['A'][phase]['maxima']} for phase in ['refresh','net']}
    analysis=dict(completed=True,update=6,capture_time_sec=1.599933183,steps=result,parity=parity,
                   cohorts=cohorts,ordinary_movement=movement(previous,start,keys),arms=arms,
                   A_to_B=movement(states['A_result'],states['B_result'],[f'x{i}' for i in range(7)]),
                   percent_reduction_of_revision_magnitudes=changes,input_sha256=pins,
                   caveats=['Revision magnitudes are not ground-truth errors.',
                            'Only one refresh from identical ON-conditioned state, not propagated history.',
                            'B restores only seven newly-rejected landmark caches for one native linearization; not an integrated triangulator.',
                            'Postevaluation uses original native DLT in BOTH arms, not another refinement step.',
                            'Smart cost sums vary with rejection and are not an independent merit function.',
                            'Linear solve parity/residuals are not nonlinear stationarity.'])
    for p,h in pins.items():assert sha(p)==h,p
    with (H/'analysis.json').open('x') as f:json.dump(analysis,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in analysis.items() if k not in ['arms','input_sha256','ordinary_movement','A_to_B']},indent=2))
    for arm,data in arms.items():
        for phase in ['refresh','net']:print(arm,phase,'tilt',data[phase]['common_tilt_deg'],data[phase]['maxima'])
        print(arm,'costs',data['original_rule_postevaluation'])
    print('A_to_B',analysis['A_to_B']['maxima'])


if __name__=='__main__':main()
