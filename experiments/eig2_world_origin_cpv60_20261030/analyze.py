"""CP60 coordinate-origin analysis, not trajectory scoring."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
T=H
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'
helper=H.parent/'eig2_refinement_refresh_cpv58_20261030/analyze.py'
spec=importlib.util.spec_from_file_location('cp58_geometry',helper)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def main():
    assert json.loads((T/'test_passed.json').read_text())['passed']
    inputs=[p for p in P.iterdir() if p.is_file()]+[T/'test_passed.json',Path(__file__),helper]
    pins={str(p):m.sha(p) for p in inputs}
    names=['previous','initial','A_ordinary','A_refresh','B_ordinary','B_refresh','translated_initial','B_ordinary_shifted','B_refresh_shifted']
    states={name:{k:np.array(v) for k,v in json.loads((P/(name+'.json')).read_text()).items()} for name in names}
    prev=states['previous'];keys=sorted(k for k in prev if k.startswith('x'))
    assert keys==['x0','x1','x2','x3','x4','x5']
    for k in prev:assert np.array_equal(prev[k],states['initial'][k]),k
    for degree in [.01,1,10,45,100]:
        v=[0,0,0,np.cos(np.radians(degree)/2),np.sin(np.radians(degree)/2),0,0]
        assert abs(m.angle(m.rotation(v))-degree)<1e-12
    result=json.loads((P/'result.json').read_text());parity=json.loads((P/'original_parity.json').read_text())
    assert result['real_GN_steps']==4 and result['translation_m']==[3,-2,1]
    assert all(p['factors']==356 and p['smart']==338 and p['other']==18 for p in result['phases'])
    assert parity['ordinary_equation_relative']<1e-12 and parity['refresh_exact_equation_relative']<1e-12
    assert parity['ordinary_state_tangent']<1e-6 and parity['refresh_state_tangent']<1e-5
    arms={}
    for arm in ['A','B']:
        ordinary=states[arm+'_ordinary'];final=states[arm+'_refresh']
        data=dict(ordinary=m.movement(prev,ordinary,keys),refresh=m.movement(ordinary,final,keys),net=m.movement(prev,final,keys),
                  newest_ordinary=m.movement(states['initial'],ordinary,['x6']),newest_refresh=m.movement(ordinary,final,['x6']))
        for phase in ['ordinary','refresh']:
            rows=[json.loads(l) for l in (P/(arm+'_'+phase+'_factors.jsonl')).open()]
            data[phase+'_postevaluation']=m.costs(rows)
            data[phase+'_postevaluation']['factor571']=next(r for r in rows if r['slot']==571)
        arms[arm]=data
    changes={phase:{metric:100*(arms['A'][phase]['maxima'][metric]-arms['B'][phase]['maxima'][metric])/arms['A'][phase]['maxima'][metric]
                    for metric in arms['A'][phase]['maxima']} for phase in ['ordinary','refresh','net']}
    native=[json.loads(l) for l in (P/'natural_fixed_state.jsonl').open()]
    natural={}
    for phase in ['initial','postordinary','postrefresh']:
        rows=[r for r in native if r['phase']==phase];assert len(rows)==338
        natural[phase]=dict(classification_changes=[r for r in rows if r['A_status']!=r['B_status']],
                            equation_changes_over_1e_8=sum(r['equation_relative']>1e-8 for r in rows),
                            max_equation_relative=max(r['equation_relative'] for r in rows),
                            max_valid_point_difference_m=max(r['point_difference_m'] for r in rows if r['point_difference_m'] is not None))
    analysis=dict(completed=True,update=6,capture_time_sec=1.599933183,steps=result,parity=parity,arms=arms,natural_fixed_physical_state=natural,
                  A_to_B={phase:m.movement(states['A_'+phase],states['B_'+phase],[f'x{i}' for i in range(7)]) for phase in ['ordinary','refresh']},
                  percent_reduction_of_revision_magnitudes=changes,input_sha256=pins,
                  caveats=['Only world-origin representation shifted; pose priors and landmark/camera caches consistently transformed. No cache clearing/refinement.',
                           'Two native GN steps per arm from identical saved initial state, not propagated history.',
                           'Revision magnitudes and costs are not ground-truth errors or ATE.',
                           'Postevaluation on graph clones is not an additional optimizer step.',
                           'Smart cost sums change when factors are rejected; not an independent merit function.',
                           'Linear equation/solve checks do not establish nonlinear stationarity.'])
    for p,h in pins.items():assert m.sha(p)==h,p
    with (H/'analysis.json').open('x') as f:json.dump(analysis,f,indent=2,allow_nan=False)
    print('PARITY',parity);print('STEPS',result)
    for arm,data in arms.items():
        for phase in ['ordinary','refresh','net']:print(arm,phase,'tilt',data[phase]['common_tilt_deg'],data[phase]['maxima'])
        print(arm,'COSTS',data['refresh_postevaluation'])
    for phase,data in analysis['A_to_B'].items():print('A_TO_B',phase,data['maxima'])
    print('REDUCTIONS',changes)
    print('NATURAL_FIXED_STATE',natural)

if __name__=='__main__':main()
