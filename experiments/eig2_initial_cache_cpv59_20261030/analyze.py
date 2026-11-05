"""CP59 saved-state revision analysis, not trajectory scoring."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
T=H/'attempt_exact_values'
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'attempt_exact_values/results'
helper=H.parent/'eig2_refinement_refresh_cpv58_20261030/analyze.py'
spec=importlib.util.spec_from_file_location('cp58_geometry',helper)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def main():
    assert json.loads((T/'test_passed.json').read_text())['passed']
    inputs=[p for p in P.iterdir() if p.is_file()]+[T/'test_passed.json',Path(__file__),helper,H/'initial_byte_identity.json']
    pins={str(p):m.sha(p) for p in inputs}
    names=['previous','initial','A_ordinary','A_refresh','B_ordinary','B_refresh','captured_ordinary','captured_refresh']
    states={name:{k:np.array(v) for k,v in json.loads((P/(name+'.json')).read_text()).items()} for name in names}
    prev=states['previous'];keys=sorted(k for k in prev if k.startswith('x'))
    assert keys==['x0','x1','x2','x3','x4','x5']
    for k in prev:assert np.array_equal(prev[k],states['initial'][k]),k
    for degree in [.01,1,10,45,100]:
        v=[0,0,0,np.cos(np.radians(degree)/2),np.sin(np.radians(degree)/2),0,0]
        assert abs(m.angle(m.rotation(v))-degree)<1e-12
    result=json.loads((P/'result.json').read_text());parity=json.loads((P/'original_parity.json').read_text())
    assert result['real_GN_steps']==4 and result['untouched_gaussians']==355 and result['untouched_max_difference']==0
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
    analysis=dict(completed=True,update=6,capture_time_sec=1.599933183,steps=result,parity=parity,arms=arms,
                  A_to_B={phase:m.movement(states['A_'+phase],states['B_'+phase],[f'x{i}' for i in range(7)]) for phase in ['ordinary','refresh']},
                  percent_reduction_of_revision_magnitudes=changes,input_sha256=pins,
                  caveats=['Only571 initial camera-pose cache cleared; no repeated cache clearing or refinement.',
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

if __name__=='__main__':main()
