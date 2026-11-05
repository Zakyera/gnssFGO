"""CP61 local-coordinate saved-event analysis; no trajectory scoring."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'results'
helper=H.parent/'eig2_refinement_refresh_cpv58_20261030/analyze.py'
spec=importlib.util.spec_from_file_location('cp58_geometry',helper)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def rows(name):return [json.loads(l) for l in (P/name).open()]
def main():
    assert json.loads((H/'test_passed.json').read_text())['passed']
    pins={str(p):m.sha(p) for p in list(P.iterdir())+[H/'test_passed.json',Path(__file__),helper] if p.is_file()}
    names=['previous','initial','translated_initial']+[a+'_'+p for a in 'ABC' for p in ['ordinary','refresh']]+['C_ordinary_shifted','C_refresh_shifted']
    states={n:{k:np.array(v) for k,v in json.loads((P/(n+'.json')).read_text()).items()} for n in names}
    prev=states['previous'];keys=[f'x{i}' for i in range(6)]
    assert sorted(k for k in prev if k.startswith('x'))==keys
    for k in prev:assert np.array_equal(prev[k],states['initial'][k])
    result=json.loads((P/'result.json').read_text());parity=json.loads((P/'original_parity.json').read_text())
    assert result['real_GN_steps']==6 and all(c['factors']==338 for c in result['controls'])
    assert parity['ordinary_equation_relative']<1e-12 and parity['refresh_exact_equation_relative']<1e-12
    assert parity['ordinary_state_tangent']<1e-6 and parity['refresh_state_tangent']<1e-5
    assert result['B_C_ordinary_tangent']<1e-6 and result['B_C_refresh_tangent']<1e-6
    arms={}
    for arm in 'ABC':
        ordinary=states[arm+'_ordinary'];final=states[arm+'_refresh']
        data=dict(ordinary=m.movement(prev,ordinary,keys),refresh=m.movement(ordinary,final,keys),net=m.movement(prev,final,keys),
                  newest_ordinary=m.movement(states['initial'],ordinary,['x6']),newest_refresh=m.movement(ordinary,final,['x6']))
        for phase in ['ordinary','refresh']:data[phase+'_postevaluation']=m.costs(rows(arm+'_'+phase+'_factors.jsonl'))
        arms[arm]=data
    decisions={}
    for phase in ['ordinary','refresh']:
        b=rows('B_'+phase+'_decisions.jsonl');c=rows('C_'+phase+'_decisions.jsonl')
        assert len(b)==len(c)==338
        assert b==c,('B/C cache/used status divergence',phase)
        postb=rows('B_'+phase+'_factors.jsonl');postc=rows('C_'+phase+'_factors.jsonl')
        assert [(r['slot'],r['status']) for r in postb]==[(r['slot'],r['status']) for r in postc]
        original={r['slot']:r['status'] for r in rows('A_ordinary_factors.jsonl') if r['type'].startswith('smart')}
        decisions[phase]=dict(retriangulated=sum(r['due'] for r in b),candidate_valid=sum(r['after_status']==0 for r in b))
        if phase=='refresh':
            decisions[phase]['versus_original_refresh_status_changes']=[
                dict(slot=r['slot'],original=original[r['slot']],candidate=r['after_status']) for r in b if original[r['slot']]!=r['after_status']]
    cc=rows('coordinate_controls.jsonl');assert len(cc)==2028
    assert all(r['A_status']==r['B_status'] and r['point_difference_m']<1e-7 and r['equation_relative']<1e-7 and r['cost_relative']<1e-7 for r in cc)
    cold=rows('local_geometry.jsonl');assert len(cold)==1014
    change={phase:{k:100*(arms['A'][phase]['maxima'][k]-arms['B'][phase]['maxima'][k])/arms['A'][phase]['maxima'][k]
                  for k in arms['A'][phase]['maxima']} for phase in ['ordinary','refresh','net']}
    analysis=dict(completed=True,update=6,capture_time_sec=1.599933183,steps=result,parity=parity,arms=arms,
                  decisions=decisions,coordinate_controls=dict(checks=len(cc),max_point_m=max(r['point_difference_m'] for r in cc),
                        max_equation_relative=max(r['equation_relative'] for r in cc),max_cost_relative=max(r['cost_relative'] for r in cc)),
                  cold_candidate_valid={phase:sum(r['status']==0 for r in cold if r['phase']==phase) for phase in ['initial','postordinary','postrefresh']},
                  A_to_B={p:m.movement(states['A_'+p],states['B_'+p],[f'x{i}' for i in range(7)]) for p in ['ordinary','refresh']},
                  B_to_C={p:m.movement(states['B_'+p],states['C_'+p],[f'x{i}' for i in range(7)]) for p in ['ordinary','refresh']},
                  percent_reduction_of_revision_magnitudes=change,input_sha256=pins,
                  caveats=['Candidate chart changes native algebraic point/rank decisions; identical thresholds do not mean identical classifications.',
                           'Historical initial caches are unchanged; candidate applies only at normal freshness trigger. Not propagated startup history.',
                           'Revision magnitudes and residual costs are not ground-truth errors, ATE or full-trajectory performance.',
                           'Postevaluation is on clones, candidate for B/C and original native for A, not another graph solve.',
                           'Smart cost sums depend on rejected support; not an independent merit function.',
                           'Native linear solve checks do not establish nonlinear stationarity.'])
    for p,h in pins.items():assert m.sha(p)==h,p
    with (H/'analysis.json').open('x') as f:json.dump(analysis,f,indent=2,allow_nan=False)
    print('STEPS',result);print('PARITY',parity);print('COORDINATE',analysis['coordinate_controls'])
    print('DECISIONS',decisions);print('COLD_VALID',analysis['cold_candidate_valid'])
    for arm,data in arms.items():
        for phase in ['ordinary','refresh','net']:print(arm,phase,'tilt',data[phase]['common_tilt_deg'],data[phase]['maxima'])
        print(arm,'COSTS',data['refresh_postevaluation'])
    for pair in ['A_to_B','B_to_C']:
        for phase,d in analysis[pair].items():print(pair,phase,d['maxima'])
    print('REDUCTION',change)
if __name__=='__main__':main()

