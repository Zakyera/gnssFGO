"""Read completed CP-V37 results, verify accounting, retain compact evidence."""
import csv
import hashlib
import json
import math
from pathlib import Path

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_native_refresh_cpv37_20261029')


def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def pairs(p):
    return {k:float(v) for k,v in csv.reader(p.open())}


def rows(p):
    return list(csv.DictReader(p.open()))


def movement(p):
    r=rows(p)
    out={}
    for kind in ['x','v','b']:
        group=[x for x in r if x['type']==kind]
        out[kind+'_max_delta']=max(float(x['position_or_vector_delta']) for x in group)
    poses=[x for x in r if x['type']=='x']
    for key in ['rotation_delta_deg','relative_translation_change','relative_rotation_change_deg']:
        out['max_'+key]=max(float(x[key]) for x in poses)
    out['newest_pose']=poses[-1]
    return out


assert json.loads((H/'finished.json').read_text())['passed']
old=json.loads((H.parent/'eig2_counterfactual_cpv36_20261029/diagnostic_results_v2.json').read_text())
old={int(x['update']):x for x in old['checkpoints']}
summary=dict(checkpoints=[],source_runs=0,cbs_on_runs=0,nonlinear_iterations=0,
             single_undamped_steps_per_checkpoint=1,weights_changed=False,
             native_smart_factors_retained=True,new_accuracy_metrics=False)
times=[160.4,187.2,200.4,267.4,267.6,267.8,268.0]
for k,t in zip([800,934,1000,1335,1336,1337,1338],times):
    d=P/'results'/f'update_{k}'
    assert (d/'complete.txt').exists()
    assert sha(d/'input_before.bin')==sha(d/'input_after.bin')
    a=dict(update=k,approx_time_s=t,comparison=pairs(d/'comparison.txt'),branches={})
    checks=rows(d/'prior_checks.csv')
    assert checks and all(x['pass']=='1' for x in checks)
    a['prior_checks']=dict(count=len(checks),energy_relative_max=max(float(x['energy_relative_error']) for x in checks),derivative_absolute_max=max(float(x['derivative_absolute_error']) for x in checks))
    factors={}
    for branch in ['captured','stored_control','refreshed_once']:
        if (d/(branch+'_failed.txt')).exists():
            a['branches'][branch]={'failure':(d/(branch+'_failed.txt')).read_text()}
            continue
        metrics=pairs(d/(branch+'_summary.txt'))
        f=rows(d/(branch+'_factors.csv'))
        assert len(f)==metrics['factors']
        assert abs(math.fsum(float(x['cost']) for x in f)-metrics['total_cost'])<=1e-8*(1+abs(metrics['total_cost']))
        factors[branch]={int(x['slot']):x for x in f}
        a['branches'][branch]={'metrics':metrics}
        if branch!='captured':
            a['branches'][branch]['equations']=pairs(d/(branch+'_equations.txt'))
            a['branches'][branch]['movement']=movement(d/(branch+'_movement.csv'))
            assert list(factors[branch])==list(factors['captured'])
            assert all(x['type']==factors['captured'][i]['type'] for i,x in factors[branch].items())
            smart=[i for i,x in factors['captured'].items() if x['type']=='smart']
            assert len(smart)==a['comparison']['native_smart_retained']
            transitions={}
            for i in smart:
                key=factors['captured'][i]['valid']+'->'+factors[branch][i]['valid']
                transitions[key]=transitions.get(key,0)+1
            a['branches'][branch]['smart_validity_transitions']=transitions
    b=a['branches']['captured']['metrics']
    expected=float(old[k]['starts']['incremental_result']['model']['native_initial_cost'])
    assert abs(b['total_cost']-expected)<=1e-10*(1+abs(expected))
    if 'metrics' in a['branches']['refreshed_once']:
        c=a['branches']['refreshed_once']['metrics']
        a['imu_position_residual_reduction_percent']=100*(1-c['max_imu_position_m']/b['max_imu_position_m'])
        a['total_native_cost_reduction_percent']=100*(1-c['total_cost']/b['total_cost'])
    summary['checkpoints'].append(a)

summary['prior_checks_total']=sum(x['prior_checks']['count'] for x in summary['checkpoints'])
summary['successful_refresh_steps']=sum('metrics' in x['branches']['refreshed_once'] for x in summary['checkpoints'])
summary['input_snapshots_unchanged']=True
with (H/'results.json').open('x') as f:json.dump(summary,f,indent=2)
for x in summary['checkpoints']:
    a=x['branches']['captured']['metrics'];b=x['branches']['refreshed_once']['metrics']
    print(x['update'], 'IMU m',a['max_imu_position_m'],'->',b['max_imu_position_m'],
          'reduction%',x['imu_position_residual_reduction_percent'],
          'movement',x['branches']['refreshed_once']['movement'],
          'smart',x['branches']['refreshed_once']['smart_validity_transitions'])
