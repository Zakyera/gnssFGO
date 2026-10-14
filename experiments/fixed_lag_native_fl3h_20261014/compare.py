"""Summarize retained results only. No estimation, alignment, or GT tuning."""
import csv,json,math
from pathlib import Path
import numpy as np
from driver import ROOT,ORACLE,FL3,protected,sha,HERE

def rows(path):
    with path.open() as f:return list(csv.DictReader(f))

def stats(x):
    a=np.asarray(x,float)
    return dict(rms=float(np.sqrt(np.mean(a*a))),median=float(np.median(a)),
                p95=float(np.quantile(a,.95)),max=float(np.max(a)))

def values(path):
    out={}
    with path.open() as f:
        for line in f:
            a=line.split();out[(a[0],int(a[1]))]=np.array(a[2:],float)
    return out

def snapshot_errors(a,b):
    assert a.keys()==b.keys()
    result={k:[] for k in ('position_m','orientation_rad','velocity_mps','omega_radps')}
    for key,v in a.items():
        u=b[key]
        if key[0]=='x':
            v,u=v.reshape(3,4),u.reshape(3,4)
            result['position_m'].append(np.linalg.norm(v[:,3]-u[:,3]))
            r=v[:,:3].T@u[:,:3]
            sine=np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])*.5
            result['orientation_rad'].append(math.atan2(sine,(np.trace(r)-1)*.5))
        else:result['velocity_mps' if key[0]=='v' else 'omega_radps'].append(np.linalg.norm(v-u))
    return {k:stats(v) for k,v in result.items()}

def main():
    assert json.loads((ROOT/'full_validation.json').read_text())['passed']
    service=rows(ROOT/'full_service.csv')
    target=next(r for r in service if int(r['event'])==84958)
    assert int(target['newest'])==1224 and int(target['publications'])==1225
    fl3e=Path('/home/yeranis/results/gnssfgo_fixed_lag_complete_update_fl3e_20261013')
    strict=json.loads((fl3e/'validation.json').read_text())
    result={'same_event_84958':{
        'old_strict_seconds':float(strict['target_event_seconds']),
        'new_incremental_seconds':float(target['seconds']),
        'old_charged_steps':strict['total_steps'],
        'new_extra_retirement_updates':int(target['extra_retirement_updates_max']),
        'retained_state_difference':snapshot_errors(values(ROOT/'full_diagnostics/snapshot_1224.values'),values(fl3e/'diagnostics/after_event.values')),
        'limitation':'Different histories/marginal contracts since initialization; not a same-objective speedup or equivalent optimization certificate.'}}
    errors=rows(ROOT/'full_state_differences.csv')
    result['online_vs_full_history_all']={k:stats([float(r[k]) for r in errors]) for k in errors[0] if k!='index'}
    result['reference_paths_m']={}
    for kind,file in [('online','online_states.csv'),('final_retrospective','states.csv')]:
        a=np.array([[float(r[k]) for k in ('px','py','pz')] for r in rows(ORACLE/file)])
        result['reference_paths_m'][kind]=float(np.linalg.norm(np.diff(a,axis=0),axis=1).sum())
    pub=[r for r in service if int(r['nonempty'])>0]
    result['storage']={k:max(int(r[k]) for r in pub) for k in ('supports','slots','nonempty','marginals','marginal_max_dimension','rss_kib','fixed_keys')}
    result['factor_slots_at_support']={i:next(int(r['slots']) for r in pub if int(r['newest'])==i) for i in (70,200,600,1200,1224,1480)}
    result['old_strict_partial_timing']=json.loads((FL3/'full_timed_timing_analysis.json').read_text())['whole_support_seconds']
    result['gradient_diagnostic']=rows(ROOT/'full_diagnostics/diagnostic.csv')
    result['protected_unchanged']=protected()==json.loads((ROOT/'protected.json').read_text())
    assert result['protected_unchanged']
    result['protected_count']=len(protected())
    result['binary_sha256']=sha(ROOT/'build_replay')
    result['prototype_files_sha256']={p.name:sha(p) for p in HERE.iterdir() if p.is_file()}
    with (ROOT/'comparison.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('prototype_files_sha256','gradient_diagnostic')},indent=2))

if __name__=='__main__':main()
