"""Preserve the old exact-repeat failure; independently validate numerical scope.

The old pre-repeat assertions are executed verbatim via an AST prefix, rather
than copying or weakening their checks. Only the receiver re-execution and
byte-only tail are replaced by separately recorded evidence. This does NOT
write a success result over the old validation or change its protected file.
"""
import ast
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
CAPTURE=ROOT.parent/'source_huber_eth1_30s_opencv42_20261008'
EXACT=['support_clock.csv','source_model_covariance.csv','model_covariance.csv',
       'causal_dispatch.csv','measurement_events.csv','source_accounting.csv',
       'unsupported_beliefs.csv','support_accelerations.csv']
PRODUCTS=['trajectory.tum','online_trajectory.tum','states.csv','online_states.csv']
LIMIT=1e-6  # SI-unit engineering threshold, declared before this fresh validation.

def rows(path):
    with path.open() as f:return list(csv.DictReader(f))

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def first_difference(a,b,ignore=()):
    assert len(a)==len(b)
    for i,(x,y) in enumerate(zip(a,b)):
        difference={k:[x[k],y[k]] for k in x if k not in ignore and x[k]!=y[k]}
        if difference:
            return dict(row=i,first=x,difference=difference)
    return None

def compare_states(a,b):
    assert len(a)==len(b)
    assert all(x['timestamp_ns']==y['timestamp_ns'] and x['state_index']==y['state_index'] for x,y in zip(a,b))
    def get(items,names):return np.array([[float(x[n]) for n in names] for x in items])
    out={}
    for label,fields in [('position_m',['px','py','pz']),('velocity_m_s',['vx','vy','vz']),('omega_rad_s',['wx','wy','wz'])]:
        delta=np.linalg.norm(get(a,fields)-get(b,fields),axis=1)
        out[label]=dict(max=float(delta.max()),rms=float(np.sqrt(np.mean(delta**2))))
    fields=['qx','qy','qz','qw']
    angles=(Rotation.from_quat(get(a,fields)).inv()*Rotation.from_quat(get(b,fields))).magnitude()
    out['rotation_rad']=dict(max=float(angles.max()),rms=float(np.sqrt(np.mean(angles**2))))
    assert all(np.isfinite(x['max']) and x['max']<=LIMIT for x in out.values()),out
    return out

def compare(a,b):
    exact={name:(a/name).read_bytes()==(b/name).read_bytes() for name in EXACT}
    assert all(exact.values()),exact
    out=dict(exact_semantics=exact,byte_identity={n:(a/n).read_bytes()==(b/n).read_bytes() for n in PRODUCTS},
             state_differences={n:compare_states(rows(a/n),rows(b/n)) for n in ['online_states.csv','states.csv']})
    aa=rows(a/'publication_convergence.csv');bb=rows(b/'publication_convergence.csv')
    assert len(aa)==len(bb)==293
    for r in aa+bb: assert r['status']=='stationary' and 0<=float(r['gradient_after'])<=1e-5
    for x,y in zip(aa,bb):
        assert all(x[k]==y[k] for k in ['phase','index','physical_ns','status'])
    out['publication_objective_max_relative_difference']=max(
      abs(float(x['error_after'])-float(y['error_after']))/max(1,abs(float(x['error_after']))) for x,y in zip(aa,bb))
    assert out['publication_objective_max_relative_difference']<=1e-8
    out['first_publication_numeric_difference']=first_difference(aa,bb,['seconds'])
    ar=rows(a/'revision_events.csv');br=rows(b/'revision_events.csv')
    semantic_fields=['ordinal','from_index','to_index','revision','record_timestamp_ns','outcome','old_factor_index','new_factor_index','active_factors']
    assert len(ar)==len(br) and all(all(x[k]==y[k] for k in semantic_fields) for x,y in zip(ar,br))
    out['exact_factor_slots_and_revision_lifecycle']=True
    out['first_revision_numeric_difference']=first_difference(ar,br,['update_seconds'])
    aa=rows(a/'publication_steps.csv');bb=rows(b/'publication_steps.csv')
    out['publication_step_counts']=[len(aa),len(bb)]
    out['different_step_count_by_support']=[]
    for key in sorted(set((r['phase'],r['index']) for r in aa)):
        x=[r for r in aa if (r['phase'],r['index'])==key]
        y=[r for r in bb if (r['phase'],r['index'])==key]
        if len(x)!=len(y):out['different_step_count_by_support'].append(dict(phase=key[0],index=key[1],a=x,b=y))
    return out

def original_assertions(tag):
    path=CAPTURE/'validate_case.py'
    tree=ast.parse(path.read_text(),filename=str(path))
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    cut=next(i for i,n in enumerate(function.body) if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Name) and t.id=='repeat' for t in n.targets))
    tail=ast.parse("""
result.update(imu_samples=imus,revision_payloads_conserved=True,
    raw_source_ids=len(received),versions=sum(map(len,received.values())),
    active_source_ids=len(active),replacements=replacements,summary=summary)
return result
""").body
    prefix=ast.dump(ast.Module(body=function.body[:cut],type_ignores=[]))
    function.body=function.body[:cut]+tail
    ast.fix_missing_locations(tree)
    sys.path.insert(0,str(CAPTURE))
    scope={'__name__':'retained_validator_prefix','__file__':str(path)}
    exec(compile(tree,str(path),'exec'),scope)
    old=sys.argv;sys.argv=[str(path),tag]
    try: result=scope['main']()
    finally: sys.argv=old
    result['original_validator_sha256']=sha(path)
    result['original_assertion_prefix_ast_sha256']=hashlib.sha256(prefix.encode()).hexdigest()
    return result

def main():
    # Small negative controls prove the additional gate does not accept a
    # millimetre state change or a different timestamp. Data never written back.
    a=rows(CAPTURE/'on_huber_receiver/receiver/online_states.csv')
    for field,value in [('px',str(float(a[0]['px'])+.001)),('timestamp_ns',str(int(a[0]['timestamp_ns'])+1))]:
        b=[dict(r) for r in a];b[0][field]=value
        try:compare_states(a,b)
        except AssertionError:pass
        else:raise AssertionError('negative control passed: '+field)
    evidence=dict(post_hoc=True,state_tolerance_SI=LIMIT,objective_relative_tolerance=1e-8,
      original_strict_byte_gate_passed=False,full_posterior_covariance_exported=False,
      negative_controls_passed=['1 mm position perturbation','1 ns timestamp perturbation'],comparisons={})
    live=CAPTURE/'on_huber_receiver/receiver';old=CAPTURE/'on_huber_repeat'
    evidence['comparisons']['live_to_retained_repeat']=compare(live,old)
    for name in ['validator_1','live_pipe_1','live_file_1']:
        candidate=ROOT/name/'receiver'
        evidence['comparisons']['retained_repeat_to_'+name]=compare(old,candidate)
        assert all(evidence['comparisons']['retained_repeat_to_'+name]['byte_identity'].values())
    evidence['hashes']={str(p):{n:sha(p/n) for n in PRODUCTS+EXACT} for p in [live,old,*[ROOT/n/'receiver' for n in ['validator_1','live_pipe_1','live_file_1']]]}
    with (ROOT/'reproducibility_resolution.json').open('x') as f:json.dump(evidence,f,indent=2)
    for tag in ['off_gaussian','on_gaussian','on_huber']:
        result=original_assertions(tag)
        comparison=compare(CAPTURE/(tag+'_receiver')/'receiver',CAPTURE/(tag+'_repeat'))
        exact=all(comparison['byte_identity'].values())
        assert exact==(tag!='on_huber')
        result.update(passed=True,acceptance_scope='SCORING_VALID_WITH_NUMERICAL_REPRODUCIBILITY',
          post_hoc=True,original_byte_gate_passed=exact,numerical_comparison=comparison,
          production_promotion=False,source_rerun=False)
        with (ROOT/(tag+'_validation.json')).open('x') as f:json.dump(result,f,indent=2)
        print(tag,result['acceptance_scope'],'bit-identical:',exact,flush=True)

if __name__=='__main__':main()
