from pathlib import Path
import csv,json,sys,numpy as np
from prepare import ROOT,RUN,TAGS,sha
mode=sys.argv[1];case=ROOT/(mode+'_wnoa');old=RUN/(TAGS[mode]+'_receiver/receiver')
s=json.loads((case/'summary.json').read_text());m=json.loads((ROOT/'manifest.json').read_text())
def rows(p):return list(csv.DictReader(p.open()))
checks={}
for name in ['online_trajectory.tum','online_states.csv','support_clock.csv']:
    a=(case/name).read_bytes().splitlines(keepends=True);b=(old/name).read_bytes().splitlines(keepends=True)
    checks[name]=a==b[:len(a)];assert checks[name],name
if mode=='on':
    for name in ['trajectory.tum','states.csv','source_accounting.csv','causal_dispatch.csv','model_covariance.csv','source_model_covariance.csv','measurement_events.csv']:
        checks[name]=sha(case/name)==sha(old/name);assert checks[name],name
assert s['receiver_model']=='source_wnoa' and s['experimental_qc_diagonal']==[.00375]*3+[.015]*3
assert s['source_model_noise']=={} and s['imu_factor_count']==s['bias_variable_count']==s['acceleration_variable_count']==0
assert s['revision_messages_loaded']==m['inputs'][mode]['counts']['B']
assert s['pending_at_end']==s['revision_duplicates']==s['revision_stale']==0
conv=rows(case/'publication_convergence.csv')
assert len(conv)==s['support_states'] and all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in conv)
active={};replaces=0;clock={int(r['state_index']):int(r['available_timestamp_ns']) for r in rows(case/'support_clock.csv')}
for r in rows(case/'causal_dispatch.csv'):
    assert int(r['arrival_ns'])<=int(r['insertion_ns']) and clock[int(r['required_support'])]<=int(r['insertion_ns'])
    key=tuple(r[k] for k in ['source','session','from_index','to_index'])
    if r['outcome']=='insert':assert key not in active
    else:
        assert r['outcome']=='replace' and key in active and int(r['revision'])>active[key];replaces+=1
    active[key]=int(r['revision'])
assert len(active)==s['beliefs_accepted'] and replaces==s['revision_removals']
unsupported=rows(case/'unsupported_beliefs.csv')
assert s['revision_messages_loaded']==len(active)+replaces+len(unsupported)
report={'passed':True,'identity':checks,'supports':s['support_states'],'active_ids':len(active),'replacements':replaces,'unsupported':len(unsupported),
        'max_gradient':max(float(r['gradient_after']) for r in conv),'maximum_factor_covariance_abs_difference':s['maximum_factor_covariance_abs_difference']}
(ROOT/f'validation_{mode}.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
