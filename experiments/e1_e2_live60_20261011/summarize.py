"""Post-validation historical comparisons; no new estimator/scoring run."""
import csv
import importlib.util
import json
from pathlib import Path
import numpy as np

WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN=WS/'runs/gnssfgo_e1_e2_live60_20261011'
ROOT=RUN/'prepared'
TAG='live60_on'
spec=importlib.util.spec_from_file_location('saved_timing',WS/'runs/glim_cbs_diagnostic_budget_20261010/compare_timing.py')
timing=importlib.util.module_from_spec(spec);spec.loader.exec_module(timing)
stats=timing.stats
validation=json.loads((ROOT/(TAG+'_validation.json')).read_text());assert validation['passed']
cases={'original':(WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared','full160_on'),
       'e1_only':(WS/'runs/gnssfgo_e1_live60_20261011/prepared',TAG),
       'e1_e2':(ROOT,TAG)}
data={k:timing.read(root/(tag+'_source')) for k,(root,tag) in cases.items()}
t0=1727775008.003475
common=sorted(set.intersection(*[set(d['scan']) for d in data.values()]))
assert common
for i in common:assert len({d['scan'][i]['stamp'] for d in data.values()})==1
windows=[]
for lo,hi in [(0,60),(0,20),(20,40),(40,60)]:
    ids=[i for i in common if lo<=data['e1_e2']['scan'][i]['stamp']-t0<hi]
    assert ids
    w={'seconds':[lo,hi],'common_scans':len(ids),'cases':{}}
    for name,d in data.items():
        scan={k:stats([d['scan'][i][k] for i in ids]) for k in ['pre_callback','smoother','post_callback','total','nfactors','active']}
        nested={}
        for kind,cols in {'on_smoother_update':{'sidecar_update':10},
                          'publish_outgoing':{'local_covariance_rebuild':17,'belief_query':18,'ros_publish':20},
                          'on_smoother_update_finish':{'outgoing_plus_revisions':12}}.items():
            rows=[d['cbs'][kind][i][0] for i in ids if i in d['cbs'].get(kind,{})]
            nested.update({k:stats([r[j] for r in rows]) for k,j in cols.items()})
        pub=d['cbs'].get('publish_odometry',{})
        ages=stats([pub[i][1]-d['scan'][i]['stamp'] for i in ids if i in pub])
        w['cases'][name]={'scan':scan,'nested_ms':nested,'native_publication_age_s':ages}
    windows.append(w)
def receiver_stats(root,tag):
    samples=[json.loads(x) for x in (root/(tag+'_control/monitor.jsonl')).open()]
    samples=[r for r in samples if r['published_poses']>0 and 0<r['last_imu_ns']*1e-9-t0<60]
    with (root/(tag+'_receiver/receiver/publication_convergence.csv')).open() as f:
        rows=[r for r in csv.DictReader(f) if 0<float(r['physical_ns'])*1e-9-t0<60]
    return {'sampled_queue':stats([r['queued'] for r in samples]),'output_age_s':stats([r['output_age_seconds'] for r in samples]),
            'publication_refinement_ms':stats([1000*float(r['seconds']) for r in rows]),
            'early_middle_late_ms':{str((lo,hi)):stats([1000*float(r['seconds']) for r in rows if lo<=float(r['physical_ns'])*1e-9-t0<hi]) for lo,hi in [(0,20),(20,40),(40,60)]},
            'max_publication_gradient':max(float(r['gradient_after']) for r in rows)}
result={'passed_validation':True,'windows':windows,
        'timing_scope':'Three historical live captures, same physical scan stamps but DIFFERENT nonlinear Values/arrivals; not randomized A/B.',
        'coverage':validation['coverage'],'receiver':{k:receiver_stats(*v) for k,v in cases.items()},
        'candidate_end_transport':validation['final_transport_status'],
        'headroom':{'limit_p95_ms':800,'observed_ms':windows[0]['cases']['e1_e2']['scan']['total']['p95'],
                    'met':windows[0]['cases']['e1_e2']['scan']['total']['p95']<800},
        'capture_wall_s':json.loads((ROOT/(TAG+'_control/complete.json')).read_text())['seconds'],
        'conservation':{k:validation[k] for k in ['imu_samples','raw_source_ids','versions','active_source_ids','replacements']},
        'numerical_repeat':validation['numerical_repeat']}
with (ROOT/(TAG+'_receiver/receiver/e2_memo.csv')).open() as f:result['cache']=list(csv.DictReader(f))
with (RUN/'engineering_summary.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['windows','numerical_repeat','cache']},indent=2))
for w in windows:
    print('WINDOW',w['seconds'],'scans',w['common_scans'])
    for k,c in w['cases'].items():print(k,'scan',c['scan']['total'],'covprep',c['nested_ms']['local_covariance_rebuild'],'sidecar',c['nested_ms']['sidecar_update'])
