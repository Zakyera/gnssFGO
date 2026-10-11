#!/usr/bin/env python3
"""Post-validation engineering comparison on common physical scan stamps."""
import csv
import importlib.util
import json
from pathlib import Path
import numpy as np

WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN=WS/'runs/gnssfgo_e1_live60_20261011'
ROOT=RUN/'prepared'
OLD=WS/'runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared'
TAG='live60_on'
spec=importlib.util.spec_from_file_location('saved_timing',WS/'runs/glim_cbs_diagnostic_budget_20261010/compare_timing.py')
timing=importlib.util.module_from_spec(spec);spec.loader.exec_module(timing)
stats=timing.stats
validation=json.loads((ROOT/(TAG+'_validation.json')).read_text())
assert validation['passed']
data={'baseline':timing.read(OLD/'full160_on_source'),
      'candidate':timing.read(ROOT/(TAG+'_source'))}
t0=1727775008.003475
common=sorted(set(data['baseline']['scan'])&set(data['candidate']['scan']))
assert common
for i in common:
    assert data['baseline']['scan'][i]['stamp']==data['candidate']['scan'][i]['stamp'],i
windows=[]
for lo,hi in [(0,60),(0,20),(20,40),(40,60)]:
    ids=[i for i in common if lo<=data['candidate']['scan'][i]['stamp']-t0<hi]
    assert ids
    window={'seconds':[lo,hi],'common_scans':len(ids),'cases':{}}
    for name,d in data.items():
        scan={k:stats([d['scan'][i][k] for i in ids]) for k in
              ['pre_callback','smoother','post_callback','total','nfactors','active']}
        nested={}
        for kind,cols in {
            'on_smoother_update':{'sidecar_update':10},
            'publish_outgoing':{'local_covariance_rebuild':17,'belief_query':18,'ros_publish':20},
            'on_smoother_update_finish':{'outgoing_plus_revisions':12}}.items():
            rows=[d['cbs'][kind][i][0] for i in ids if i in d['cbs'].get(kind,{})]
            nested.update({k:stats([r[j] for r in rows]) for k,j in cols.items()})
        pub=d['cbs'].get('publish_odometry',{})
        ages=stats([pub[i][1]-d['scan'][i]['stamp'] for i in ids if i in pub])
        window['cases'][name]={'scan':scan,'nested_ms':nested,'native_publication_age_s':ages}
    a,b=window['cases']['baseline'],window['cases']['candidate']
    window['median_ratios_candidate_over_baseline']={
        'total_scan':b['scan']['total']['median']/a['scan']['total']['median'],
        'local_covariance_rebuild':b['nested_ms']['local_covariance_rebuild']['median']/a['nested_ms']['local_covariance_rebuild']['median']}
    windows.append(window)

def receiver_stats(root,tag,elapsed_limit):
    samples=[json.loads(x) for x in (root/(tag+'_control/monitor.jsonl')).open()]
    samples=[r for r in samples if r['published_poses']>0 and 0<r['last_imu_ns']*1e-9-t0<elapsed_limit]
    with (root/(tag+'_receiver/receiver/publication_convergence.csv')).open() as f:
        rows=[r for r in csv.DictReader(f) if 0<float(r['physical_ns'])*1e-9-t0<elapsed_limit]
    return {'monitor_samples':len(samples),'sampled_queue':stats([r['queued'] for r in samples]),
            'output_age_s':stats([r['output_age_seconds'] for r in samples]),
            'publication_refinement_ms':stats([1000*float(r['seconds']) for r in rows]),
            'max_publication_gradient':max(float(r['gradient_after']) for r in rows)}
result={'passed_validation':True,'windows':windows,
        'timing_scope':'Historical baseline first60s, same physical scan stamps; not a randomized matched A/B capture.',
        'not_full_sequence_acceptance':True,
        'coverage':validation['coverage'],
        'source_log_summaries':{k:d['summary'] for k,d in data.items()},
        'baseline_warning_count_scope':'Entire143.79s baseline log, not only first60s; do not compare totals as equal-duration rates.',
        'receiver':{'baseline_first60s':receiver_stats(OLD,'full160_on',60),
                    'candidate_first60s':receiver_stats(ROOT,TAG,60)},
        'candidate_end_transport':validation['final_transport_status'],
        'engineering_margin_target':{'p95_scan_ms_limit':800,
            'observed_ms':windows[0]['cases']['candidate']['scan']['total']['p95'],
            'met':windows[0]['cases']['candidate']['scan']['total']['p95']<800},
        'capture_wall_s':json.loads((ROOT/(TAG+'_control/complete.json')).read_text())['seconds'],
        'conservation':{k:validation[k] for k in ['imu_samples','raw_source_ids','versions','active_source_ids','replacements']},
        'numerical_repeat':validation['numerical_repeat']}
with (RUN/'engineering_summary.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ['source_log_summaries','numerical_repeat','windows']},indent=2))
for w in windows:
    print('WINDOW',w['seconds'],'scans',w['common_scans'])
    for k,c in w['cases'].items():
        print(k,'total_ms',c['scan']['total'],'covariance_rebuild_ms',c['nested_ms']['local_covariance_rebuild'],
              'sidecar_update_ms',c['nested_ms']['sidecar_update'])
