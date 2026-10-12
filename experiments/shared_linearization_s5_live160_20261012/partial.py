"""Read-only evidence from an aborted capture, never a full-run validator."""
from collections import defaultdict
import csv
import json
from pathlib import Path
import re
import sys
from prepare import HERE,ROOT,RUN,OLD,WS,sha
sys.path.insert(0,str(HERE.parent/'e1b_local_update_audit_20261012'))
from analyze_retained import parse_scan,fields,ns,summary,BASE_NS
from analyze import source_log

def read(p):return json.loads(p.read_text())
source=ROOT/'capacity160_on_source';live=ROOT/'capacity160_on_receiver';control=ROOT/'capacity160_on_control'
assert (control/'failed.txt').read_text().strip()=='live queue gate exceeded'
assert not (control/'complete.json').exists()
scans=[];queries=[];bad_tokens=defaultdict(int);query_edges={};producer_first={}
for line in source_log(source).open():
    scan=parse_scan(line)
    if scan:scans.append(scan)
    for token in ['SHARED_NATIVE_SHADOW_FAILURE','SHARED_NATIVE_SHADOW_SETUP_FAILURE','SHARED_NATIVE_PROVIDER_FAILURE']:
        if token in line:bad_tokens[token]+=1
    f=fields(line,'GLIM_CBS_EXPORT_BELIEF_ROW,')
    if f:producer_first[(int(f[1]),int(f[2]))]=dict(tj=ns(f[7]),record=ns(f[4]))
    if 'SHARED_NATIVE_PROVIDER_QUERY ' in line:
        clean=re.sub(r'\x1b\[[0-9;]*m','',line).split('SHARED_NATIVE_PROVIDER_QUERY ',1)[1]
        row=dict(p.split('=',1) for p in clean.split())
        row.pop('matrix');row={k:float(v) if k=='query_ms' else int(v) for k,v in row.items()}
        row['edge']=(row['from']&((1<<56)-1),row['to']&((1<<56)-1))
        queries.append(row)
last=scans[-1]['sec'];profile=defaultdict(list);incomplete=0;aborted=0
for line in (source/'e1b_profile.jsonl').open():
    try:r=json.loads(line)
    except json.JSONDecodeError:incomplete+=1;continue
    if r['aborted']:aborted+=1;continue
    assert sum(v[2] for v in r['owner_wall'].values())==r['owner_wall']['root'][1]
    profile[r['context']].append(r)
convergence=list(csv.DictReader((live/'receiver/publication_convergence.csv').open()))
supports=[r for r in convergence if r['phase']=='support']
monitor=[json.loads(l) for l in (control/'monitor.jsonl').open()]
historic=[]
oldsource=WS/'runs/gnssfgo_e1_e2_live160_20261011/prepared/live160_on_source'
for line in source_log(oldsource).open():
    r=parse_scan(line)
    if r and r['sec']<=last:historic.append(r)
blocks={}
for lo,hi in [(n,min(n+10,last+1e-6)) for n in range(0,150,10) if n<last]+[(0,40),(40,120),(120,last+1e-6),(0,last+1e-6)]:
    selected=[s for s in scans if lo<=s['sec']<hi]
    old=[s for s in historic if lo<=s['sec']<hi]
    pub=[r for r in supports if lo<=(int(r['physical_ns'])-BASE_NS)/1e9<hi]
    mon=[r for r in monitor if lo<=(r['last_imu_ns']-BASE_NS)/1e9<hi and r['published_poses']>0]
    q=[r for r in queries if lo<=(producer_first[r['edge']]['tj']-BASE_NS)/1e9<hi]
    b=dict(actual_range_s=[lo,hi],scans=len(selected),
        scan_ms=summary(s['total_ms'] for s in selected),main_ms=summary(s['main_ms'] for s in selected),
        active_frames=summary(s['active_frames'] for s in selected),
        over1000ms=sum(s['total_ms']>1000 for s in selected),
        historical_E1_E2_scan_ms=summary(s['total_ms'] for s in old),
        receiver_refinement_ms=summary(float(r['seconds'])*1000 for r in pub),
        receiver_refinement_attempts=summary(int(r['attempted']) for r in pub),
        receiver_refinement_over1000ms=sum(float(r['seconds'])>1 for r in pub),
        queue=summary(r['queued'] for r in mon),output_age_s=summary(r['output_age_seconds'] for r in mon),
        provider={k:summary(r[k] for r in q) for k in ['query_ms','active','admitted','absorbed','removed']},contexts={})
    for c,rows in profile.items():
        rows=[r for r in rows if lo<=r['timestamp']-BASE_NS/1e9<hi]
        b['contexts'][str(c)]=dict(rows=len(rows),root_ms=summary(r['owner_wall']['root'][1]/1e6 for r in rows),
            root_sum_s=sum(r['owner_wall']['root'][1] for r in rows)/1e9,
            gpu_linearize_calls=sum(r['owner_wall'].get('gpu_linearize',[0])[0] for r in rows),
            gpu_sync_calls=sum(r['owner_wall'].get('gpu_sync_linearize',[0])[0] for r in rows),
            gpu_cache_hits=sum(r['metrics'][7] for r in rows),
            sync_calls_per_update=summary(r['owner_wall'].get('gpu_sync_linearize',[0])[0] for r in rows))
    blocks[f'{lo}-{hi:.6f}']=b
events=defaultdict(int)
for line in (live/'events.csv').open():events[line.split(',',1)[0]]+=1
summ=read(live/'receiver/summary.json')
memory=defaultdict(list)
for line in (RUN/'memory_observations.jsonl').open():
    r=json.loads(line)
    for container,entries in r['containers'].items():
        assert isinstance(entries,list),(container,entries)
        for e in entries:memory[Path(e['executable']).name].append(e)
memory_summary={k:dict(samples=len(v),first_rss_mib=v[0]['rss_kib']/1024,last_rss_mib=v[-1]['rss_kib']/1024,
    sampled_max_rss_mib=max(e['rss_kib'] for e in v)/1024,
    observed_peak_rss_mib=max(e['peak_rss_kib'] for e in v)/1024) for k,v in memory.items()}
coverage={}
for role,p in [('fused_online',live/'receiver/online_trajectory.tum'),('fused_final',live/'receiver/trajectory.tum')]:
    rows=[list(map(float,l.split())) for l in p.open() if l.strip() and not l.startswith('#')]
    coverage[role]=dict(rows=len(rows),span_s=rows[-1][0]-rows[0][0],sha256=sha(p))
result=dict(status='CAPACITY_FAILED_PARTIAL_CAPTURE',failure='live queue gate exceeded',
    requested_duration_s=160,last_recorded_input_elapsed_s=(monitor[-1]['last_imu_ns']-BASE_NS)/1e9,
    last_completed_glim_scan_elapsed_s=last,scans=len(scans),ordinary_queries=len(queries),
    provider_failure_tokens=dict(bad_tokens),provider_resets=max(r['resets'] for r in queries),
    all_query_counter_balances=all(r['admitted']==r['active']+r['absorbed']+r['removed'] for r in queries),
    last_query={k:v for k,v in queries[-1].items() if k not in ['from','to']},
    truncated_profile_lines=incomplete,aborted_profile_rows=aborted,local_update_rows=len(profile[2]),
    publication_records=len(convergence),support_publications=len(supports),
    all_recorded_publications_stationary=all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in convergence),
    maximum_recorded_gradient=max(float(r['gradient_after']) for r in convergence),
    maximum_queue=max(r['max_queue'] for r in monitor),
    maximum_queue_scope='controller observation at abort',
    final_transport_maximum_queue=read(live/'transport_complete.json')['max_queue'],
    last_live_status=read(live/'transport_status.json'),
    partial_receiver_summary={k:summ[k] for k in ['support_states','beliefs_accepted','revision_messages_loaded','revision_removals','pending_at_end']},
    recorded_event_counts=dict(events),receiver_partial_marker=(live/'complete.marker').exists(),
    controller_complete=False,source_complete=(source/'complete.marker').exists(),
    covariance_wire_validation='not run after abort',deterministic_repeat='not run after abort',
    blocks=blocks,memory=memory_summary,partial_receiver_coverage=coverage,
    caveats=['Partial diagnostic, not the requested full160 capacity validation.',
       'The transport status froze at abort and is not a final drained-queue measurement.',
       'No receiver complete.marker exists; transport_complete.json and journal E record abort drain only.',
       'Historical timing is restricted to the observed physical prefix but is from a different coupled capture.',
       'No source rerun, model edit, covariance tuning, full numerical validation or GT scoring.'])
with (HERE/'partial_summary.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ['blocks','last_live_status','memory','caveats']},indent=2))
