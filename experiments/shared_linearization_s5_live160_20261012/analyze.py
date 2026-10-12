"""Post-capture timing, delay and history analysis; never launches an estimator."""
from collections import defaultdict
import itertools
import json
from pathlib import Path
import re
import sys
from prepare import HERE,ROOT,RUN,OLD,WS,sha

sys.path.insert(0,str(HERE.parent/'e1b_local_update_audit_20261012'))
from analyze_retained import parse_scan,fields,ns,summary,BASE_NS

def read(p):return json.loads(p.read_text())

def source_log(source):
    log,=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
    return log

def get_timings(source):
    scans=[]
    for line in source_log(source).open():
        scan=parse_scan(line)
        if scan:scans.append(scan)
    assert [r['frame'] for r in scans]==list(range(1,len(scans)+1))
    contexts=defaultdict(list)
    profile=source/'e1b_profile.jsonl'
    if profile.exists():
        for line in profile.open():
            r=json.loads(line)
            assert r['schema']==1 and not r['aborted']
            assert sum(v[2] for v in r['owner_wall'].values())==r['owner_wall']['root'][1]
            contexts[r['context']].append(r)
    blocks={}
    intervals=[(n,n+10) for n in range(0,160,10)]+[(0,40),(40,120),(120,160),(0,160)]
    for lo,hi in intervals:
        rows=[r for r in scans if lo<=r['sec']<hi]
        if not rows:continue
        block=dict(scans=len(rows),timing={k:summary(r[k] for r in rows) for k in
            ['total_ms','main_ms','pre_ms','post_ms','active_frames']},
            over_1000ms=sum(r['total_ms']>1000 for r in rows),contexts={})
        for context,all_rows in contexts.items():
            selected=[r for r in all_rows if lo<=r['timestamp']-BASE_NS/1e9<hi]
            if not selected:continue
            sync_calls=sum(r['owner_wall'].get('gpu_sync_linearize',[0])[0] for r in selected)
            sync_ns=sum(r['owner_wall'].get('gpu_sync_linearize',[0,0])[1] for r in selected)
            block['contexts'][str(context)]=dict(rows=len(selected),
                root_ms=summary(r['owner_wall']['root'][1]/1e6 for r in selected),
                sum_root_s=sum(r['owner_wall']['root'][1] for r in selected)/1e9,
                gpu_linearize_calls=sum(r['owner_wall'].get('gpu_linearize',[0])[0] for r in selected),
                gpu_sync_calls=sync_calls,gpu_sync_ms=sync_ns/1e6,
                gpu_sync_ms_per_call=sync_ns/sync_calls/1e6 if sync_calls else None,
                gpu_cache_hits=sum(r['metrics'][7] for r in selected),
                sync_calls_per_update=summary(r['owner_wall'].get('gpu_sync_linearize',[0])[0] for r in selected),
                active_keys=summary(r['metrics'][0] for r in selected),
                nonnull_factors=summary(r['metrics'][2] for r in selected))
        blocks[f'{lo}-{hi}']=block
    return dict(scans=len(scans),span_s=scans[-1]['sec']-scans[0]['sec'],
        sum_scan_s=sum(r['total_ms'] for r in scans)/1000,blocks=blocks,
        log_sha256=sha(source_log(source)))

def get_delays():
    source=ROOT/'capacity160_on_source';live=ROOT/'capacity160_on_receiver'
    producers={}
    for line in source_log(source).open():
        for marker in ['GLIM_CBS_EXPORT_BELIEF_ROW,','GLIM_CBS_REVISION_BELIEF_ROW,']:
            f=fields(line,marker)
            if f:
                assert len(f)==10 and f[0]=='g'
                key=(int(f[1]),int(f[2]),int(f[3]))
                assert key not in producers
                producers[key]=dict(record=ns(f[4]),tj=ns(f[7]))
    all_first=defaultdict(list);glim_first=[];seen=set();edge_end={}
    for index,(event,receipt) in enumerate(itertools.zip_longest((live/'events.csv').open(),(live/'receipts.jsonl').open())):
        assert event is not None and receipt is not None
        f=event.strip().split(',');r=json.loads(receipt)
        assert index==r['event'] and f[0]==r['kind'] and int(f[1])==r['receipt_ros_ns']
        if f[0]!='B':continue
        end=int(f[9]);sec=(end-BASE_NS)/1e9
        if int(f[5])==0:
            all_first[f[2]].append(dict(sec=sec,age_s=(int(f[1])-end)/1e9))
        if f[2]!='glim_hesai':continue
        meta=r['metadata'];key=(int(f[6]),int(f[7]),int(meta['source_revision']))
        assert key not in seen;seen.add(key)
        p=producers[key]
        assert p['record']==ns(meta['source_record']) and p['tj']==end
        edge_end[key[:2]]=end
        if key[2]==0:
            glim_first.append(dict(sec=sec,producer_age_s=(p['record']-end)/1e9,
                logtail_s=(int(f[1])-p['record'])/1e9))
    assert seen==producers.keys()
    blocks={}
    for lo,hi in [(n,n+10) for n in range(0,160,10)]+[(0,40),(40,120),(120,160),(0,160)]:
        chosen=[r for r in glim_first if lo<=r['sec']<hi]
        blocks[f'{lo}-{hi}']=dict(glim={k:summary(r[k] for r in chosen) for k in ['producer_age_s','logtail_s']},
            first_observed_age_s={sid:summary(r['age_s'] for r in rows if lo<=r['sec']<hi)
                 for sid,rows in all_first.items()})
    return dict(matched_glim_versions=len(seen),first_glim_versions=len(glim_first),blocks=blocks),edge_end

def main():
    assert read(RUN/'validation_logs/complete.json')['passed']
    v=read(ROOT/'capacity160_on_validation.json')
    assert v['passed'] and all(r['byte_identical'] for r in v['repeat_files'].values())
    for p,h in read(ROOT/'protected_manifest.json').items():assert sha(p)==h,p
    current=get_timings(ROOT/'capacity160_on_source')
    previous=get_timings(OLD/'prepared/profile40_on_source')
    b0=get_timings(WS/'runs/gnssfgo_shared_linearization_b0_20261012/prepared/profile40_on_source')
    e1e2=get_timings(WS/'runs/gnssfgo_e1_e2_live160_20261011/prepared/live160_on_source')
    delays,edge_end=get_delays()
    provider=read(ROOT/'capacity160_on_provider_validation.json')
    history={}
    for lo,hi in [(n,n+10) for n in range(0,160,10)]+[(0,40),(40,120),(120,160),(0,160)]:
        selected=[r for r in provider['rows'] if lo<=(edge_end[tuple(r['edge'])]-BASE_NS)/1e9<hi]
        history[f'{lo}-{hi}']={k:summary(r[k] for r in selected) for k in ['active','admitted','absorbed','removed','query_ms']}
    memory=defaultdict(list)
    for line in (RUN/'memory_observations.jsonl').open():
        row=json.loads(line)
        for container,entries in row['containers'].items():
            assert isinstance(entries,list),(container,entries)
            for entry in entries:memory[Path(entry['executable']).name].append(entry)
    memory_summary={name:dict(samples=len(rows),first_rss_mib=rows[0]['rss_kib']/1024,
        last_rss_mib=rows[-1]['rss_kib']/1024,max_sampled_rss_mib=max(r['rss_kib'] for r in rows)/1024,
        max_observed_process_peak_rss_mib=max(r['peak_rss_kib'] for r in rows)/1024) for name,rows in memory.items()}
    result=dict(functional_validation_passed=True,timing=current,previous_S4d=previous,B0=b0,
        historical_E1_E2_live160=e1e2,delays=delays,history=history,memory=memory_summary,
        performance_gates=dict(full_scan_p95_under800ms=current['blocks']['0-160']['timing']['total_ms']['p95']<800,
            late_scan_p95_under800ms=current['blocks']['120-160']['timing']['total_ms']['p95']<800),
        controller=read(ROOT/'capacity160_on_control/complete.json'),
        provider={k:v for k,v in provider.items() if k!='rows'},
        receiver={k:v[k] for k in ['versions','active_source_ids','replacements','max_publication_gradient',
            'live_output_age_s','final_transport_status','repeat_files','coverage','source_lifecycle_counts']},
        limitations=['Different coupled source captures, not matched-input speed attribution.',
            'Native Gaussian capture/commit is included in MAIN; scope timings are nested.',
            'Later diagnostic revision covariance remains MAIN, not the ordinary provider.',
            'Memory observation starts after playback admission; sampled RSS is not GPU memory.',
            'No GT accuracy claim or universal capacity certification.'])
    with (HERE/'summary.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(dict(performance_gates=result['performance_gates'],
        scan_ms=current['blocks']['0-160']['timing']['total_ms'],
        late_scan_ms=current['blocks']['120-160']['timing']['total_ms'],
        provider=result['provider'],memory=memory_summary),indent=2))

if __name__=='__main__':main()
