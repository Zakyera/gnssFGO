"""Archive bounded evidence and compare complete GLIM scan latency to B0."""
from pathlib import Path
import hashlib,json,shutil,sys
from collections import defaultdict
from prepare import HERE,RUN,ROOT,BUILD,OLD,sha
sys.path.insert(0,str(HERE.parent/'shared_linearization_b0_20261012'))
from analyze_profile import legacy_timers,summary
def read(p):return json.loads(p.read_text())
def timing(run):
    source=run/'prepared/profile40_on_source'
    log,=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
    scans,callback=legacy_timers(log);first=scans[1]['timestamp']
    records=[json.loads(l) for l in (source/'e1b_profile.jsonl').open()]
    contexts=defaultdict(list)
    for r in records:
        assert r['schema']==1 and not r['aborted']
        assert sum(v[2] for v in r['owner_wall'].values())==r['owner_wall']['root'][1]
        contexts[r['context']].append(r)
    result=dict(scans=len(scans),physical_span_s=scans[max(scans)]['timestamp']-first,windows={})
    for key,start,end in [('early_0_10s',0,10),('middle_10_30s',10,30),('late_30_40s',30,40),('all',0,40)]:
        selected=[s for s in scans.values() if start<=s['timestamp']-first<end]
        result['windows'][key]={k:summary(s[k] for s in selected) for k in ['total_ms','main_ms','pre_ms']}
    result['contexts']={str(k):dict(rows=len(rows),
        root_seconds=sum(r['owner_wall']['root'][1] for r in rows)/1e9,
        root_ms=summary(r['owner_wall']['root'][1]/1e6 for r in rows),
        gpu_linearize_calls=sum(r['owner_wall'].get('gpu_linearize',[0])[0] for r in rows),
        gpu_sync_calls=sum(r['owner_wall'].get('gpu_sync_linearize',[0])[0] for r in rows),
        gpu_cache_hits=sum(r['metrics'][7] for r in rows)) for k,rows in contexts.items()}
    result['complete_scan_total_seconds']=sum(s['total_ms'] for s in scans.values())/1000
    result['legacy_sidecar_callback_ms']=summary(callback.values())
    result['profile_sha256']=sha(source/'e1b_profile.jsonl')
    return result
def main():
    assert read(RUN/'validation_logs/complete.json')['passed']
    validation=read(ROOT/'profile40_on_validation.json')
    assert validation['passed'] and all(v['byte_identical'] for v in validation['repeat_files'].values())
    for p,h in read(ROOT/'protected_manifest.json').items():assert sha(Path(p))==h,p
    b=timing(OLD);c=timing(RUN)
    assert b['scans']==c['scans']
    provider=read(ROOT/'profile40_on_provider_validation.json')
    ratios={k:c['windows']['all']['total_ms'][k]/b['windows']['all']['total_ms'][k] for k in ['mean','median','p95','max']}
    result=dict(passed=True,baseline=b,candidate=c,complete_scan_candidate_over_baseline=ratios,
        complete_scan_p95_under_800ms=c['windows']['all']['total_ms']['p95']<800,
        controller=read(ROOT/'profile40_on_control/complete.json'),
        query_ms=summary(r['query_ms'] for r in provider['rows']),
        query_total_s=sum(r['query_ms'] for r in provider['rows'])/1000,
        provider={k:v for k,v in provider.items() if k!='rows'},
        source_coverage=read(ROOT/'profile40_on_source/source_coverage.json'),
        receiver={k:validation[k] for k in ['versions','active_source_ids','replacements','unsupported',
          'max_publication_gradient','live_output_age_s','final_transport_status','repeat_files']},
        caveats=['Matched settings but separate coupled live captures, not identical factor values/interleaving.',
          'Native Gaussian history collection/commit is included in MAIN update; not separately timed in this capture.',
          'Ordinary query timing includes repeated small extraction helper for existing diagnostics.',
          'First40s only; no long-run or real-time certification. No source-covariance accuracy claim.',
          'Later diagnostic revision covariance remains MAIN, unchanged from B0. No production promotion.'])
    (HERE/'summary.json').write_text(json.dumps(result,indent=2))
    evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
    def copy(src,dest):
        dst=evidence/dest;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
    for name in ['profile40_on_validation.json','profile40_on_native_validation.json','profile40_on_lifecycle_validation.json',
        'profile40_on_diagnostics_validation.json','profile40_on_provider_validation.json','profile40_on.json','protected_manifest.json',
        'profile40_on_source/effective_mode_check.json','profile40_on_source/source_coverage.json',
        'profile40_on_control/complete.json','profile40_on_receiver/transport_status.json','preflight/validation.json']:
        copy(ROOT/name,'capture/'+name)
    for name in ['ready.json','result.json','map_guard.log','command.txt','params.json']:copy(RUN/'no_data_startup'/name,'startup/'+name)
    for label,path in [('release',BUILD),('asan_pie',BUILD.parent/'asan_1'),('asan',BUILD.parent/'asan_nopie')]:
        for name in ['build_result.json','commands.json','command.json','tests/result.json','tests/tests.xml','tests/tests.log']:
            if (path/name).exists():copy(path/name,label+'/'+name)
    for name in ['result.json','regression.xml']:copy(BUILD/'nonregression'/name,'nonregression/'+name)
    for name in ['experiment.py','run_case.py','source.env','source_runner.sh','verify_effective.py','map_guard.py','glim_profile.launch','receiver_e2.sh']:
        copy(ROOT/name,'harness/'+name)
    for p in (BUILD/'source').iterdir():
        if p.is_file() and p.suffix in ['.h','.cpp']:copy(p,'private_source/'+p.name)
    for p in (BUILD/'overlay').iterdir():
        if p.is_file():copy(p,'private_source/overlay/'+p.name)
    for p in (RUN/'validation_logs').iterdir():
        if p.is_file():copy(p,'validation/'+p.name)
    (evidence/'hashes.json').write_text(json.dumps({str(p.relative_to(evidence)):sha(p) for p in sorted(evidence.rglob('*')) if p.is_file()},indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ['baseline','candidate','receiver','source_coverage']},indent=2))
if __name__=='__main__':main()
