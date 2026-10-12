"""Summarize the fixed S4b/S4c experiment, including an unsuccessful speed gate."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import shutil
import statistics
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s4_certificate_20261012'
BASE=WS/'runs/shared_linearization_s4_eigenvalues_20261012'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()

def stats(xs):
    xs=sorted(xs);n=len(xs);assert n
    z=(n-1)*.95;i=int(z)
    return dict(n=n,mean=statistics.mean(xs),median=statistics.median(xs),
                p95=xs[i]+(xs[min(i+1,n-1)]-xs[i])*(z-i),max=xs[-1],total=sum(xs))

def rows(p): return [json.loads(s) for s in p.read_text().splitlines()]

def certificates(q):
    assert q and all(r['rank.certified']+r['rank.fallback']==1 for r in q)
    result=dict(queries=len(q),certified=sum(int(r['rank.certified']) for r in q),
                fallback=sum(int(r['rank.fallback']) for r in q),
                reasons=dict(Counter(int(r['rank.reason']) for r in q)))
    result['fallback_fraction']=result['fallback']/len(q)
    result['certificate_ms']=stats([r['rank.certificate_s']*1000 for r in q])
    result['fallback_ms']=stats([r.get('rank.fallback_s',0)*1000 for r in q])
    result['combined_check_ms']=stats([(r['rank.certificate_s']+r.get('rank.fallback_s',0))*1000 for r in q])
    result['dimensions']=sorted(set(int(r['rank.dimension']) for r in q))
    return result

def main():
    assert not (HERE/'summary.json').exists()
    result={'correctness_passed':True,'tests':{},'variants':{},'profile':{},'identities':{}}
    for phase in ['release','asan','timing']:
        result['identities'][phase]=json.loads((ROOT/(phase+'_results/complete.json')).read_text())
        assert result['identities'][phase]['passed']
    for mode in ['release','asan']:
        test=ET.parse(ROOT/(mode+'_results/unit.xml')).getroot()
        summary={k:int(test.attrib[k]) for k in ['tests','failures','errors','disabled']}
        assert summary['tests']==40 and summary['failures']==summary['errors']==summary['disabled']==0
        result['tests'][mode]=summary
    for variant in ['reference','candidate']:
        by_pass={i:rows(ROOT/f'timing_results/measured{i}_{variant}.jsonl') for i in range(1,4)}
        all_rows=sum(by_pass.values(),[])
        summary={kind:stats([r[kind+'_total_s']*1000 for r in all_rows if r['kind']==kind]) for kind in ['stage','commit','query']}
        summary['per_pass_seconds']={i:{kind:sum(r[kind+'_total_s'] for r in rs if r['kind']==kind) for kind in ['stage','commit','query']} for i,rs in by_pass.items()}
        summary['per_pass_wall_seconds']={i:rs[-1]['wall_s'] for i,rs in by_pass.items()}
        summary['epoch_bins']={label:stats([r['query_total_s']*1000 for r in all_rows if r['kind']=='query' and lo<=r['epoch']<=hi])
                              for label,lo,hi in [('early',1,195),('middle',196,587),('late',588,783)]}
        summary['max_errors']={k:max(r[k] for r in all_rows if r['kind']=='summary') for k in ['max_information_relative_error','max_covariance_relative_error']}
        summary['conservation']={k:by_pass[1][-1][k] for k in ['epoch','queries','retirements','admitted','absorbed','removed','active']}
        assert summary['conservation']==dict(epoch=783,queries=20,retirements=313,admitted=5321,absorbed=4513,removed=0,active=808)
        for rs in by_pass.values():
            assert all(rs[-1][k]==v for k,v in summary['conservation'].items())
        assert summary['max_errors']['max_information_relative_error']<1e-9
        assert summary['max_errors']['max_covariance_relative_error']<1e-8
        result['variants'][variant]=summary
        q=[r for r in rows(ROOT/f'timing_results/profile_{variant}.jsonl') if r['kind']=='query']
        fields=set().union(*(r.keys() for r in q))
        result['profile'][variant]={k:stats([r.get(k,0)*1000 for r in q]) for k in sorted(fields) if k.endswith('_s')}
        if variant=='candidate':
            result['certificates']=certificates([r for r in all_rows if r['kind']=='query'])
            result['certificate_unique_checkpoints']=certificates([r for r in by_pass[1] if r['kind']=='query'])
            result['profile_certificates']=certificates(q)
    ref=result['variants']['reference'];cand=result['variants']['candidate']
    result['query_speedup_median']=ref['query']['median']/cand['query']['median']
    result['query_median_reduction_fraction']=1-cand['query']['median']/ref['query']['median']
    result['query_mean_reduction_fraction']=1-cand['query']['mean']/ref['query']['mean']
    result['paired_query_speedup']={i:ref['per_pass_seconds'][i]['query']/cand['per_pass_seconds'][i]['query'] for i in range(1,4)}
    result['speed_gates']=dict(median_25_percent=result['query_median_reduction_fraction']>=.25,
                             every_pair_faster=all(x>1 for x in result['paired_query_speedup'].values()),
                             no_p95_regression=cand['query']['p95']<=ref['query']['p95'])
    result['speed_gate_passed']=all(result['speed_gates'].values())
    result['projected_389_queries_seconds']={v:389*result['variants'][v]['query']['mean']/1000 for v in ['reference','candidate']}
    result['projection_limit']='20 saved checkpoints times389 historical requests; not measured every request or in live GLIM.'
    # The unmodified numerical path is mechanically checked, not assumed.
    original=(BASE/'driver/candidate.cpp').read_text()
    candidate=(ROOT/'driver/candidate.cpp').read_text()
    prefix='#include "RankCertificate.h"\n#include "Timing.h"\n'
    assert candidate.startswith(prefix)
    stripped=candidate[len(prefix):]
    begin=stripped.index('\nvoid certifiedRankCheck(')
    end=stripped.index('bool sameChart(',begin)
    stripped=stripped[:begin]+stripped[end:]
    stripped=stripped.replace('  certifiedRankCheck(h);','  positiveDefinite(h);')
    assert stripped==original
    for name in ['NativeGaussianHistory.h','test_native_gaussian.cpp','rank_tests.cpp','replay.cpp','Timing.h']:
        assert sha(ROOT/'driver'/name)==sha(BASE/'driver'/name)
    protected=json.loads((REPO/'experiments/shared_linearization_s3_capture_20261012/evidence/capture/protected_manifest.json').read_text())
    for p,digest in protected.items(): assert sha(p)==digest,p
    historical_pins={}
    for name in ['shared_linearization_s4_timing_20261012','shared_linearization_s4_eigenvalues_20261012']:
        pins=json.loads((REPO/'experiments'/name/'hashes.json').read_text())
        for p,digest in pins.items(): assert sha(p)==digest,p
        historical_pins[name]=len(pins)
    pins=json.loads((ROOT/'input_hashes.json').read_text())
    for p,digest in pins.items(): assert sha(p)==digest,p
    integrity=json.loads((ROOT/'baseline_after.json').read_text());assert not integrity['failures']
    assert json.loads((ROOT/'baseline_before.json').read_text())==integrity
    result['integrity']=dict(protected_s3_files=len(protected),historical_pins=historical_pins,
                             new_input_pins=len(pins),baseline=integrity,
                             unchanged_covariance_path=True,unchanged_previous_tests=True)
    evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
    for sub in ['release','asan','release_results','asan_results','timing_results']:
        dest=evidence/sub;dest.mkdir()
        for p in (ROOT/sub).iterdir():
            if p.suffix in ['.json','.jsonl','.xml','.stderr','.log']: shutil.copy2(p,dest/p.name)
    for name in ['candidate.cpp','rank_access.cpp']:
        shutil.copy2(ROOT/'driver'/name,evidence/name)
    for name in ['input_hashes.json','baseline_before.json','baseline_after.json','timing_environment.json']:
        shutil.copy2(ROOT/name,evidence/name)
    paths=[HERE/'PROTOCOL.md',HERE/'RankCertificate.h',ROOT/'driver/candidate.cpp',
           ROOT/'release/candidate',ROOT/'release/tests',ROOT/'asan/candidate',ROOT/'asan/tests',
           BASE/'release/candidate',BASE/'release/instrumented',BASE/'asan/candidate']
    (HERE/'hashes.json').write_text(json.dumps({str(p):sha(p) for p in paths},indent=2))
    (HERE/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:result[k] for k in ['tests','query_speedup_median','query_median_reduction_fraction','paired_query_speedup','speed_gates','certificates','projected_389_queries_seconds']},indent=2))

if __name__=='__main__': main()
