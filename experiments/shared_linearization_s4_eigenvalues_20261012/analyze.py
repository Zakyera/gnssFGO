"""Archive small evidence and summarize the predeclared matched replay."""
from pathlib import Path
import hashlib
import json
import shutil
import statistics
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s4_eigenvalues_20261012'
BASE=WS/'runs/shared_linearization_s4_timing_20261012'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def stats(xs):
    xs=sorted(xs);n=len(xs);assert n
    z=(n-1)*.95;i=int(z)
    return dict(n=n,mean=statistics.mean(xs),median=statistics.median(xs),
                p95=xs[i]+(xs[min(i+1,n-1)]-xs[i])*(z-i),max=xs[-1],total=sum(xs))

def rows(p):return [json.loads(s) for s in p.read_text().splitlines()]

def main():
    assert not (HERE/'summary.json').exists()
    result={'passed':True,'tests':{},'variants':{},'profile':{},'identities':{}}
    for phase in ['release','asan','timing']:
        result['identities'][phase]=json.loads((ROOT/(phase+'_results/complete.json')).read_text())
        assert result['identities'][phase]['passed']
    for mode in ['release','asan']:
        test=ET.parse(ROOT/(mode+'_results/unit.xml')).getroot()
        summary={k:int(test.attrib[k]) for k in ['tests','failures','errors','disabled']}
        assert summary['tests']==31 and summary['failures']==summary['errors']==summary['disabled']==0
        result['tests'][mode]=summary
    for variant in ['original','candidate']:
        by_pass={i:rows(ROOT/f'timing_results/measured{i}_{variant}.jsonl') for i in range(1,4)}
        all_rows=sum(by_pass.values(),[])
        summary={kind:stats([r[kind+'_total_s']*1000 for r in all_rows if r['kind']==kind]) for kind in ['stage','commit','query']}
        summary['per_pass_seconds']={i:{kind:sum(r[kind+'_total_s'] for r in rs if r['kind']==kind) for kind in ['stage','commit','query']} for i,rs in by_pass.items()}
        summary['per_pass_wall_seconds']={i:rs[-1]['wall_s'] for i,rs in by_pass.items()}
        summary['epoch_bins']={label:stats([r['query_total_s']*1000 for r in all_rows if r['kind']=='query' and lo<=r['epoch']<=hi])
                              for label,lo,hi in [('early',1,195),('middle',196,587),('late',588,783)]}
        summary['max_errors']={k:max(r[k] for r in all_rows if r['kind']=='summary') for k in ['max_information_relative_error','max_covariance_relative_error']}
        summary['conservation']={k:by_pass[1][-1][k] for k in ['epoch','queries','retirements','admitted','absorbed','removed','active']}
        for rs in by_pass.values():
            assert all(rs[-1][k]==v for k,v in summary['conservation'].items())
        result['variants'][variant]=summary
        q=[r for r in rows(ROOT/f'timing_results/profile_{variant}.jsonl') if r['kind']=='query']
        result['profile'][variant]={k:stats([r[k]*1000 for r in q]) for k in q[0] if k.endswith('_s')}
    ref=result['variants']['original'];cand=result['variants']['candidate']
    result['query_speedup_median']=ref['query']['median']/cand['query']['median']
    result['query_reduction_fraction']=1-cand['query']['mean']/ref['query']['mean']
    result['paired_query_speedup']={i:ref['per_pass_seconds'][i]['query']/cand['per_pass_seconds'][i]['query'] for i in range(1,4)}
    result['repeatable_query_improvement']=all(x>1 for x in result['paired_query_speedup'].values())
    result['projected_389_queries_seconds']={v:389*result['variants'][v]['query']['mean']/1000 for v in ['original','candidate']}
    result['projection_limit']='20 saved checkpoints times389 observed direct LOCAL requests; not measured at every request, not live or end-to-end.'
    # Post-run integrity, including actual generated source and unchanged timers.
    original=(REPO/'experiments/shared_linearization_s2_20261012/NativeGaussianHistory.cpp').read_text()
    candidate=(ROOT/'driver/candidate.cpp').read_text()
    old='Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h);'
    new='Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h, Eigen::EigenvaluesOnly);'
    assert original.count(old)==1 and candidate==original.replace(old,new)
    timed=(ROOT/'release/instrumented.cpp').read_text()
    stripped=''.join(l for l in timed.splitlines(True) if l!='#include "Timing.h"\n' and 'diagnostic_timing::Span timing(' not in l and 'timing.mark(' not in l)
    assert stripped==candidate
    protected=json.loads((REPO/'experiments/shared_linearization_s3_capture_20261012/evidence/capture/protected_manifest.json').read_text())
    for p,digest in protected.items():assert sha(p)==digest,p
    s4pins=json.loads((REPO/'experiments/shared_linearization_s4_timing_20261012/hashes.json').read_text())
    for p,digest in s4pins.items():assert sha(p)==digest,p
    pins=json.loads((ROOT/'input_hashes.json').read_text())
    for p,digest in pins.items():assert sha(p)==digest,p
    integrity=json.loads((ROOT/'baseline_integrity_after.json').read_text());assert not integrity['failures']
    result['integrity']=dict(protected_s3_files=len(protected),s4a_identity_pins=len(s4pins),
                             new_input_pins=len(pins),baseline=integrity,one_line_only=True)
    evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
    for sub in ['release','asan','release_results','asan_results','timing_results']:
        dest=evidence/sub;dest.mkdir()
        for p in (ROOT/sub).iterdir():
            if p.suffix in ['.json','.jsonl','.xml','.stderr','.log']:shutil.copy2(p,dest/p.name)
    for name in ['candidate.cpp','rank_access.cpp']:
        shutil.copy2(ROOT/'driver'/name,evidence/name)
    for name in ['input_hashes.json','baseline_integrity.json','baseline_integrity_after.json','timing_environment.json']:
        shutil.copy2(ROOT/name,evidence/name)
    paths=[HERE/'PROTOCOL.md',ROOT/'driver/candidate.cpp',ROOT/'release/candidate',ROOT/'release/instrumented',
           ROOT/'asan/candidate',ROOT/'asan/tests',BASE/'release/original',BASE/'asan_nopie/original']
    (HERE/'hashes.json').write_text(json.dumps({str(p):sha(p) for p in paths},indent=2))
    (HERE/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:result[k] for k in ['tests','query_speedup_median','query_reduction_fraction','paired_query_speedup','repeatable_query_improvement','projected_389_queries_seconds']},indent=2))

if __name__=='__main__':main()
