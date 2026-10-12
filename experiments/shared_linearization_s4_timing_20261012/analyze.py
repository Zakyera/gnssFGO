"""Summarize the frozen timing passes and preserve small reproducibility evidence."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import statistics
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s4_timing_20261012'
SOURCE=WS/'runs/gnssfgo_shared_linearization_s3_20261012/prepared/profile40_on_source'
def stats(xs):
    xs=sorted(xs);n=len(xs);assert n
    def quant(p):
        v=(n-1)*p;i=int(v);return xs[i]+(xs[min(i+1,n-1)]-xs[i])*(v-i)
    return dict(n=n,mean=statistics.mean(xs),median=statistics.median(xs),p95=quant(.95),max=xs[-1],total=sum(xs))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    assert not (HERE/'summary.json').exists()
    result={'passed':True,'variants':{}}
    for mode in ['release','asan_nopie']:
        assert json.loads((ROOT/(mode+'_results/complete.json')).read_text())['passed']
        unit=ET.parse(ROOT/(mode+'_results/unit.xml')).getroot()
        result[mode+'_tests']={k:int(unit.attrib[k]) for k in ['tests','failures','errors','disabled']}
        assert result[mode+'_tests']['failures']==result[mode+'_tests']['errors']==0
    for variant in ['original','instrumented']:
        by_pass={}
        for rep in range(1,4):
            rows=[json.loads(x) for x in (ROOT/f'release_results/measured{rep}_{variant}.jsonl').read_text().splitlines()]
            by_pass[rep]=rows
        rows=sum(by_pass.values(),[])
        summary={}
        for kind in ['stage','commit','query']:
            selected=[r for r in rows if r['kind']==kind]
            keys=[k for k in selected[0] if k.endswith('_s')]
            summary[kind]={k:stats([r[k]*1000 for r in selected]) for k in keys}
        summary['per_pass']={rep:{kind:sum(r[kind+'_total_s'] for r in rs if r['kind']==kind) for kind in ['stage','commit','query']} for rep,rs in by_pass.items()}
        summary['errors']={key:max(r[key] for r in rows if r['kind']=='summary') for key in ['max_information_relative_error','max_covariance_relative_error']}
        # Epoch quarters are not claimed to be timestamp bins.
        summary['epoch_bins']={}
        for label,lo,hi in [('early',1,195),('middle',196,587),('late',588,783)]:
            summary['epoch_bins'][label]={kind:stats([r[kind+'_total_s']*1000 for r in rows if r['kind']==kind and lo<=r['epoch']<=hi]) for kind in ['stage','query']}
        summary['retirement_only_ms']=stats([r['stage_total_s']*1000 for r in rows if r['kind']=='stage' and r['retiring']])
        if variant=='instrumented':
            summary['retirement_elimination_only_ms']=stats([r['stage.retirement_elimination_s']*1000 for r in rows if r['kind']=='stage' and r['retiring']])
            q=[r for r in rows if r['kind']=='query']
            summary['rank_fraction']=sum(r['query.dense_rank_check_s'] for r in q)/sum(r['query_total_s'] for r in q)
            summary['non_rank_query_ms']=stats([(r['query_total_s']-r['query.dense_rank_check_s'])*1000 for r in q])
            summary['timer_unaccounted_ms']=stats([(r['query_total_s']-sum(v for k,v in r.items() if k.startswith('query.')))*1000 for r in q])
        result['variants'][variant]=summary
    log=next((SOURCE/'ros_logs').glob('*/glim_rosnode-1-stdout.log'))
    request=[];revisions=0
    for line in log.open():
        m=re.search(r'CBS_ODOM_MARGINAL_PAIR_COUNT_ROW,G2K,g,k,(\d+),(\d+),(\d+)',line)
        if m:request.append(tuple(map(int,m.groups())))
        if 'GLIM_CBS_REVISION_BELIEF_ROW,' in line:revisions+=1
    assert request and all(x[0]%2==0 for x in request)
    result['cadence']=dict(log=str(log),log_sha256=sha(log),calls=len(request),requested_pairs=sum(x[0]//2 for x in request),successful_marginals=sum(x[1] for x in request),outgoing_beliefs=sum(x[2] for x in request),revision_rows=revisions,
        note='Revision diagnostics use MAIN jointMarginalInformation separately; not extra LOCAL provider calls. Only 20 exact native query checkpoints retained.')
    result['cadence']['projected_query_seconds']=sum(x[1] for x in request)*result['variants']['original']['query']['query_total_s']['mean']/1000
    result['cadence']['projection_limit']='Sampled-query mean times observed call count only; not an unsampled all-query or end-to-end benchmark.'
    base=json.loads((ROOT/'release_results/complete.json').read_text())
    result['identities']=base
    result['instrumentation_total_ratio']=sum(sum(v.values()) for v in result['variants']['instrumented']['per_pass'].values())/sum(sum(v.values()) for v in result['variants']['original']['per_pass'].values())
    (HERE/'summary.json').write_text(json.dumps(result,indent=2))
    evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
    for mode in ['release','asan','asan_nopie']:
        for sub in [mode,mode+'_results']:
            dest=evidence/sub;dest.mkdir()
            for p in (ROOT/sub).iterdir():
                if p.suffix in ['.json','.jsonl','.stderr','.xml','.log']:
                    shutil.copy2(p,dest/p.name)
    shutil.copy2(ROOT/'release/instrumented.cpp',evidence/'instrumented.cpp')
    # Independently checked artifacts stay unchanged; retain their references.
    paths=[HERE/'PROTOCOL.md',ROOT/'driver/NativeGaussianHistory.cpp',ROOT/'driver/NativeGaussianHistory.h',ROOT/'driver/test_native_gaussian.cpp',SOURCE/'native_gaussian.bin']
    paths+=list((ROOT/'release').glob('original'))+list((ROOT/'release').glob('instrumented'))
    (HERE/'hashes.json').write_text(json.dumps({str(p):sha(p) for p in paths},indent=2))
    print(json.dumps({'rank_fraction':result['variants']['instrumented']['rank_fraction'],'cadence':result['cadence'],'instrumentation_total_ratio':result['instrumentation_total_ratio']},indent=2))
if __name__=='__main__':main()
