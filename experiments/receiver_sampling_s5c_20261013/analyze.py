"""Disjoint work classification plus explicitly overlapping caller contexts."""
from pathlib import Path
import bisect
import collections
import csv
import json
import math
import statistics
import sys
from symbolize import records, ROOT

ORIGIN = 1727775008003475000

def work(names):
    """Exactly one category per sample, most-specific inner work first."""
    has = lambda token: any(token in name for name in names)
    numerical = has('numericalDerivative')
    motion = has('GPWNOAPrior')
    source = has('WnoaRelativeFactor') or has('GPWNOAInterpolator')
    if numerical:
        return ('motion numerical derivatives' if motion else
                'source interpolation numerical derivatives' if source else
                'other numerical derivatives')
    if has('EliminateQR') or has('inplace_QR'):
        return 'Gaussian QR elimination/assembly'
    if has('GPWNOAPrior') and has('linearize'):
        return 'motion linearization remainder'
    if has('WnoaRelativeFactor') and has('linearize'):
        return 'source linearization remainder'
    if has('ISAM2::ISAM2(gtsam::ISAM2 const&)') or has('BayesTree<gtsam::ISAM2Clique>::BayesTree'):
        return 'solver copy'
    if has('NormalizedGradient') or has('publication_cache::Evaluation'):
        return 'stationarity/cached-evaluation remainder'
    if has('GaussianCostChange'):
        return 'stable-cost check remainder'
    if has('NonlinearFactorGraph::error') or has('NoiseModelFactor::error'):
        return 'nonlinear objective evaluation'
    if has('calculateEstimate') or has('calculateBestEstimate') or has('updateDelta') or has('BackSubstitute'):
        return 'estimate/delta/back-substitution'
    if has('linearize'):
        return 'other factor linearization'
    if has('ISAM2::update'):
        return 'solver update remainder/order/tree/bookkeeping'
    if has('ISAM2::~ISAM2') or has('BayesTree<gtsam::ISAM2Clique>::~BayesTree'):
        return 'solver destruction'
    if has('EventStream::Next') or has('ParseBelief'):
        return 'journal parsing'
    return 'other/unattributed work'

def contexts(names):
    has = lambda token: any(token in name for name in names)
    definitions = {
        'publication wrapper':'numerical_audit::Refine(',
        'source revision handler':'{lambda((anonymous namespace)::ScheduledBelief const*, long)#9}',
        'normalized-gradient certificate':'NormalizedGradient',
        'iSAM2 update':'ISAM2::update',
        'iSAM2 batch recalculation':'ISAM2::recalculateBatch',
        'iSAM2 incremental recalculation':'ISAM2::recalculateIncremental',
        'WNOA motion factor linearization':'MemoFactor<fgo::factor::GPWNOAPrior',
        'source factor linearization':'MemoFactor<fgo::offline::WnoaRelativeFactor',
        'QR elimination':'EliminateQR',
        'stable-cost check':'GaussianCostChange',
        'publication cached evaluation':'publication_cache::Evaluation',
        'full-history rebuild':'RebuildFullHistoryQR',
        'numerical differentiation':'numericalDerivative',
    }
    return [key for key,token in definitions.items() if has(token)]

def kernel(names):
    """Includes inlined numerical templates missed by named-stack lower bounds."""
    has=lambda token:any(token in name for name in names)
    if has('linearize') and has('GPWNOAPrior::evaluateError'):
        return 'motion residual/Jacobian kernel'
    if has('linearize') and has('WnoaRelativeFactor::unwhitenedError'):
        return 'source residual/Jacobian kernel'
    return work(names)

def stats(values):
    order=sorted(values)
    def percentile(p):
        pos=(len(order)-1)*p;lo=math.floor(pos);hi=math.ceil(pos)
        return order[lo]*(hi-pos)+order[hi]*(pos-lo) if lo!=hi else order[lo]
    return dict(count=len(order),mean=statistics.mean(order),median=statistics.median(order),
                p95=percentile(.95),maximum=max(order)) if order else {}

def main():
    tag=sys.argv[1];censored=len(sys.argv)>2 and sys.argv[2]=='censored'
    suffix='_censored' if censored else ''
    output=ROOT/(tag+suffix+'_attribution.json');assert not output.exists()
    quality=json.loads((ROOT/(tag+'_sampling_validation.json')).read_text())
    if censored:
        assert not quality['passed'] and quality['truncated']>0
        assert not any(quality[k] for k in ['write_failures','overruns','unwind_errors','empty_stacks'])
        assert quality['handler_unchanged'] and quality['main_tid_only']
        assert .90<=quality['coverage_ratio']<=1.05
    else:assert quality['passed']
    symbols=json.loads((ROOT/(tag+suffix+'_symbols.json')).read_text())
    rows=records(ROOT/(tag+'_profile'))
    cap=json.loads((ROOT/(tag+'_profile')/'start.json').read_text())['maximum_frames']
    # The progress record is emitted after an input event, before support
    # creation/publication. The last recorded physical arrival labels work
    # until the next event. This is a cohort label, not an internal timer.
    with (ROOT/tag/'stream_progress.csv').open() as f:progress=list(csv.DictReader(f))
    wall=[int(r['wall_unix_ns']) for r in progress]
    assert wall==sorted(wall)
    groups={name:[] for name in ['all','20-40s','60-80s','120-140s','140s-end']}
    depth=[];unknown_leaf=0;unmapped_leaf=0;empty=0
    exclusive=collections.Counter();inclusive=collections.Counter();phase=collections.Counter()
    frame_counts=collections.Counter()
    for row in rows:
        frame=[symbols[str(pc-(i>0))] for i,pc in enumerate(row[8:8+row[4]]) if pc]
        # Recursive Bayes-tree frames can repeat hundreds of times. Inclusion
        # depends on membership, not multiplicity; keep the leaf first.
        names=list(dict.fromkeys(f['function'] for f in frame))
        unknown_leaf+=names[0]=='??';unmapped_leaf+=frame[0]['elf']=='[unmapped]'
        exclusive[names[0]]+=1;inclusive.update(set(names));depth.append(row[4])
        frame_counts.update(f['elf'] for f in frame[:1])
        truncated=censored and row[4]>=cap
        cs=[] if truncated else contexts(names)
        w='unattributed truncated stack' if truncated else work(names)
        ph=('publication' if 'publication wrapper' in cs else
            'source revision' if 'source revision handler' in cs else 'support/startup/final/other')
        phase[ph]+=1
        index=bisect.bisect_right(wall,row[1])-1
        simulated=(int(progress[index]['arrival_ns'])-ORIGIN)/1e9 if index>=0 else 0
        entry=dict(work=w,kernel=w if truncated else kernel(names),contexts=cs,phase=ph,simulated=simulated)
        groups['all'].append(entry)
        for label,lo,hi in [('20-40s',20,40),('60-80s',60,80),('120-140s',120,140),('140s-end',140,1e9)]:
            if lo<=simulated<hi:groups[label].append(entry)
    result=dict(samples=len(rows),stack_depth=stats(depth),unknown_leaf=unknown_leaf,
        censored_secondary_only=censored,strict_sampling_gate_passed=quality['passed'],
        unmapped_leaf=unmapped_leaf,leaf_elf_counts=dict(frame_counts),phase_counts=dict(phase),
        exclusive_functions=exclusive.most_common(70),inclusive_functions=inclusive.most_common(120),cohorts={})
    for label,entries in groups.items():
        wc=collections.Counter(e['work'] for e in entries)
        cc=collections.Counter(c for e in entries for c in e['contexts'])
        pc=collections.Counter(e['phase'] for e in entries)
        joint=collections.Counter((e['phase'],e['work']) for e in entries)
        kc=collections.Counter(e['kernel'] for e in entries)
        kj=collections.Counter((e['phase'],e['kernel']) for e in entries)
        result['cohorts'][label]=dict(samples=len(entries),work_counts=dict(wc),
            kernel_counts=dict(kc),phase_kernel_counts={' / '.join(k):v for k,v in kj.items()},
            inclusive_context_counts=dict(cc),phase_counts=dict(pc),
            phase_work_counts={' / '.join(k):v for k,v in joint.items()})
    with output.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({key:value for key,value in result.items() if not key.endswith('_functions')},indent=2))

if __name__=='__main__':main()
