"""Read retained S5 logs only. Never execute/import an estimator or change input."""
import bisect
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared')
LIVE = ROOT / 'capacity160_on_receiver'
REC = LIVE / 'receiver'
BASE = 1727775008003475000
BUILD = Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate')
FILES = [REC / n for n in ['publication_convergence.csv', 'publication_steps.csv', 'e2_memo.csv',
    'stream_progress.csv', 'revision_events.csv', 'causal_dispatch.csv', 'summary.json']]
FILES += [LIVE / 'receipts.jsonl', LIVE / 'events.csv', LIVE / 'resolved_config.json',
          ROOT / 'capacity160_on_control/monitor.jsonl', ROOT / 'capacity160_on_control/failed.txt']
FILES += [BUILD / n for n in ['replay', 'DiagnosticReplay.cpp', 'SourceWnoaModel.h', 'ExactLinearizationMemo.h']]
FILES += [Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/build/FullPublicationRefinement.h')]
FILES += [REPO / 'online_fgo/include' / n for n in ['offline_process/BoundedISAM2Refinement.h',
    'offline_process/TransactionalISAM2Rebuild.h', 'factor/motion/GPWNOAPrior.h',
    'model/gp_interpolator/GPWNOAInterpolator.h']]

def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()

def rows(p):
    with p.open() as f: return list(csv.DictReader(f))

def stats(items):
    x = sorted(items)
    if not x: return {'n': 0}
    def percentile(q):
        u = (len(x)-1)*q; lo = int(u); hi = math.ceil(u)
        return x[lo] + (x[hi]-x[lo])*(u-lo)
    return dict(n=len(x), mean=statistics.mean(x), median=statistics.median(x),
                p95=percentile(.95), maximum=x[-1], minimum=x[0], total=sum(x))

def sec(ns): return (int(ns)-BASE)/1e9

def main():
    assert not (HERE / 'summary.json').exists(), 'Refuse to overwrite audit output'
    before = {str(p): sha(p) for p in FILES}
    assert before[str(BUILD / 'replay')] == '9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
    assert (ROOT / 'capacity160_on_control/failed.txt').read_text().strip() == 'live queue gate exceeded'
    conv = rows(REC / 'publication_convergence.csv')
    steps = rows(REC / 'publication_steps.csv')
    by_step = defaultdict(list)
    for r in steps: by_step[(r['phase'], int(r['index']))].append(r)
    memos = {}; previous = {0: Counter(), 1: Counter()}
    count_keys = ['calls', 'hits', 'misses', 'invalidations', 'failures']
    for r in rows(REC / 'e2_memo.csv'):
        kind = int(r['kind']); delta = {k: int(r[k])-previous[kind][k] for k in count_keys}
        assert all(x >= 0 for x in delta.values())
        assert delta['calls'] == delta['hits'] + delta['misses']
        previous[kind] = Counter({k: int(r[k]) for k in count_keys})
        memos[(r['phase'], int(r['index']), kind)] = dict(
            delta=delta, active=int(r['live_entries']), bytes=int(r['live_matrix_signature_bytes']))
    support = []
    for r in conv:
        if r['phase'] != 'support': continue
        i = int(r['index']); ss = by_step[(r['phase'], i)]
        normal = [s for s in ss if s['recovery'] == '0']
        recover = [s for s in ss if s['recovery'] == '1']
        assert all(int(s['relinearized']) == 3*(i+1) for s in normal)
        assert len(normal) == int(r['attempted'])
        m = {k: memos[('support', i, k)] for k in [0,1]}
        assert m[0]['active'] == i
        support.append(dict(index=i, sec=sec(r['physical_ns']), milliseconds=float(r['seconds'])*1000,
            attempts=int(r['attempted']), accepted=int(r['accepted']), recovery=bool(recover),
            recovery_steps=len(recover), consensus_steps=sum(s['consensus']=='1' for s in ss),
            local_prediction_steps=sum(s['local_prediction']=='1' for s in ss),
            rejected_steps=sum(s['accepted']=='0' for s in ss),
            before_gradient=float(r['gradient_before']), after_gradient=float(r['gradient_after']),
            keys=3*(i+1), factors=3+m[0]['active']+m[1]['active'], memo=m,
            all_ordinary_accepted=all(s['accepted']=='1' for s in normal),
            steps=ss))
    revisions = rows(REC / 'revision_events.csv'); dispatch = rows(REC / 'causal_dispatch.csv')
    assert len(revisions) == len(dispatch)
    revision_data = []
    for r,d in zip(revisions,dispatch):
        for k in ['from_index','to_index','revision','outcome']: assert r[k] == d[k]
        assert r['record_timestamp_ns'] == d['arrival_ns']
        revision_data.append(dict(sec=sec(d['insertion_ns']), source=d['source'],
            ms=float(r['update_seconds'])*1000, created_support=int(d['created_support']),
            outcome=r['outcome'], current_correction=float(r['current_position_correction_m'])))
    receipts = [json.loads(l) for l in (LIVE/'receipts.jsonl').open()]
    with (LIVE/'events.csv').open() as f:
        for i,line in enumerate(f):
            columns=line.strip().split(','); rr=receipts[i]
            assert rr['event']==i and rr['kind']==columns[0] and rr['receipt_ros_ns']==int(columns[1])
    assert i+1==len(receipts)
    progress = rows(REC/'stream_progress.csv'); delay=[]; gaps=[]
    last = None
    for p in progress:
        rr = receipts[int(p['event_count'])-1]
        assert rr['receipt_ros_ns'] == int(p['arrival_ns'])
        wall=int(p['wall_unix_ns']); elapsed=(wall-rr['wall_ns'])/1e6
        assert elapsed>=0
        delay.append(dict(sec=sec(p['arrival_ns']), ms=elapsed, kind=rr['kind'], wall=wall))
        if last and int(p['created_support'])>int(last['created_support']):
            assert int(p['created_support'])==int(last['created_support'])+1
            # Bounds around the support loop: includes its work, possible idle
            # wait, and the next event; not a timer for a specific function.
            gaps.append(dict(index=int(p['created_support']), sec=sec(p['arrival_ns']),
                             ms=(wall-int(last['wall_unix_ns']))/1e6))
        last=p
    monitor=[json.loads(l) for l in (ROOT/'capacity160_on_control/monitor.jsonl').open()]
    end=max(r['sec'] for r in support)+1e-6
    bins=[(0,40),(40,80),(80,100),(100,120),(120,130),(130,140),(140,end),(0,end)]
    blocks={}
    for lo,hi in bins:
        c=[r for r in support if lo<=r['sec']<hi]
        rev=[r for r in revision_data if lo<=r['sec']<hi]
        receipt=[r for r in receipts if lo<=sec(r['receipt_ros_ns'])<hi and r['kind']=='B']
        d=[r for r in delay if lo<=r['sec']<hi]
        mon=[r for r in monitor if lo<=sec(r['last_imu_ns'])<hi and r['published_poses']>0]
        no_recovery=[r for r in c if not r['recovery']]
        recovery=[r for r in c if r['recovery']]
        a=dict(range_s=[lo,hi], publications=len(c), refinement_ms=stats(r['milliseconds'] for r in c),
            attempts=stats(r['attempts'] for r in c), attempt_histogram=dict(Counter(r['attempts'] for r in c)),
            keys=stats(r['keys'] for r in c), factors=stats(r['factors'] for r in c),
            no_recovery_ms=stats(r['milliseconds'] for r in no_recovery),
            with_recovery_ms=stats(r['milliseconds'] for r in recovery),
            recovery_publications=len(recovery), recovery_steps=sum(r['recovery_steps'] for r in c),
            local_prediction_steps=sum(r['local_prediction_steps'] for r in c),
            before_gradient=stats(r['before_gradient'] for r in c),
            after_gradient=stats(r['after_gradient'] for r in c),
            ordinary_ms_per_attempt=stats(r['milliseconds']/r['attempts'] for r in no_recovery if r['attempts']),
            by_attempt={n:stats(r['milliseconds'] for r in no_recovery if r['attempts']==n) for n in sorted({r['attempts'] for r in no_recovery})},
            cache={}, revisions={},
            source_receipt_events=len(receipt),
            revision_update_ms=stats(r['ms'] for r in rev),
            queue_and_processing_sojourn_ms=stats(r['ms'] for r in d),
            observed_queue=stats(r['queued'] for r in mon),
            support_transition_gap_ms=stats(r['ms'] for r in gaps if lo<=r['sec']<hi))
        for k,name in [(0,'wnoa'),(1,'source_relative')]:
            sums={f:sum(r['memo'][k]['delta'][f] for r in c) for f in count_keys}
            sums['hit_fraction']=sums['hits']/sums['calls'] if sums['calls'] else None
            sums['calls_per_publication']=stats(r['memo'][k]['delta']['calls'] for r in c)
            sums['misses_per_publication']=stats(r['memo'][k]['delta']['misses'] for r in c)
            a['cache'][name]=sums
        for source in ['glim_hesai','kimera_front']:
            rr=[r for r in rev if r['source']==source]
            a['revisions'][source]=dict(n=len(rr), update_ms=stats(r['ms'] for r in rr),
                outcomes=dict(Counter(r['outcome'] for r in rr)))
        blocks[f'{lo}-{hi:.6f}']=a
    thresholds={}
    for threshold in [1,100,500,1000]:
        chosen=next((r for r in monitor if r['queued']>=threshold or r['max_queue']>=threshold),None)
        if chosen: thresholds[threshold]=dict(sec=sec(chosen['last_imu_ns']),
            queue=chosen['queued'], cumulative_max=chosen['max_queue'], output_age_s=chosen['output_age_seconds'])
    worst=sorted(support,key=lambda r:r['milliseconds'],reverse=True)[:10]
    trace=[r for r in support if 138<=r['sec']<end]
    # Compact trace preserves every recorded step, but avoids huge memo deltas.
    for r in trace: r.pop('memo',None)
    for p,h in before.items(): assert sha(Path(p))==h,p
    result=dict(task='S5a',scope='retained-log and frozen-code audit only',input_sha256=before,
        unchanged_after=True,all_nonrecovery_steps_relinearize_all_keys=True,
        all_recorded_publications_stationary=all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in conv),
        blocks=blocks, first_queue_threshold_observations=thresholds,
        worst_publications=[{k:v for k,v in r.items() if k!='memo'} for r in worst],
        late_trace=trace,
        limits=['Publication timer includes Refine and cache-counter flush, not total support/event cost.',
          'Cache deltas include intervening updates as well as publication, and are counts not timing.',
          'Revision update timer covers ReplaceFactor only, not construction/validation/bookkeeping.',
          'Progress-minus-receipt time includes queueing plus processing; not CPU time.',
          'No internal timing split for differentiation, QR, solver copy, gradient accumulation or IO.',
          'Counts/correlations cannot certify a speedup or the precise cause of extra iterations.'])
    with (HERE/'summary.json').open('x') as f: json.dump(result,f,indent=2,allow_nan=False)
    print('INPUTS_UNCHANGED',len(before),'publications',len(support),'revision rows',len(revisions))
    for name,r in blocks.items():
        print(name,'ms median/p95',r['refinement_ms']['median'],r['refinement_ms']['p95'],
              'attempts',r['attempt_histogram'],'recovery',r['recovery_publications'],
              'revision sum ms',r['revision_update_ms']['total'])

if __name__=='__main__':main()
