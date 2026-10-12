"""Disjoint profile attribution only after exact-equivalence gates pass."""
import csv
from collections import defaultdict
import json
from pathlib import Path
import statistics

ROOT = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
BASE = 1727775008003475000


def rows(path):
    with path.open() as f: return list(csv.DictReader(f))


def stats(items):
    x = sorted(items)
    if not x: return {'n':0}
    p = (len(x)-1)*.95; lo=int(p); hi=min(lo+1,len(x)-1)
    return dict(n=len(x), mean=statistics.mean(x), median=statistics.median(x),
                p95=x[lo]+(x[hi]-x[lo])*(p-lo), maximum=x[-1], total=sum(x))


def main():
    assert not (ROOT/'analysis.json').exists()
    for gate in ['prefix','full_reference','full']:
        assert json.loads((ROOT/(gate+'_validation.json')).read_text())['passed'], gate
    profile = rows(ROOT/'full_on_profile.csv')
    assert all(int(r['foreign_scopes']) == 0 for r in profile)
    cumulative = {}; snapshots = defaultdict(dict); final = {}
    for r in profile:
        key=(r['context'],r['tag'])
        current={k:int(r[k]) for k in ['calls','inclusive_ns','exclusive_ns','cpu_ns']}
        previous=cumulative.get(key,dict.fromkeys(current,0))
        delta={k:current[k]-previous[k] for k in current}
        assert all(v>=0 for v in delta.values()), (key,delta)
        assert current['exclusive_ns']<=current['inclusive_ns']
        cumulative[key]=current
        if r['phase']=='support':
            snapshots[(int(r['index']),int(r['physical_ns']))][key]=delta
        if r['phase']=='process_end': final[key]=current
    root=final[('other','run')]['inclusive_ns']
    # Every completed child interval is assigned exactly once, including
    # unmatched outer bookkeeping. The final profiler flush itself is excluded.
    assert sum(r['exclusive_ns'] for r in final.values()) == root
    pubs={tag:{int(r['index']):float(r['seconds'])*1000 for r in rows(ROOT/tag/'publication_convergence.csv')
         if r['phase']=='support'} for tag in ['full_off','full_on']}
    result=dict(passed=True, foreign_scopes=0, inclusive_exclusive_reconcile=True,
      whole_run_wall_s=root/1e9, whole_run_cpu_s=final[('other','run')]['cpu_ns']/1e9,
      process_runtime={tag:json.loads((ROOT/(tag+'.json')).read_text()) for tag in ['full_off','full_on']},
      total_disjoint=[], windows={})
    for (context,tag),r in sorted(final.items(),key=lambda pair:pair[1]['exclusive_ns'],reverse=True):
        result['total_disjoint'].append(dict(context=context,tag=tag,calls=r['calls'],
          exclusive_s=r['exclusive_ns']/1e9, exclusive_percent=100*r['exclusive_ns']/root,
          inclusive_s=r['inclusive_ns']/1e9,cpu_s=r['cpu_ns']/1e9))
    for lo,hi in [(0,40),(40,100),(100,120),(120,140),(140,148.580001)]:
        selected=[(index,data) for (index,ns),data in snapshots.items() if lo<=(ns-BASE)/1e9<hi]
        totals=defaultdict(lambda:dict(calls=0,inclusive_ns=0,exclusive_ns=0,cpu_ns=0))
        for _,data in selected:
            for key,r in data.items():
                for k,v in r.items(): totals[key][k]+=v
        pub=totals[('publication','publication')]['inclusive_ns']
        d=dict(publications=len(selected), wall_ms={tag:stats(pubs[tag][i] for i,_ in selected) for tag in pubs},
          publication_inclusive_s=pub/1e9,
          publication_cpu_s=totals[('publication','publication')]['cpu_ns']/1e9,
          components=[])
        for (context,tag),r in sorted(totals.items(),key=lambda pair:pair[1]['exclusive_ns'],reverse=True):
            d['components'].append(dict(context=context,tag=tag,calls=r['calls'],exclusive_s=r['exclusive_ns']/1e9,
              inclusive_s=r['inclusive_ns']/1e9,percent_of_publication_time=100*r['exclusive_ns']/pub if context=='publication' else None))
        d['instrumentation_overhead_fraction']=d['wall_ms']['full_on']['total']/d['wall_ms']['full_off']['total']-1
        result['windows'][f'{lo}-{hi}']=d
    with (ROOT/'analysis.json').open('x') as f: json.dump(result,f,indent=2)
    print('Whole-run seconds wall/CPU',result['whole_run_wall_s'],result['whole_run_cpu_s'])
    for label,b in result['windows'].items():
        print(label,'pub seconds',b['publication_inclusive_s'],'overhead',b['instrumentation_overhead_fraction'])
        for r in [r for r in b['components'] if r['context']=='publication'][:9]:
            print(' ',r['tag'],r['exclusive_s'],r['percent_of_publication_time'])


if __name__=='__main__': main()
