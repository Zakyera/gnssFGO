"""Whole-service timing including source revisions, not solve time alone."""
import collections,csv,json,sys
import numpy as np
from driver import ROOT
ORIGIN=1727775008583475000

def stats(x):
    a=np.asarray(x,float)
    return dict(count=len(a),mean=float(a.mean()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max())) if len(a) else None
def analyze(name):
    with (ROOT/(name+'_service.csv')).open() as f:r=list(csv.DictReader(f))
    blocks=[];bucket=0.;belief=0.;prior=0;start=int(r[0]['arrival_ns']);finish=0.;completion=collections.deque();backlogs=[];delays=[];phases=collections.defaultdict(list)
    for row in r:
        sec=float(row['seconds']);bucket+=sec
        if row['kind']=='B':belief+=sec
        new=int(row['publications'])>prior
        phases['support_event' if new else 'belief_event' if row['kind']=='B' else 'EOF' if row['kind']=='E' else 'raw_clock_only'].append(sec)
        if new:
            assert int(row['publications'])==prior+1
            blocks.append(dict(time=(int(row['arrival_ns'])-ORIGIN)*1e-9,seconds=bucket,belief_seconds=belief,publication_seconds=sec,
              supports=int(row['supports']),active=int(row['active']),pending=int(row['pending']),slots=int(row['slots']),
              nonempty=int(row['nonempty']),marginals=int(row['marginals']),marginal_max_dimension=int(row['marginal_max_dimension']),rss_kib=int(row['rss_kib'])))
            prior+=1;bucket=belief=0.
        release=(int(row['arrival_ns'])-start)*1e-9/.1
        while completion and completion[0]<=release:completion.popleft()
        backlogs.append((release*.1,len(completion)))
        finish=max(finish,release)+sec;completion.append(finish);delays.append((release*.1,finish-release))
    bins={}
    for label,lo,hi in [('early',0,20),('middle',60,80),('late',120,150),('final',140,150)]:
        rows=[b for b in blocks if lo<=b['time']<hi]
        if not rows:continue
        bins[label]=dict(total=stats([b['seconds'] for b in rows]),
           source_versions=stats([b['belief_seconds'] for b in rows]),
           support_publication=stats([b['publication_seconds'] for b in rows]),
           maximum={k:max(b[k] for b in rows) for k in ('supports','active','pending','slots','nonempty','marginals','marginal_max_dimension','rss_kib')},
           queue_max=max(v for t,v in backlogs if lo<=t<hi),simulated_latency=stats([v for t,v in delays if lo<=t<hi]))
    overall=stats([b['seconds'] for b in blocks]);late=bins.get('late',bins['early'])
    # No accumulated backlog: drain fits one support-wall interval at each late-bin end;
    # also report exact queue/delay so burst latency is not concealed by this check.
    tails={}
    for lo in range(0,150,10):
        selected=[v for t,v in delays if lo<=t<lo+10]
        if selected:tails[str(lo)]=selected[-1]
    passed=overall['p95']<=.8 and late['total']['mean']<1. and max(v for t,v in backlogs)<1000 and all(v<1. for k,v in tails.items() if int(k)>=70)
    run=json.loads((ROOT/(name+'.json')).read_text());completed=run['returncode']==0
    measured=sum(float(x['seconds']) for x in r)
    result=dict(case=name,completed=completed,process_wall_seconds=run['seconds'],
      scope='completed logged events; failed transaction not timed separately' if not completed else 'complete replay',
      wall_minus_logged_seconds=run['seconds']-measured,whole_support_seconds=overall,bins=bins,event_phase_seconds={k:stats(v) for k,v in phases.items()},
      total_measured_seconds=sum(float(x['seconds']) for x in r),eof_unassigned_tail_seconds=bucket,
      simulated_queue_max=max(v for t,v in backlogs),simulated_latency_seconds=stats([v for t,v in delays]),
      ten_second_bin_end_latency_seconds=tails,partial_timing_thresholds_met=passed,operational_target_pass=passed and completed,
      gate='p95<=.8s,late mean<1s,queue<1000,no accumulated late-bin tail>=1s',
      limitation='offline single-core recorded-arrival simulation; no co-running source contention; diagnostic snapshots excluded; not a production/live acceptance')
    with (ROOT/(name+'_timing_analysis.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2));return result
if __name__=='__main__':analyze(sys.argv[1])

