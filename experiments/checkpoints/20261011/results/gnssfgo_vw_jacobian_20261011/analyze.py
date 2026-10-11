"""Bounded replay comparison; never computes accuracy or changes a setting."""
import collections, csv, hashlib, json, math, statistics
from pathlib import Path
root=Path(__file__).resolve().parent
old=Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/cached')
new=root/'candidate'; t0=1727775008003475000
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def stats(values):
    v=sorted(float(x) for x in values)
    if not v:return {'n':0}
    k=(len(v)-1)*.95;i=int(k)
    return dict(n=len(v),mean=statistics.mean(v),median=statistics.median(v),
      rms=math.sqrt(sum(x*x for x in v)/len(v)),p95=v[i]+(v[min(i+1,len(v)-1)]-v[i])*(k-i),maximum=max(v),sum=sum(v))
def summary(p):
    c=rows(p/'publication_convergence.csv'); states=rows(p/'online_states.csv')
    r=rows(p/'revision_events.csv');d=rows(p/'causal_dispatch.csv');u=rows(p/'unsupported_beliefs.csv')
    valid=[x for x in c if x['status']=='stationary']
    failed=[x for x in c if x['status']!='stationary']
    result=dict(convergence_rows=len(c),published=len(states),last=c[-1],
      failed=failed,valid_max_gradient=max(float(x['gradient_after']) for x in valid),
      failed_not_published=all(x['index'] not in {s['state_index'] for s in states} for x in failed),
      publication_seconds=stats(x['seconds'] for x in c),
      revision_outcomes=dict(collections.Counter(x['outcome'] for x in r)),
      dispatch_outcomes=dict(collections.Counter(x['outcome'] for x in d)),
      dispatched=len(d),unsupported=len(u),
      final_trajectory_bytes=(p/'trajectory.tum').stat().st_size,
      last_published_elapsed=(int(states[-1]['timestamp_ns'])-t0)/1e9,
      last_attempt_elapsed=(int(c[-1]['physical_ns'])-t0)/1e9)
    result['windows']=[]
    for lo,hi in [(0,60),(60,100),(100,115),(115,120),(120,125),(125,130),(130,160)]:
      a=[x for x in c if lo<=(int(x['physical_ns'])-t0)/1e9<hi]
      result['windows'].append(dict(elapsed=[lo,hi],publication=stats(x['seconds'] for x in a)))
    return result
result={'baseline':summary(old),'candidate':summary(new)}
result['unchanged_prefixes']={}
for name,drop in [('causal_dispatch.csv',set()),
    ('revision_events.csv',{'update_seconds','current_position_correction_m','endpoint_position_correction_m'}),
    ('unsupported_beliefs.csv',set()),('support_clock.csv',set()),('model_covariance.csv',set())]:
    a,b=rows(old/name),rows(new/name);n=min(len(a),len(b))
    clean=lambda x:{k:v for k,v in x.items() if k not in drop}
    diff=next((i for i in range(n) if clean(a[i])!=clean(b[i])),None)
    result['unchanged_prefixes'][name]=dict(old_rows=len(a),new_rows=len(b),compared=n,
      same_prefix=diff is None,first_difference=diff,excluded=sorted(drop))
a,b=rows(old/'online_states.csv'),rows(new/'online_states.csv')
bytime={x['timestamp_ns']:x for x in a};diffs={k:[] for k in ('position_m','velocity_m_s','omega_rad_s','orientation_rad')}
for y in b:
    if y['timestamp_ns'] not in bytime:continue
    x=bytime[y['timestamp_ns']]
    for label,keys in [('position_m',['px','py','pz']),('velocity_m_s',['vx','vy','vz']),('omega_rad_s',['wx','wy','wz'])]:
      diffs[label].append(math.sqrt(sum((float(y[k])-float(x[k]))**2 for k in keys)))
    q=[float(x[k]) for k in ('qx','qy','qz','qw')];s=[float(y[k]) for k in ('qx','qy','qz','qw')]
    # Relative quaternion angle, stable for tiny rotations.
    ax,ay,az,aw=q;bx,by,bz,bw=s
    vec=[aw*bx-ax*bw-ay*bz+az*by,aw*by+ax*bz-ay*bw-az*bx,aw*bz-ax*by+ay*bx-az*bw]
    scalar=sum(i*j for i,j in zip(q,s))
    diffs['orientation_rad'].append(2*math.atan2(math.sqrt(sum(z*z for z in vec)),abs(scalar)))
result['common_published_state_difference']={k:stats(v) for k,v in diffs.items()}
ca,cb=rows(old/'publication_convergence.csv'),rows(new/'publication_convergence.csv')
lookup={(x['phase'],x['index']):x for x in ca};pairs=[(lookup[(x['phase'],x['index'])],x) for x in cb if (x['phase'],x['index']) in lookup]
result['common_refinement_runtime']={'supports':len(pairs),'baseline_seconds':sum(float(a['seconds']) for a,b in pairs),
 'candidate_seconds':sum(float(b['seconds']) for a,b in pairs)}
result['first_changed_decision']=next((dict(index=x['index'],baseline={k:x[k] for k in ('attempted','accepted','status')},
    candidate={k:y[k] for k in ('attempted','accepted','status')}) for x,y in pairs
    if any(x[k]!=y[k] for k in ('attempted','accepted','status'))),None)
result['checks']=dict(unchanged_identity_timing_covariance_prefixes=all(x['same_prefix'] for x in result['unchanged_prefixes'].values()),
 valid_publications_meet_threshold=result['candidate']['valid_max_gradient']<=1e-5,
 failed_state_not_published=result['candidate']['failed_not_published'])
result['replay_command']=json.loads((root/'command_replay.json').read_text())
result['receiver_sha256']=hashlib.sha256((root/'build/replay').read_bytes()).hexdigest()
with (root/'comparison.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='replay_command'},indent=2))
assert all(result['checks'].values()), 'Comparison validation failure: stop'
