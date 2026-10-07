from pathlib import Path
import csv,json
import numpy as np
from scipy.spatial.transform import Rotation
from prepare import ROOT
from beliefs import stat

def rows(p):return list(csv.DictReader(p.open()))
def write(p,data):
    with p.open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
def change(a,b):
    return dict(position_m=float(np.linalg.norm(b[2:5]-a[2:5])),orientation_rad=float((Rotation.from_quat(a[5:9]).inv()*Rotation.from_quat(b[5:9])).magnitude()),
                velocity_mps=float(np.linalg.norm(b[9:12]-a[9:12])),omega_radps=float(np.linalg.norm(b[12:15]-a[12:15])))

pubs=[];changes=[];summary={};selections={};lastindices={}
for mode in ['off','on']:
    assert json.loads((ROOT/f'validation_{mode}.json').read_text())['passed']
    case=ROOT/(mode+'_wnoa');dirs=sorted(case.glob('snapshot_[0-9]*'),key=lambda p:int(p.name[9:]))+[case/'snapshot_final']
    arrays=[np.loadtxt(p/'incremental_states.csv',delimiter=',',skiprows=1,ndmin=2) for p in dirs]
    final_rows=rows(dirs[-1]/'incremental_states.csv');times=np.array([int(r['timestamp_ns']) for r in final_rows],dtype=np.int64)
    lastindices[mode]=len(times)-1
    ledgers=[]
    for p in dirs:
        ll=rows(p/'ledger.csv');ledgers.append({tuple(r[k] for k in ['source','session','from_index','to_index']):r for r in ll})
    front=[max((int(r['to_ns']) for r in ll.values()),default=0) for ll in ledgers]
    selected={'294','594','894',str(len(times)-1),'final'}
    for k,(p,a,L,F) in enumerate(zip(dirs,arrays,ledgers,front)):
        label=p.name[9:];n=len(a)-1;c=int(np.searchsorted(times,F,side='right')-1) if F else -1;b=int(np.searchsorted(times,F)) if F else -1
        meta=rows(p/'summary.csv')[0]
        assert all(int(r['arrival_ns'])<=int(meta['available_ns']) for r in L.values())
        pub=dict(mode=mode,snapshot=label,now_ns=int(times[n]),frontier_ns=F,covered=c,boundary=b,current=n,
                 endpoint_delay_s=(int(times[n])-F)*1e-9 if F else '',pure_prediction_s=(int(times[n])-int(times[b]))*1e-9 if F else '',
                 speed_mps=float(np.linalg.norm(a[n,9:12])),omega_radps=float(np.linalg.norm(a[n,12:15])),source_ids=len(L))
        pubs.append(pub)
        if not F or k+1==len(dirs):continue
        later=ledgers[k+1];nexta=arrays[k+1]
        counts={}
        for source in ['glim_hesai','kimera_front']:
            counts[source+'_new']=sum(key[0]==source for key in later.keys()-L.keys())
            counts[source+'_revised']=sum(key[0]==source and later.get(key,{}).get('revision')!=r['revision'] for key,r in L.items())
        for zone,i in [('covered',c),('boundary',b),('current',n)]:
            changes.append(dict(mode=mode,snapshot=label,next_snapshot=dirs[k+1].name[9:],zone=zone,timestamp_ns=int(times[i]),**counts,**change(a[i],nexta[i])))
        changes.append(dict(mode=mode,snapshot=label,next_snapshot='final',zone='final_at_same_time',timestamp_ns=int(times[n]),**counts,**change(a[n],arrays[-1][n])))
    ch=[r for r in changes if r['mode']==mode];current=[r for r in ch if r['zone']=='current'];worst=max(current,key=lambda r:r['position_m'])
    selected.update([worst['snapshot'],worst['next_snapshot']]);selections[mode]=selected
    summary[mode]={'changes':{z:{f:stat([r[f] for r in ch if r['zone']==z]) for f in ['position_m','orientation_rad','velocity_mps','omega_radps']} for z in ['covered','boundary','current','final_at_same_time']},
                   'largest_current_correction':worst,'largest_event_zones':[r for r in ch if r['snapshot']==worst['snapshot']],
                   'max_speed_mps':max(r['speed_mps'] for r in pubs if r['mode']==mode),'endpoint_delay_s':stat([r['endpoint_delay_s'] for r in pubs if r['mode']==mode and r['frontier_ns']])}
    print(mode,'snapshots',len(dirs),'largest event',worst,flush=True)
write(ROOT/'publications.csv',pubs);write(ROOT/'changes.csv',changes)
# Use the union of selected physical support indices in OFF and ON.
selected=set.union(*selections.values());requests=[];(ROOT/'payloads').mkdir(exist_ok=False)
for mode in ['off','on']:
    ev={}
    for e in csv.reader((ROOT/'inputs'/f'{mode}_events.csv').open()):
        if e[0]=='B':ev[(e[2],e[3],e[6],e[7],e[5])]=e
    for label in sorted(selected,key=lambda x:10**9 if x=='final' else int(x)):
        folder=ROOT/(mode+'_wnoa')/('snapshot_'+label);payload=[]
        for l in rows(folder/'ledger.csv'):
            e=ev[tuple(l[k] for k in ['source','session','from_index','to_index','revision'])]
            assert e[1]==l['arrival_ns'] and e[8]==l['from_ns'] and e[9]==l['to_ns']
            payload.append([l['slot'],l['source'],*e[8:]])
        with (ROOT/'payloads'/f'{mode}_{label}.csv').open('x',newline='') as f:
            w=csv.writer(f);w.writerow(['slot','source','from_ns','to_ns',*[f'mu{i}' for i in range(6)],*[f'c{i}' for i in range(36)]]);w.writerows(payload)
        requests.append(dict(mode=mode,snapshot=label))
write(ROOT/'requests.csv',requests)
(ROOT/'prediction_summary.json').write_text(json.dumps(summary,indent=2))
