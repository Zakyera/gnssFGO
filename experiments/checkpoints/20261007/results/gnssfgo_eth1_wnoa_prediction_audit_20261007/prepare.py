from pathlib import Path
import csv, json, hashlib, subprocess
import numpy as np
from scipy.spatial.transform import Rotation

R=Path(__file__).resolve().parent
P=R.parent/'gnssfgo_eth1_local_prediction_20261007'
I=R.parent/'gnssfgo_eth1_wnoa_wnoj_120s_20261006/inputs'
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()
def write(name,data):
    with (R/name).open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
def states(p):
    a=np.loadtxt(p,delimiter=',',skiprows=1,ndmin=2)
    # Preserve nanosecond timestamps as integers rather than float epoch times.
    rr=rows(p);ts=np.array([int(r['timestamp_ns']) for r in rr],dtype=np.int64)
    assert np.isfinite(a).all() and np.array_equal(a[:,0],np.arange(len(a)))
    return a,ts
def diff(a,b):
    return dict(position_m=float(np.linalg.norm(b[2:5]-a[2:5])),
        orientation_rad=float((Rotation.from_quat(a[5:9]).inv()*Rotation.from_quat(b[5:9])).magnitude()),
        velocity_mps=float(np.linalg.norm(b[9:12]-a[9:12])),
        omega_radps=float(np.linalg.norm(b[12:15]-a[12:15])))

if __name__=='__main__':
    protected={}
    for mode in ['off','on']:
        for p in (P/f'{mode}_wnoa').rglob('*'):
            if p.is_file():protected[str(p)]=sha(p)
        p=I/f'{mode}_events.csv';protected[str(p)]=sha(p)
    for p in (P/'build/wnoa').glob('*'):
        if p.is_file():protected[str(p)]=sha(p)
    repo=Path('/home/yeranis/repos/gnssFGO')
    for p in repo.rglob('*'):
        if p.is_file() and '.git' not in p.parts:protected[str(p)]=sha(p)
    (R/'manifest.json').write_text(json.dumps(protected,indent=2)+'\n')
    (R/'preexisting_status.txt').write_text(subprocess.check_output(['git','status','--short'],cwd=repo,text=True))
    (R/'head.txt').write_text(subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True))
    pubs=[];changes=[];requests=[];selected={};san=[]
    for mode in ['off','on']:
        case=P/f'{mode}_wnoa'
        dirs=sorted([p for p in case.glob('snapshot_*') if p.name!='snapshot_final'],key=lambda p:int(p.name.split('_')[1]))
        dirs.append(case/'snapshot_final')
        data=[states(p/'incremental_states.csv') for p in dirs]
        ledgers=[rows(p/'ledger.csv') for p in dirs]
        frontiers=[max([int(l['to_ns']) for l in ll],default=0) for ll in ledgers]
        currentrows=[]
        for k,(folder,(a,t),ll,F) in enumerate(zip(dirs,data,ledgers,frontiers)):
            label=folder.name[9:];n=len(a)-1;future=data[k+1] if k+1<len(data) else None
            c=int(np.searchsorted(t,F,side='right')-1) if F else -1
            b=int(np.searchsorted(t,F)) if F else -1
            assert not F or 0<=c<=b<=n
            d=dict(mode=mode,snapshot=label,now_ns=int(t[-1]),frontier_ns=F,covered=c,boundary=b,current=n,
                endpoint_delay_s=(int(t[-1])-F)*1e-9 if F else '',
                pure_prediction_s=(int(t[-1])-int(t[b]))*1e-9 if F else '',
                speed_mps=float(np.linalg.norm(a[-1,9:12])),omega_radps=float(np.linalg.norm(a[-1,12:15])),
                source_ids=len(ll),glim_ids=sum(l['source'].startswith('glim') for l in ll))
            pubs.append(d)
            if not future or not F:continue
            nexta,nextt=future
            assert np.array_equal(t,nextt[:len(t)])
            nextledger={tuple(l[x] for x in ['source','session','from_index','to_index']):l['revision'] for l in ledgers[k+1]}
            oldledger={tuple(l[x] for x in ['source','session','from_index','to_index']):l['revision'] for l in ll}
            new=len(nextledger.keys()-oldledger.keys());revs=sum(nextledger.get(i)!=v for i,v in oldledger.items())
            for zone,idx in [('current',n),('covered',c),('boundary',b)]:
                dd=dict(mode=mode,snapshot=label,next_snapshot=dirs[k+1].name[9:],zone=zone,state_index=idx,
                    timestamp_ns=int(t[idx]),new_ids=new,replacements=revs,**diff(a[idx],nexta[idx]))
                changes.append(dd)
                if zone=='current':currentrows.append({**d,**diff(a[idx],nexta[idx])})
            first=next((j for j in range(k+1,len(data)) if frontiers[j]>=int(t[-1])),None)
            if first is not None:
                changes.append(dict(mode=mode,snapshot=label,next_snapshot=dirs[first].name[9:],zone='first_covered',state_index=n,
                    timestamp_ns=int(t[n]),new_ids='',replacements='',**diff(a[n],data[first][0][n])))
            changes.append(dict(mode=mode,snapshot=label,next_snapshot='final',zone='final',state_index=n,
                timestamp_ns=int(t[n]),new_ids='',replacements='',**diff(a[n],data[-1][0][n])))
        candidates=[d for d in pubs if d['mode']==mode and d['frontier_ns']]
        reasons={}
        def select(label,why):reasons.setdefault(str(label),[]).append(why)
        for d in candidates:
            if d['snapshot']!='final' and int(d['snapshot'])%20==0:select(d['snapshot'],'regular_20')
        select(candidates[0]['snapshot'],'first_source');select('final','final')
        for key in ['endpoint_delay_s','pure_prediction_s','position_m','orientation_rad','velocity_mps','omega_radps','speed_mps']:
            # omega_radps in currentrows is revision magnitude; rate also selected separately below.
            for d in sorted(currentrows,key=lambda d:d[key],reverse=True)[:5]:select(d['snapshot'],'largest_'+key)
        for d in sorted(candidates,key=lambda d:d['omega_radps'],reverse=True)[:5]:select(d['snapshot'],'largest_rate')
        probes=[d for d in candidates if (case/f"snapshot_{d['snapshot']}/factor_payload_probe.txt").exists()]
        for d in probes[::max(1,len(probes)//4)]:select(d['snapshot'],'payload_probe')
        selected[mode]=reasons
        ev={}
        with (I/f'{mode}_events.csv').open() as f:
            for e in csv.reader(f):
                if e[0]=='B':
                    key=(e[2],e[3],e[6],e[7],e[5]);assert key not in ev;ev[key]=e
        for d in candidates:
            if d['snapshot'] not in reasons:continue
            folder=case/f"snapshot_{d['snapshot']}";payload=[]
            for l in rows(folder/'ledger.csv'):
                e=ev[tuple(l[k] for k in ['source','session','from_index','to_index','revision'])]
                assert e[1]==l['arrival_ns'] and e[8]==l['from_ns'] and e[9]==l['to_ns']
                assert int(e[1])<=d['now_ns']+100000000 # publication may include late clock availability
                payload.append([l['slot'],l['source'],*e[8:]])
            with (R/f"{mode}_{d['snapshot']}_factors.csv").open('x',newline='') as f:
                w=csv.writer(f);w.writerow(['slot','source','from_ns','to_ns',*[f'mu{i}' for i in range(6)],*[f'c{i}' for i in range(36)]]);w.writerows(payload)
            req={k:d[k] for k in ['mode','snapshot','frontier_ns','covered','boundary','current']};requests.append(req)
        worst=max(currentrows,key=lambda d:d['position_m'])['snapshot']
        san.extend(r for r in requests if r['mode']==mode and r['snapshot'] in [candidates[0]['snapshot'],worst,'final'])
        print(mode,len(dirs),'snapshots',len(reasons),'covariance requests',flush=True)
    write('publications.csv',pubs);write('changes.csv',changes);write('requests.csv',requests);write('sanitizer_requests.csv',san)
    (R/'selection.json').write_text(json.dumps(selected,indent=2)+'\n')
    print('protected',len(protected),flush=True)
