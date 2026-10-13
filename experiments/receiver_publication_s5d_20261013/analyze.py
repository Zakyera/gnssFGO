"""Read-only scientific CSV analysis, separate JSON results; no GT or tuning."""
import csv
import json
from pathlib import Path
import numpy as np

ROOT=Path('/home/yeranis/results/gnssfgo_publication_s5d_20261013')
REF=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/full_on1')

def rows(path):
    with path.open() as f:return list(csv.DictReader(f))

def stats(x):
    x=np.asarray(x,dtype=float)
    return dict(count=len(x),mean=float(x.mean()),median=float(np.median(x)),
                rms=float(np.sqrt(np.mean(x*x))),p95=float(np.quantile(x,.95)),max=float(x.max())) if len(x) else None

def angle(a,b):
    # Quaternion inverse(a)*b in xyzw order, shortest rotation, radians.
    a=a/np.linalg.norm(a,axis=-1,keepdims=True);b=b/np.linalg.norm(b,axis=-1,keepdims=True)
    v=a[...,3,None]*b[...,:3]-b[...,3,None]*a[...,:3]-np.cross(a[...,:3],b[...,:3])
    w=np.sum(a*b,axis=-1)
    return 2*np.arctan2(np.linalg.norm(v,axis=-1),np.abs(w))

def delta(a,b):
    return dict(p_m=np.linalg.norm(b[:,:3]-a[:,:3],axis=1),
                q_rad=angle(a[:,3:7],b[:,3:7]),
                v_mps=np.linalg.norm(b[:,7:10]-a[:,7:10],axis=1),
                w_radps=np.linalg.norm(b[:,10:13]-a[:,10:13],axis=1))

def main():
    assert np.isclose(angle(np.array([[0.,0.,0.,1.]]),np.array([[0.,0.,.5,np.sqrt(.75)]]))[0],np.pi/3)
    assert angle(np.array([[0.,0.,0.,1.]]),np.array([[0.,0.,0.,-1.]]))[0]==0
    assert json.loads((ROOT/'full_on.json').read_text())['passed'],'reject unvalidated observer'
    conv=rows(REF/'publication_convergence.csv');steps=rows(REF/'publication_steps.csv')
    clocks={int(r['state_index']):int(r['physical_timestamp_ns']) for r in rows(REF/'support_clock.csv')}
    t0=clocks[0]
    result=dict(reference=str(REF),observed=str(ROOT/'full_on'),cohorts={},snapshots=[])
    for lo,hi in [(0,40),(40,80),(80,120),(120,140),(140,149)]:
        c=[r for r in conv if r['phase']=='support' and lo<=(int(r['physical_ns'])-t0)*1e-9<hi]
        ids={int(r['index']) for r in c}
        s=[r for r in steps if r['phase']=='support' and int(r['index']) in ids]
        result['cohorts'][f'{lo}-{hi}']=dict(count=len(c),before_gradient=stats([float(r['gradient_before']) for r in c]),
            initial_stationary=sum(float(r['gradient_before'])<=1e-5 for r in c),
            attempts=stats([int(r['attempted']) for r in c]),
            objective_drop=stats([float(r['error_before'])-float(r['error_after']) for r in c]),
            publication_ms=stats([1000*float(r['seconds']) for r in c]),
            recovery_publications=len({r['index'] for r in s if r['recovery']=='1'}))
    cols=['px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz']
    published={int(r['state_index']):np.array([float(r[k]) for k in cols]) for r in rows(REF/'online_states.csv')}
    for folder in sorted((ROOT/'full_on').glob('s5d_support_*'),key=lambda p:int(p.name.split('_')[-1])):
        i=int(folder.name.split('_')[-1]);frames=rows(folder/'frames.csv')
        s=[r for r in steps if r['phase']=='support' and int(r['index'])==i]
        c=next(r for r in conv if r['phase']=='support' and int(r['index'])==i)
        assert len(frames)==len(s)+1,(i,len(frames),len(s))
        assert float(frames[0]['normalized_gradient'])==float(c['gradient_before'])
        data=np.genfromtxt(folder/'states.csv',delimiter=',',skip_header=1)
        assert len(data)==len(frames)*(i+1)
        assert np.array_equal(data[:,0],np.repeat(np.arange(len(frames)),i+1))
        assert np.array_equal(data[:,1],np.tile(np.arange(i+1),len(frames)))
        states=data[:,2:].reshape(len(frames),i+1,13)
        final=max([0]+[n+1 for n,r in enumerate(s) if r['accepted']=='1'])
        assert np.max(np.abs(states[final,-1]-published[i]))<=1e-12,(i,'publication mismatch')
        assert float(frames[final]['normalized_gradient'])==float(c['gradient_after'])
        age=np.array([(clocks[i]-clocks[j])*1e-9 for j in range(i+1)])
        initial=delta(states[0],states[final]);snap=dict(index=i,time_s=(clocks[i]-t0)*1e-9,
            final_frame=final,attempts=int(c['attempted']),recovery=any(r['recovery']=='1' for r in s),
            initial_gradient=float(c['gradient_before']),final_gradient=float(c['gradient_after']),
            initial_objective=float(c['error_before']),final_objective=float(c['error_after']),
            publication_ms=float(c['seconds'])*1000,
            initial_to_final={k:dict(all=stats(v),latest=float(v[-1]),recent5=stats(v[age<=5]),old5=stats(v[age>5]),
                maximum_state_index=int(np.argmax(v)),maximum_state_age_s=float(age[np.argmax(v)])) for k,v in initial.items()},
            frames=[],gradient_breakdown={})
        for n,f in enumerate(frames):
            if n==0:meta=dict(kind='incremental',accepted=True,objective=float(c['error_before']))
            else:
                r=s[n-1];assert float(f['normalized_gradient'])==float(r['gradient_after'])
                meta=dict(kind='recovery' if r['recovery']=='1' else 'trial',accepted=r['accepted']=='1',
                          objective=float(r['error_after']),iteration=int(r['iteration']),relinearized=int(r['relinearized']))
            dif=delta(states[n],states[final]);prev=delta(states[max(0,n-1)],states[n])
            meta.update(frame=n,gradient=float(f['normalized_gradient']),
                        remaining={k:dict(max=float(v.max()),rms=float(np.sqrt(np.mean(v*v))),latest=float(v[-1])) for k,v in dif.items()},
                        remaining_counts={k:{str(t):int(np.count_nonzero(dif[k]>t)) for t in limits}
                                          for k,limits in [('p_m',[1e-6,1e-4,1e-3,1e-2]),('q_rad',[1e-6,1e-4,1e-3])]},
                        preceding_frame_step={k:float(v.max()) for k,v in prev.items()})
            snap['frames'].append(meta)
        gs=rows(folder/'gradients.csv')
        for n in [0,final]:
            g=[r for r in gs if int(r['frame'])==n]
            assert len(g)==12*(i+1)
            assert np.isclose(max(float(r['normalized']) for r in g),float(frames[n]['normalized_gradient']),rtol=0,atol=0)
            blocks={}
            for name,code,components in [('rotation','x',range(3)),('translation','x',range(3,6)),('velocity','v',range(3)),('omega','w',range(3))]:
                part=[r for r in g if r['key'][0]==code and int(r['coordinate']) in components]
                above=[r for r in part if float(r['normalized'])>1e-5]
                top=max(part,key=lambda r:float(r['normalized']))
                blocks[name]=dict(max=float(top['normalized']),key=top['key'],coordinate=int(top['coordinate']),
                    state_age_s=float(age[int(top['key'][1:])]),violating_coordinates=len(above),
                    violating_old5=sum(age[int(r['key'][1:])]>5 for r in above),coordinates=len(part))
                blocks[name]['violating_old5']=int(blocks[name]['violating_old5'])
            snap['gradient_breakdown'][str(n)]=blocks
        result['snapshots'].append(snap)
    assert len(result['snapshots'])==len(json.loads((ROOT/'select.json').read_text())['indices'])
    with (ROOT/'analysis.json').open('x') as f:json.dump(result,f,indent=2)
    for s in result['snapshots']:
        d=s['initial_to_final'];first=s['frames'][1] if len(s['frames'])>1 else s['frames'][0]
        print(s['index'],round(s['time_s'],2),'attempts',s['attempts'],'g',f"{s['initial_gradient']:.6g}",
              'max_p_mm',f"{1000*d['p_m']['all']['max']:.6g}",'current_p_mm',f"{1000*d['p_m']['latest']:.6g}",
              'first_remaining_mm',f"{1000*first['remaining']['p_m']['max']:.6g}")
if __name__=='__main__':main()
