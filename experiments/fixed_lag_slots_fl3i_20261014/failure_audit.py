"""Bounded saved-output diagnosis after the full numerical gate failed.
No optimizer, replay, tolerance change, or promotion.
"""
import csv,json,math
import numpy as np
from driver import ROOT,BASEROOT,protected

LIMIT=np.array([1e-6,1e-8,1e-6,1e-6])
NAMES=['position_m','orientation_rad','velocity_mps','omega_radps']
def states(root):
    out={'online_incremental':[],'finalized_incremental':[]}
    for p in sorted((root/'full_outputs/commits').iterdir(),key=lambda p:int(p.name)):
        with (p/'records.txt').open() as f:
            for r in csv.reader(f):
                if r[0] in out:out[r[0]].append(r)
    return out
def main():
    actual,expected=states(ROOT),states(BASEROOT)
    result={'passed':False,'failure':'predeclared FL3h state equivalence tolerance',
            'limits':dict(zip(NAMES,LIMIT.tolist())),'kinds':{}}
    for kind in actual:
        assert len(actual[kind])==len(expected[kind])
        data=[]
        for a,b in zip(actual[kind],expected[kind]):
            assert a[1:4]==b[1:4]
            v,u=np.array(a[4:],float),np.array(b[4:],float)
            q,q0=v[3:7]/np.linalg.norm(v[3:7]),u[3:7]/np.linalg.norm(u[3:7])
            angle=2*math.atan2(np.linalg.norm(q0[3]*q[:3]-q[3]*q0[:3]-np.cross(q0[:3],q[:3])),abs(q@q0))
            data.append([int(a[1]),np.linalg.norm(v[:3]-u[:3]),angle,np.linalg.norm(v[7:10]-u[7:10]),np.linalg.norm(v[10:13]-u[10:13])])
        data=np.array(data);error=data[:,1:];bad=np.any(error>LIMIT,axis=1)
        first=int(np.argmax(bad)) if bad.any() else None
        result['kinds'][kind]={'count':len(data),'max':dict(zip(NAMES,error.max(axis=0).tolist())),
          'rms':dict(zip(NAMES,np.sqrt(np.mean(error*error,axis=0)).tolist())),
          'max_support':dict(zip(NAMES,data[np.argmax(error,axis=0),0].astype(int).tolist())),
          'failed_rows':int(bad.sum()),'first_failed_support':None if first is None else int(data[first,0]),
          'first_failed_errors':None if first is None else dict(zip(NAMES,error[first].tolist()))}
        with (ROOT/('failed_gate_'+kind+'_differences.csv')).open('x') as f:
            w=csv.writer(f);w.writerow(['support']+NAMES);w.writerows(data)
    result['protected_unchanged']=protected()==json.loads((ROOT/'protected.json').read_text())
    assert result['protected_unchanged']
    with (ROOT/'full_failure_audit.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
