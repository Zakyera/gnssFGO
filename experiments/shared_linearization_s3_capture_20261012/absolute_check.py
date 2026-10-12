"""Report absolute norms alongside the predeclared relative oracle gates."""
import json
import numpy as np
from scipy import linalg
from oracle import Reader,Oracle,assemble,jr,inv,adj
from prepare import ROOT,HERE

out=HERE/'absolute_errors.json';assert not out.exists()
o=Oracle();s=None;p=None;rows=[]
reader=Reader(ROOT/'profile40_on_source/native_gaussian.bin')
try:
    for tag,r in reader.records():
        if tag=='S':s=r
        elif tag=='C':o.commit(s,r)
        elif tag=='P':p=r
        elif tag=='Q':
            keys,dims,h,offset=assemble(([o.prior] if o.prior else [])+[r[2] for r in o.active.values()])
            _,_,candidate,_=assemble(p['factors'],keys)
            idx=[offset[k]+j for k in [p['a'],p['b']] for j in range(6)]
            H=h[:-1,:-1];cov=linalg.solve(H,np.eye(len(H))[:,idx],assume_a='sym')[idx,:]
            a=linalg.block_diag(jr(p['df']),jr(p['dt']));current=a@cov@a.T
            z=inv(p['pf'])@p['pt'];j=np.column_stack((-adj(inv(z)),np.eye(6)))
            relative=j@current@j.T
            rows.append(dict(epoch=o.epoch,information_frobenius=float(linalg.norm(candidate-h)),
                information_max_abs=float(np.max(np.abs(candidate-h))),
                joint_covariance_frobenius=float(linalg.norm(r['cov']-cov)),
                current_covariance_frobenius=float(linalg.norm(r['current']-current)),
                relative_covariance_frobenius=float(linalg.norm(r['relative']-relative))))
            o.query(p,r)
        elif tag=='F':raise RuntimeError(r)
finally:reader.f.close()
result=dict(passed=True,checkpoints=rows,maxima={k:max(r[k] for r in rows) for k in rows[0] if k!='epoch'})
out.write_text(json.dumps(result,indent=2));print(json.dumps(result['maxima'],indent=2))
