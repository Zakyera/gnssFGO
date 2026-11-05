"""Numerical gate, not a timing analysis; accepts only predeclared tolerances."""
from pathlib import Path
import json
import math
import sys

def load(path):
    data={}
    counters={}
    with Path(path).open() as f:
        for line in f:
            label,kind,*values=line.split()
            target=counters if kind=="counts" else data
            if label in target:raise AssertionError("duplicate record "+label)
            target[label]=(kind,values)
    return data,counters

def compare(reference,candidate,exact=False):
    a,ca=load(reference);b,cb=load(candidate)
    assert a.keys()==b.keys(),("record identity differs",list(a.keys()-b.keys())[:5],list(b.keys()-a.keys())[:5])
    maxima={};counts={};worst={};numeric=0
    for label,(kind,raw) in a.items():
        other,rb=b[label]
        assert other==kind,(label,kind,other)
        counts[kind]=counts.get(kind,0)+1
        if kind in ("exact","bytes"):
            assert raw==rb,("exact mismatch",label)
            continue
        assert raw[:2]==rb[:2],("shape",label)
        nr,nc=map(int,raw[:2]);va=list(map(float,raw[2:]));vb=list(map(float,rb[2:]))
        assert len(va)==len(vb)==nr*nc,("size",label)
        assert all(map(math.isfinite,va+vb)),("nonfinite",label)
        numeric+=len(va)
        diff=max(map(abs,(x-y for x,y in zip(va,vb))),default=0)
        bound=0 if exact else (
          1e-9 if kind=="state" else
          1e-10+1e-6*max(map(abs,va),default=0) if kind=="cov" else
          1e-8+1e-6*max(map(abs,va),default=0))
        if kind=="pose":
            assert (nr,nc)==(4,4)
            translation=max(abs(va[4*i+3]-vb[4*i+3]) for i in range(3))
            # R_ref.T R_candidate; atan2 avoids an acos rounding floor.
            rel=[[sum(va[4*k+i]*vb[4*k+j] for k in range(3)) for j in range(3)] for i in range(3)]
            skew=[rel[2][1]-rel[1][2],rel[0][2]-rel[2][0],rel[1][0]-rel[0][1]]
            angle=math.atan2(.5*math.sqrt(sum(x*x for x in skew)),
                            .5*(sum(rel[i][i] for i in range(3))-1))
            assert translation<=(0 if exact else 1e-9),("translation",label,translation)
            assert angle<=(0 if exact else 1e-10),("orientation",label,angle)
            for key,value in [("translation",translation),("orientation_rad",angle)]:
                if value>=maxima.get(key,0):maxima[key]=value;worst[key]=label
            # Exact-disabled gate also checks complete matrix bytes-as-numbers.
            if exact:assert diff==0,("disabled pose differs",label,diff)
        else:assert diff<=bound,("numerical mismatch",label,diff,bound)
        if diff>=maxima.get(kind,0):maxima[kind]=diff;worst[kind]=label
    def tally(c):
        totals={}
        for label,(kind,vals) in c.items():
            if label.endswith("/profile"):
                group=label.split("/")[0]
                sums=totals.setdefault(group,[0]*len(vals))
                for i,v in enumerate(vals):sums[i]+=int(v)
        return totals
    return dict(passed=True,exact_disabled=exact,records=len(a),numeric_entries=numeric,
                counts=counts,max_absolute_differences=maxima,worst_record=worst,
                reference_counters=tally(ca),candidate_counters=tally(cb),
                candidate_sites=cb.get("direct/sites"))

if __name__=="__main__":
    result=compare(sys.argv[1],sys.argv[2],len(sys.argv)>3 and sys.argv[3]=="exact")
    print(json.dumps(result,indent=2))
