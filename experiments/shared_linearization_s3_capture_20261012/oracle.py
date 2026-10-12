"""Independent dense native-history oracle for the private binary journal.

Does not import GTSAM, candidate elimination, candidate prior, or candidate
covariance code. Uses recorded weighted Gaussian inputs and SciPy solves.
"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import time
import numpy as np
from scipy import linalg
from scipy.spatial.transform import Rotation

class Reader:
    def __init__(self,path):
        self.f=Path(path).open('rb');self.blocks={}
        assert self.u()==0x314a4753
    def raw(self,n):
        b=self.f.read(n)
        if len(b)!=n:raise EOFError('incomplete journal record')
        return b
    def u(self):return struct.unpack('<Q',self.raw(8))[0]
    def scalar(self):return struct.unpack('<d',self.raw(8))[0]
    def ids(self):return [self.u() for _ in range(self.u())]
    def mat(self):
        r,c=self.u(),self.u();assert r*c<10000000
        a=np.frombuffer(self.raw(r*c*8),dtype='<f8').reshape(r,c).copy()
        assert np.isfinite(a).all();return a
    def factor(self):
        keys=[];dims=[]
        for _ in range(self.u()):keys.append(self.u());dims.append(self.u())
        a=self.mat();assert a.shape==(sum(dims)+1,)*2
        assert len(keys)==len(set(keys))
        return keys,dims,a
    def records(self):
        while tag:=self.f.read(1):
            epoch=self.u()
            if tag==b'S':
                gen=self.u();retire=set(self.ids());fixed=set(self.ids());removed=set(self.ids());theta={}
                for _ in range(self.u()):
                    key,dim,typ=self.u(),self.u(),self.u();theta[key]=(dim,typ,self.mat())
                refs={};slots=set()
                for _ in range(self.u()):
                    ident,version,slot=self.u(),self.u(),self.u();changed=self.u()
                    if changed:self.blocks[ident]=self.factor()
                    assert ident not in refs and slot not in slots
                    slots.add(slot);refs[ident]=(version,slot,self.blocks[ident])
                self.blocks={i:self.blocks[i] for i in refs}
                yield 'S',dict(epoch=epoch,gen=gen,retire=retire,fixed=fixed,removed=removed,theta=theta,refs=refs)
            elif tag==b'C':
                yield 'C',dict(epoch=epoch,counts=[self.u() for _ in range(4)],io_seconds=self.scalar())
            elif tag==b'P':
                a,b=self.u(),self.u();pf,pt,df,dt=[self.mat() for _ in range(4)]
                fs=[self.factor() for _ in range(self.u())]
                yield 'P',dict(epoch=epoch,a=a,b=b,pf=pf,pt=pt,df=df.ravel(),dt=dt.ravel(),factors=fs)
            elif tag==b'Q':
                cov,transport,current,relative=[self.mat() for _ in range(4)]
                yield 'Q',dict(epoch=epoch,cov=cov,transport=transport,current=current,relative=relative,seconds=self.scalar())
            elif tag==b'F':yield 'F',dict(epoch=epoch,reason=self.raw(self.u()).decode())
            else:raise ValueError(tag)

def assemble(fs,keys=None):
    dims={}
    for ks,ds,_ in fs:
        for k,d in zip(ks,ds):
            assert k not in dims or dims[k]==d;dims[k]=d
    keys=sorted(dims) if keys is None else keys
    offsets={};n=0
    for k in keys:offsets[k]=n;n+=dims[k]
    h=np.zeros((n+1,n+1))
    for ks,ds,a in fs:
        idx=[offsets[k]+j for k,d in zip(ks,ds) for j in range(d)]+[n]
        h[np.ix_(idx,idx)]+=a
    return keys,[dims[k] for k in keys],h,offsets

def rel(a,b):return float(linalg.norm(a-b)/max(linalg.norm(b),1e-300))
def skew(w):
    a,b,c=w;return np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
def exp(x):
    h=np.zeros((4,4));h[:3,:3]=skew(x[:3]);h[:3,3]=x[3:];return linalg.expm(h)
def inv(t):
    r=np.eye(4);r[:3,:3]=t[:3,:3].T;r[:3,3]=-r[:3,:3]@t[:3,3];return r
def log(t):
    w=Rotation.from_matrix(t[:3,:3]).as_rotvec();a=np.linalg.norm(w);s=skew(w)
    v=np.eye(3)+s*.5+s@s*((1/6) if a<1e-6 else (a-np.sin(a))/a**3)
    if a>=1e-6:v=np.eye(3)+s*((1-np.cos(a))/a**2)+s@s*((a-np.sin(a))/a**3)
    return np.r_[w,linalg.solve(v,t[:3,3])]
def adj(t):return np.block([[t[:3,:3],np.zeros((3,3))],[skew(t[:3,3])@t[:3,:3],t[:3,:3]]])
def jr(x):
    ad=np.block([[skew(x[:3]),np.zeros((3,3))],[skew(x[3:]),skew(x[:3])]])
    result=np.eye(6);term=np.eye(6)
    for k in range(1,40):term=term@(-ad)/(k+1);result+=term
    return result
def numerical_chart(d):
    step=1e-6;e=inv(exp(d));out=np.empty((6,6))
    for k in range(6):
        v=np.eye(6)[k]*step;out[:,k]=(log(e@exp(d+v))-log(e@exp(d-v)))/(2*step)
    return out

class Oracle:
    def __init__(self):
        self.prior=None;self.active={};self.theta={};self.epoch=0;self.seen=set();self.absorbed=0;self.removed=0
        self.queries=[];self.retirements=0;self.io=0;self.pending=None;self.failure=None
    def commit(self,s,c):
        assert s['epoch']==c['epoch']==self.epoch+1 and s['gen']==1
        refs=s['refs'];retiring=s['retire'];theta=s['theta']
        if self.prior:
            for k in self.prior[0]:
                assert np.array_equal(self.theta[k][2],theta[k][2]),('prior chart',s['epoch'],k)
                assert k in retiring or k in s['fixed']
        assert set(self.active)-set(refs)==s['removed']
        for i,(ver,slot,f) in refs.items():
            if i in self.active:
                old=self.active[i];assert slot==old[1] and ver>=old[0] and f[0]==old[2][0]
            else:assert i not in self.seen
            assert ver==s['epoch']
            for k,d in zip(f[0],f[1]):assert theta[k][0]==d
        self.seen.update(refs);self.removed+=len(s['removed'])
        incident=[];surviving={}
        for i,r in refs.items():
            if retiring.intersection(r[2][0]):incident.append(r[2]);self.absorbed+=1
            else:surviving[i]=r
        if retiring:
            fs=([self.prior] if self.prior else [])+incident
            allkeys={k for f in fs for k in f[0]};assert retiring<=allkeys
            order=sorted(retiring)+sorted(allkeys-retiring)
            ks,ds,h,_=assemble(fs,order);count=len(retiring);m=sum(ds[:count])
            cross=h[:m,m:];a=h[:m,:m]
            # Dense Schur independently from raw blocks, not candidate priors.
            remaining=h[m:,m:]-cross.T@linalg.solve(a,cross,assume_a='sym')
            self.prior=(ks[count:],ds[count:],remaining);self.retirements+=1
            assert set(self.prior[0])<=s['fixed']
        self.active=surviving;self.theta={k:v for k,v in theta.items() if k not in retiring}
        self.epoch=s['epoch'];self.io=c['io_seconds']
        assert c['counts']==[len(self.seen),len(self.active),self.absorbed,self.removed]
        assert len(self.seen)==len(self.active)+self.absorbed+self.removed
    def query(self,p,q):
        assert p['epoch']==q['epoch']==self.epoch
        ks,ds,h,offset=assemble(([self.prior] if self.prior else [])+[r[2] for r in self.active.values()])
        ck,cd,ch,_=assemble(p['factors'],ks);assert ds==cd
        he=float(linalg.norm(ch-h)/max(1.,linalg.norm(h)));assert he<1e-9,(self.epoch,'H',he)
        h=h[:-1,:-1];eig=linalg.eigvalsh(h);assert eig[0]>0
        idx=[offset[k]+j for k in [p['a'],p['b']] for j in range(6)]
        rhs=np.eye(h.shape[0])[:,idx];expected=linalg.solve(h,rhs,assume_a='sym')[idx,:]
        ce=rel(q['cov'],expected);assert ce<1e-8,(self.epoch,'cov',ce)
        a=linalg.block_diag(jr(p['df']),jr(p['dt']));ae=rel(q['transport'],a);assert ae<1e-8,ae
        numerical=linalg.block_diag(numerical_chart(p['df']),numerical_chart(p['dt']))
        ne=rel(a,numerical);assert ne<1e-6,ne
        current=a@expected@a.T;current_e=rel(q['current'],current);assert current_e<1e-8,current_e
        z=inv(p['pf'])@p['pt'];j=np.column_stack((-adj(inv(z)),np.eye(6)))
        relative=j@current@j.T;re=rel(q['relative'],relative);assert re<1e-8,(self.epoch,'relative',re)
        jn=np.empty((6,12));step=1e-6
        for k in range(12):
            d=np.eye(6)[k%6]*step
            if k<6:pos=inv(z)@inv(p['pf']@exp(d))@p['pt'];neg=inv(z)@inv(p['pf']@exp(-d))@p['pt']
            else:pos=inv(z)@inv(p['pf'])@p['pt']@exp(d);neg=inv(z)@inv(p['pf'])@p['pt']@exp(-d)
            jn[:,k]=(log(pos)-log(neg))/(2*step)
        fd=rel(jn@numerical@expected@numerical.T@jn.T,q['relative']);assert fd<1e-6,fd
        self.queries.append(dict(epoch=self.epoch,dimension=len(h),condition=float(eig[-1]/eig[0]),
          information_relative=he,covariance_relative=ce,transport_relative=ae,current_relative=current_e,
          relative_covariance_relative=re,finite_difference_relative=fd,query_seconds=q['seconds']))

def main():
    p=argparse.ArgumentParser();p.add_argument('journal');p.add_argument('output');a=p.parse_args()
    assert not Path(a.output).exists();o=Oracle();s=None;query=None;error=None;started=time.monotonic()
    try:
        for tag,r in Reader(a.journal).records():
            if tag=='S':
                assert s is None;s=r
            elif tag=='C':o.commit(s,r);s=None
            elif tag=='P':query=r
            elif tag=='Q':o.query(query,r);query=None
            elif tag=='F':o.failure=r;break
        assert o.failure is None,o.failure
        assert s is None and query is None and len(o.queries)>0
    except Exception as exc:error=repr(exc)
    result=dict(passed=error is None,error=error,epochs=o.epoch,retirements=o.retirements,
      admitted=len(o.seen),active=len(o.active),absorbed=o.absorbed,removed=o.removed,queries=o.queries,
      failure=o.failure,pending_epoch=None if s is None else s['epoch'],io_seconds=o.io,
      seconds=time.monotonic()-started,journal_sha256=hashlib.sha256(Path(a.journal).read_bytes()).hexdigest())
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    raise SystemExit(0 if error is None else 1)
if __name__=='__main__':main()
