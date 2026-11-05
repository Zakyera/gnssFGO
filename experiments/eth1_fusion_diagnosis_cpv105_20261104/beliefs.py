"""Actual received/absorbed beliefs; diagnostics only, no receiver optimization."""
import csv
import datetime
import hashlib
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np
from scipy.linalg import eigvalsh

sys.dont_write_bytecode=True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOTS={'OFF':W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared',
       'ON':W/'runs/eth1_prior_fix_live_cpv103_20261103/prepared'}
PINS={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):p=Path(p);PINS[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def stats(x):
    x=np.asarray(x,float);assert x.size and np.isfinite(x).all()
    return dict(n=len(x),min=float(x.min()),median=float(np.median(x)),mean=float(x.mean()),
                p95=float(np.percentile(x,95)),max=float(x.max()))
spec=importlib.util.spec_from_file_location('cp105_geometry',pin(E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'))
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
epoch=read(ROOTS['ON']/'input_audit.json')['epoch_ns']
ref=g.Reference(pin(W/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag'))
roles=('glim_hesai','kimera_front')
ages=(2.5,4.5);stages=('first','age2.5','age4.5','terminal')
allrecords={};account={}
for mode,P in ROOTS.items():
    base=P/('full_'+mode.lower()+'_receiver')
    data={s:{role:{} for role in roles} for s in stages};ids={};counts={r:0 for r in roles}
    # Last-accepted belief at absorption, not the last arriving expired revision.
    terminals={}
    for role in roles:
        for path in sorted((base/'receiver/archive/ids'/role).glob('*/*')):
            f=pin(path).read_text().split();assert len(f)==59,(path,len(f))
            revision,ordinal,start,end,arrival,slot=map(int,f[1:7])
            T=np.array(f[7:23],float).reshape(4,4);C=np.array(f[23:],float).reshape(6,6)
            key=(start,end);assert key not in terminals.setdefault(role,{})
            terminals[role][key]=dict(start=start,end=end,arrival=arrival,revision=revision,ordinal=ordinal,
                mu=g.log(T),C=C,T=T,commit=int(f[0]),id=path.name)
    terminal_matches={r:0 for r in roles}
    with pin(base/'events.csv').open() as stream:
        for f in csv.reader(stream):
            if f[0]!='B':continue
            assert len(f)==52
            role=f[2];start,end=int(f[8]),int(f[9]);arrival=int(f[1]);revision=int(f[5]);ordinal=int(f[4]);key=(start,end)
            bid=(f[6],f[7]);prior=ids.setdefault((role,key),bid);assert prior==bid
            rec=dict(start=start,end=end,arrival=arrival,revision=revision,ordinal=ordinal,
                     mu=np.array(f[10:16],float),C=np.array(f[16:],float).reshape(6,6))
            counts[role]+=1
            if key not in data['first'][role]:data['first'][role][key]=rec
            for age in ages:
                if arrival-end<=round(age*1e9):data['age'+str(age)][role][key]=rec
            term=terminals[role].get(key)
            if term and revision==term['revision']:
                assert arrival==term['arrival'] and ordinal==term['ordinal']
                assert np.max(abs(rec['C']-term['C']))==0
                assert np.max(abs(g.exp(rec['mu'])-term['T']))<1e-11
                terminal_matches[role]+=1
    for role in roles:
        assert terminal_matches[role]==len(terminals[role])
        data['terminal'][role]=terminals[role]
    allrecords[mode]=data
    account[mode]=dict(received_versions=counts,terminal_markers={r:len(terminals[r]) for r in roles},
                       terminal_payload_matches=terminal_matches,unique_edges={r:len(data['first'][r]) for r in roles})

def summarize(err,C,arrival_age):
    q=np.einsum('ni,ni->n',err,np.linalg.solve(C,err[...,None])[...,0])
    qc=np.sum(np.linalg.solve(np.linalg.cholesky(C),err[...,None])[...,0]**2,axis=1)
    assert np.max(abs(q-qc)/(1+q))<1e-8
    out=dict(n=len(C),full6_normalized_reference_discrepancy=stats(q),arrival_age_s=stats(arrival_age))
    for name,sl in (('rotation',slice(0,3)),('translation',slice(3,6))):
        e=err[:,sl];c=C[:,sl,sl];tr=np.trace(c,axis1=1,axis2=2);sq=np.sum(e*e,axis=1)
        qb=np.einsum('ni,ni->n',e,np.linalg.solve(c,e[...,None])[...,0])
        out[name]=dict(trace=stats(tr),mean_squared_tangent_reference_discrepancy=float(sq.mean()),
             RMS_tangent_reference_discrepancy=float(np.sqrt(sq.mean())),
             RMS_reported_norm_uncertainty=float(np.sqrt(tr.mean())),
             squared_discrepancy_over_mean_trace=float(sq.mean()/tr.mean()),
             normalized_reference_discrepancy=stats(qb),fraction_inside_nominal_95percent=float(np.mean(qb<=7.814727903251179)))
    return out

output={};arrays={};cohorts={}
for role in roles:
    common=set(allrecords['OFF']['first'][role])&set(allrecords['ON']['first'][role])
    eligible={k for k in common if 3<=(k[1]-epoch)*1e-9<=300 and .18<=(k[1]-k[0])*1e-9<=.22}
    cohort=eligible.copy()
    for mode in ROOTS:
        for stage in stages:
            table=allrecords[mode][stage][role]
            cohort&=set(table)
            if stage.startswith('age'):
                age=float(stage[3:]);cohort={k for k in cohort if (table[k]['arrival']-k[1])*1e-9>=age-.35}
    keys=sorted(cohort);assert len(keys)>1000,(role,len(keys))
    cohorts[role]=dict(common_physical_edges=len(common),eligible_short_edges_3_to_300s=len(eligible),
                       all_stage_same_edge_cohort=len(keys),excluded=len(eligible)-len(keys),
                       first_endpoint_s=(keys[0][0]-epoch)*1e-9,last_endpoint_s=(keys[-1][1]-epoch)*1e-9)
    reference=[g.inv(ref.at(a))@ref.at(b) for a,b in keys]
    times=np.array([(b-epoch)*1e-9 for a,b in keys]);arrays[role+'_end_s']=times
    per={};cs={};es={}
    for mode in ROOTS:
        per[mode]={}
        for stage in stages:
            records=[allrecords[mode][stage][role][k] for k in keys]
            C=np.array([z['C'] for z in records]);ev=np.linalg.eigvalsh(C)
            assert ev.min()>0 and np.max(abs(C-C.transpose(0,2,1)))<1e-12
            err=np.array([g.log(g.inv(g.exp(z['mu']))@target) for z,target in zip(records,reference)])
            age=np.array([(z['arrival']-z['end'])*1e-9 for z in records])
            per[mode][stage]=summarize(err,C,age);cs[mode,stage]=C;es[mode,stage]=err
            for suffix,a in (('covariance',C),('residual',err),('arrival_age',age)):
                arrays[role+'_'+mode+'_'+stage+'_'+suffix]=a
    ratios={}
    for stage in stages:
        ratios[stage]={}
        for name,sl in (('rotation',slice(0,3)),('translation',slice(3,6))):
            traces={m:np.trace(cs[m,stage][:,sl,sl],axis1=1,axis2=2) for m in ROOTS}
            ratio=traces['ON']/traces['OFF']
            ratios[stage][name]=dict(paired_trace_ratio_ON_over_OFF=stats(ratio),
                fraction_below_half=float(np.mean(ratio<.5)),fraction_below_tenth=float(np.mean(ratio<.1)),
                aggregate_mean_trace_ratio=float(traces['ON'].mean()/traces['OFF'].mean()),
                aggregate_squared_discrepancy_ratio=float(np.mean(np.sum(es['ON',stage][:,sl]**2,axis=1))/np.mean(np.sum(es['OFF',stage][:,sl]**2,axis=1))))
        values=np.array([eigvalsh(on,off) for on,off in zip(cs['ON',stage],cs['OFF',stage])])
        ratios[stage]['generalized_covariance_eigenvalues_ON_over_OFF']=stats(values.ravel())
    bins=[]
    for lo in range(0,300,30):
        ix=(times>=lo)&(times<lo+30)
        if not ix.any():continue
        row=dict(start_s=lo,end_s=lo+30,n=int(ix.sum()))
        for mode in ROOTS:
            row[mode]=summarize(es[mode,'terminal'][ix],cs[mode,'terminal'][ix],arrays[role+'_'+mode+'_terminal_arrival_age'][ix])
        bins.append(row)
    output[role]=dict(summary=per,paired=ratios,terminal_time_bins=bins)

for path,digest in PINS.items():assert sha(path)==digest,path
np.savez_compressed(H/'belief_cohorts.npz',**arrays)
save('belief_analysis.json',dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),
     accounting=account,cohorts=cohorts,sources=output,reference_max_bracket_ns=ref.max_bracket_ns,
     method='Actual receiver events and terminal accepted markers; exact physical endpoints, unchanged right-local CPT7 6x6 covariances. Fixed receiver-age2.5/4.5s, last arrival within0.35s of cutoff, same cohort at every stage.',
     metric='Tangent residual Log(inv(Zbelief)*Z_IE); normalized reference discrepancies, NOT true NEES/NIS or trajectory RPE. No optimization.',
     input_sha256=PINS))
print(json.dumps(dict(cohorts=cohorts,paired={r:output[r]['paired'] for r in roles},
     summaries={r:output[r]['summary'] for r in roles}),indent=2))
