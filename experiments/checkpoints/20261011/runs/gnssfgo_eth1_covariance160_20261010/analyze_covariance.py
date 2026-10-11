"""Read-only covariance telemetry summaries; not a calibrated influence score.

Compare exact physical intervals and equal availability age, not row number.
Each ID contributes once per age. No optimization, covariance editing or tuning.
"""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
from collections import defaultdict
import numpy as np

ROOT=Path(__file__).resolve().parent
RUN=ROOT/'prepared'
tool=Path('/home/yeranis/repos/gnssFGO/experiments/covariance_interaction_eth1_160_preparation_20261010/inspect_capture.py')
spec=importlib.util.spec_from_file_location('frozen_inventory',tool)
iv=importlib.util.module_from_spec(spec);spec.loader.exec_module(iv)

def stats(c):
    v=iv.covariance_stats(c)
    for label,b in [('phi',c[:3,:3]),('rho',c[3:,3:])]:
        for i,x in enumerate(np.sqrt(np.diag(b))):v[label+'_std_'+str(i)]=float(x)
        for i,x in enumerate(np.linalg.eigvalsh(b)):v[label+'_marginal_eigenvalue_'+str(i)]=float(x)
    # Dimensionless cross-correlation, not a mixed-unit 6D eigenvalue ranking.
    a=np.linalg.cholesky(c[:3,:3]);b=np.linalg.cholesky(c[3:,3:])
    cross=np.linalg.solve(a,c[:3,3:])
    corr=np.linalg.solve(b,cross.T).T
    v['phi_rho_normalized_cross_singular_max']=float(np.linalg.svd(corr,compute_uv=False).max())
    return v

def summarize(rows):
    if not rows:return {'count':0}
    return {k:{**iv.distribution([r[k] for r in rows]),'mean':float(np.mean([r[k] for r in rows]))} for k in rows[0]}

def main():
    out=ROOT/'covariance_analysis';out.mkdir(exist_ok=False)
    histories={}; hashes={}; last_imu={}
    for mode in ['off','on']:
        assert json.loads((RUN/('full160_'+mode+'_validation.json')).read_text())['passed']
        journal=RUN/('full160_'+mode+'_receiver/events.csv')
        hashes[str(journal)]=iv.sha(journal)
        h=defaultdict(list)
        with journal.open() as f:
            for r in csv.reader(f):
                if r[0]=='I':last_imu[mode]=int(r[2]);continue
                if r[0]!='B':continue
                key=(r[2],int(r[8]),int(r[9]))
                cov=np.asarray(r[16:52],float).reshape(6,6)
                h[key].append(dict(arrival=int(r[1]),revision=int(r[5]),cov=cov,
                                   source_i=int(r[6]),source_j=int(r[7])))
        histories[mode]=h
    first=min(k[1] for h in histories.values() for k in h)
    selected={}; distributions={}; per_id=[]
    labels=['first','age1','age2','age3','age4','last_observed']
    for mode,h in histories.items():
        selected[mode]={};groups=defaultdict(list)
        for key,versions in h.items():
            sid,ti,tj=key
            assert all(a['arrival']<=b['arrival'] for a,b in zip(versions,versions[1:]))
            censored=(last_imu[mode]-tj)<5_000_000_000
            picks={'first':versions[0],'last_observed':versions[-1]}
            for age in [1,2,3,4]:
                candidates=[v for v in versions if v['arrival']<=tj+age*1_000_000_000]
                if candidates:picks['age'+str(age)]=candidates[-1]
            for label,v in picks.items():
                s=stats(v['cov']);selected[mode][(key,label)]=s
                groups[(sid,label,'all')].append(s)
                groups[(sid,label,'bin'+str(int((tj-first)//10_000_000_000)*10))].append(s)
                per_id.append(dict(mode=mode,source=sid,ti_ns=ti,tj_ns=tj,selection=label,
                    revision=v['revision'],availability_age_s=(v['arrival']-tj)*1e-9,
                    end_within5s_of_capture=censored,**s))
        distributions[mode]={'/'.join(k):summarize(v) for k,v in groups.items()}
    matches={}; ratio_rows=[]
    common=set(histories['off'])&set(histories['on'])
    for sid in ['glim_hesai','kimera_front']:
        for label in labels:
            ratios=[]
            for key in sorted(common):
                if key[0]!=sid:continue
                a=selected['off'].get((key,label));b=selected['on'].get((key,label))
                if a is None or b is None:continue
                r={k:b[k]/a[k] for k in ['rotation_trace_rad2','rho_trace_m2']}
                ratios.append(r);ratio_rows.append(dict(source=sid,ti_ns=key[1],tj_ns=key[2],selection=label,**r))
            matches[sid+'/'+label]=summarize(ratios)
    def save_rows(path,rows):
        assert rows
        with path.open('x') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    save_rows(out/'per_identity_age.csv',per_id)
    save_rows(out/'matched_on_over_off.csv',ratio_rows)
    summary=dict(passed=True,inputs_sha256=hashes,by_source_and_age=distributions,
       matched_ON_over_OFF=matches,
       interval_counts={s:dict(off=sum(k[0]==s for k in histories['off']),on=sum(k[0]==s for k in histories['on']),
                              exact_common=sum(k[0]==s for k in common)) for s in ['glim_hesai','kimera_front']},
       interpretation=['Every covariance is checked SPD/finite/symmetric using unchanged preparation tool.',
         'These are full rolling revisions transported to fuser, not classic source-to-source wire covariances.',
         'Fixed age is availability age measured from physical endpoint; last observed can be censored.',
         'Trace in rad^2 and m^2 kept separate. Covariance magnitude alone is NOT optimization influence or accuracy.',
         'No health calibration, missing bridge uncertainty correction, independence assumption or tuning introduced.'])
    for path,digest in hashes.items():assert iv.sha(Path(path))==digest
    with (out/'summary.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
    print(json.dumps({'passed':True,'interval_counts':summary['interval_counts'],'first_ratios':{
        k:v for k,v in matches.items() if k.endswith('/first')}},indent=2))

if __name__=='__main__':main()
