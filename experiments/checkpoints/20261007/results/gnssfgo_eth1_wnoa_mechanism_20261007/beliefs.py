"""Exact-interval, age-matched descriptive source audit; no optimizer."""
from pathlib import Path
import csv,json,importlib.util,sys
from collections import defaultdict
import numpy as np
from scipy.linalg import eigh
sys.dont_write_bytecode=True
from prepare import ROOT
spec=importlib.util.spec_from_file_location('old', '/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/covariance_age_reference.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
def stat(x):
    x=np.asarray(x,float)
    return {'n':int(x.size),'median':float(np.median(x)),'mean':float(np.mean(x)),'rms':float(np.sqrt(np.mean(x*x))),'p95':float(np.quantile(x,.95)),'max':float(np.max(x))} if x.size else {'n':0}
def histories(mode):
    h=defaultdict(list)
    for r in csv.reader((ROOT/'inputs'/f'{mode}_events.csv').open()):
        if r[0]!='B':continue
        key=(r[2],int(r[8]),int(r[9]))
        h[key].append({'arrival':int(r[1]),'revision':int(r[5]),'i':int(r[6]),'j':int(r[7]),'mu':np.array(r[10:16],float),'cov':np.array(r[16:],float).reshape(6,6)})
    return h
if __name__=='__main__':
    manifest=json.loads((ROOT/'manifest.json').read_text());cut=manifest['cutoff_ns'];start=manifest['first_imu_ns']
    H={m:histories(m) for m in ['off','on']}
    # Same support lower boundary in both modes. Final insertion ledger will independently validate the accepted set.
    init=first_support=int(next(r[2] for i,r in enumerate(csv.reader((ROOT/'inputs/off_events.csv').open())) if i==58))
    common=sorted(k for k in H['off'].keys()&H['on'].keys() if k[1]>=init and k[2]<=init+119200000000)
    rows=[];comparisons={};refs={k:old.relative_reference(k[1],k[2]) for k in common}
    assert all(v is not None for v in refs.values())
    for age in ['first','1','2','cutoff']:
        valid=[]
        for key in common:
            if age in ['1','2'] and any(H[m][key][0]['arrival']+int(age)*10**9>cut for m in H):continue
            valid.append(key)
            for mode in H:
                hist=H[mode][key]
                limit=hist[0]['arrival'] if age=='first' else cut if age=='cutoff' else hist[0]['arrival']+int(age)*10**9
                e=next(e for e in reversed(hist) if e['arrival']<=limit)
                error,cart=old.discrepancy(e['mu'],refs[key]);c=e['cov']
                assert np.isfinite(c).all() and np.linalg.eigvalsh(c).min()>0 and np.max(abs(c-c.T))<1e-12
                rows.append(dict(mode=mode,source=key[0],from_ns=key[1],to_ns=key[2],age=age,revision=e['revision'],
                    actual_revision_age_s=(e['arrival']-hist[0]['arrival'])*1e-9,
                    first_delay_s=(hist[0]['arrival']-key[2])*1e-9,
                    rotation_error_deg=np.linalg.norm(error[:3])*180/np.pi,rho_error_m=np.linalg.norm(error[3:]),cart_error_m=cart,
                    rotation_sigma_root_deg=np.sqrt(np.trace(c[:3,:3]))*180/np.pi,rho_sigma_root_m=np.sqrt(np.trace(c[3:,3:])),
                    squared_whitened_reference_discrepancy=float(error@np.linalg.solve(c,error))))
        for source in sorted({k[0] for k in valid}):
            ratios=defaultdict(list)
            for k in valid:
                if k[0]!=source:continue
                chosen={}
                for m in H:
                    h=H[m][k];limit=h[0]['arrival'] if age=='first' else cut if age=='cutoff' else h[0]['arrival']+int(age)*10**9
                    chosen[m]=next(e['cov'] for e in reversed(h) if e['arrival']<=limit)
                for part,sl in [('rotation',slice(0,3)),('rho',slice(3,6))]:
                    a=chosen['off'][sl,sl];b=chosen['on'][sl,sl]
                    ratios[part+'_trace_ratio'].append(np.trace(b)/np.trace(a));ratios[part+'_generalized_eigenvalues'].extend(eigh(b,a,eigvals_only=True))
                ratios['dense_generalized_eigenvalues'].extend(eigh(chosen['on'],chosen['off'],eigvals_only=True))
            comparisons[source+'_'+age]={k:stat(v) for k,v in ratios.items()}
    with (ROOT/'belief_comparison.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    report={'matched_intervals':len(common),'matched_indices':sum((H['off'][k][0]['i'],H['off'][k][0]['j'])==(H['on'][k][0]['i'],H['on'][k][0]['j']) for k in common),
            'unmatched':{m:len(set(H[m])-set(H['on' if m=='off' else 'off'])) for m in H},'summary':{},'covariance_comparison':comparisons}
    fields=['rotation_error_deg','rho_error_m','cart_error_m','rotation_sigma_root_deg','rho_sigma_root_m','squared_whitened_reference_discrepancy','first_delay_s']
    for mode in H:
        for source in sorted({k[0] for k in common}):
            for age in ['first','1','2','cutoff']:
                group=[r for r in rows if r['mode']==mode and r['source']==source and r['age']==age]
                report['summary'][mode+'_'+source+'_'+age]={k:stat([r[k] for r in group]) for k in fields}
    (ROOT/'belief_summary.json').write_text(json.dumps(report,indent=2));print('Matched intervals',len(common),'rows',len(rows))
