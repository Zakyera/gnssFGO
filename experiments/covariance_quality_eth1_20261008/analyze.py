"""Read-only numerical analysis of frozen factor journals; never runs an estimator."""
from pathlib import Path
import argparse, csv, hashlib, importlib.util, json, subprocess, sys, time
from collections import defaultdict
sys.dont_write_bytecode = True
import numpy as np
from scipy.stats import rankdata
from scipy.spatial.transform import Rotation

BASE = Path('/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007')
REPO = Path('/home/yeranis/repos/gnssFGO')
REF_HELPER = Path('/home/yeranis/results/gnssfgo_eth1_covariance_audit_20260930/covariance_age_reference.py')
spec = importlib.util.spec_from_file_location('ref_helper', REF_HELPER)
ref = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ref)
SOURCES = ('glim_hesai', 'kimera_front')
PARTS = {'rotation': slice(0, 3), 'rho': slice(3, 6)}

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(2**20), b''): h.update(b)
    return h.hexdigest()

def dump(path, obj):
    with path.open('x') as f: json.dump(obj, f, indent=2, allow_nan=False)

def stats(x):
    x = np.asarray(x, float)
    assert np.isfinite(x).all()
    if not x.size: return {'n': 0}
    return dict(n=int(x.size), median=float(np.median(x)), mean=float(x.mean()),
                rms=float(np.sqrt(np.mean(x*x))), p05=float(np.quantile(x,.05)),
                p95=float(np.quantile(x,.95)), min=float(x.min()), max=float(x.max()))

def corr(x, y):
    x, y = np.asarray(x,float), np.asarray(y,float)
    if len(x)<3 or np.std(x)<1e-14 or np.std(y)<1e-14: return None
    return float(np.corrcoef(x,y)[0,1])

def spearman(x,y): return corr(rankdata(x),rankdata(y))

def ranking(sigma, error, seconds, motion):
    block = (seconds//10).astype(int)
    unique = np.unique(block)
    within_x, within_y = np.empty(len(sigma)), np.empty(len(sigma))
    block_results=[]
    for b in unique:
        sel=block==b; n=int(sel.sum())
        within_x[sel]=(rankdata(sigma[sel])-.5)/n-.5
        within_y[sel]=(rankdata(error[sel])-.5)/n-.5
        block_results.append(dict(block=int(b),n=n,spearman=spearman(sigma[sel],error[sel])))
    # Descriptive partial ranks: motion quantities are held-out reference motion,
    # not a fit that changes any measurement or estimator parameter.
    design=np.column_stack([np.ones(len(sigma)),rankdata(motion[:,0]),rankdata(motion[:,1])])
    rx,ry=rankdata(sigma),rankdata(error)
    px=rx-design@np.linalg.lstsq(design,rx,rcond=None)[0]
    py=ry-design@np.linalg.lstsq(design,ry,rcond=None)[0]
    rng=np.random.default_rng(20261008); samples=[]
    blocks=[np.flatnonzero(block==b) for b in unique]
    for _ in range(500):
        ids=np.concatenate([blocks[b] for b in rng.integers(len(blocks),size=len(blocks))])
        val=spearman(sigma[ids],error[ids])
        if val is not None: samples.append(val)
    bins=[]
    for q, ids in enumerate(np.array_split(np.argsort(sigma,kind='stable'),5)):
        bins.append(dict(quintile=q+1, sigma=stats(sigma[ids]), error=stats(error[ids])))
    return dict(spearman=spearman(sigma,error),within_10s_rank_correlation=corr(within_x,within_y),
                motion_adjusted_rank_correlation=corr(px,py),
                block_resample_95_range=[float(x) for x in np.quantile(samples,[.025,.975])],
                block_count=len(blocks),block_correlations=block_results,quintiles=bins,
                highest_over_lowest_uncertainty_error_rms=bins[4]['error']['rms']/bins[0]['error']['rms'])

def marginal(C,e):
    eig,U=np.linalg.eigh(C)
    assert np.all(eig>0)
    projection=np.einsum('nji,nj->ni',U,e)
    normalized=projection/np.sqrt(eig)
    energy=normalized**2
    directions=[]
    for j in range(3):
        directions.append(dict(eigen_rank=j+1,std=stats(np.sqrt(eig[:,j])),
            absolute_projected_discrepancy=stats(abs(projection[:,j])),
            absolute_normalized_discrepancy=stats(abs(normalized[:,j])),
            fraction_marginal_whitened_energy=stats(energy[:,j]/energy.sum(axis=1))))
    return dict(directions=directions,condition_number=stats(eig[:,-1]/eig[:,0]),
                normalized_energy=stats(energy.sum(axis=1))), energy.sum(axis=1)

def load(mode):
    H=defaultdict(list); last=-1; count=0
    with (BASE/'inputs'/f'{mode}_events.csv').open() as f:
        for row in csv.reader(f):
            assert int(row[1])>=last; last=int(row[1])
            if row[0]!='B': continue
            assert len(row)==52
            source,session=row[2:4]; i,j=int(row[6]),int(row[7]); a,b=int(row[8]),int(row[9])
            assert source in SOURCES and a<b and last>=b
            key=(source,a,b); old=H[key]
            record=dict(arrival=last,revision=int(row[5]),session=session,i=i,j=j,
                        mu=np.array(row[10:16],float),cov=np.array(row[16:52],float).reshape(6,6))
            if old:
                assert (old[-1]['session'],old[-1]['i'],old[-1]['j'])==(session,i,j)
                assert old[-1]['revision']<record['revision']
            old.append(record); count+=1
    accepted={}
    with (BASE/'payloads'/f'{mode}_final.csv').open() as f:
        for row in csv.DictReader(f):
            key=(row['source'],int(row['from_ns']),int(row['to_ns']))
            assert key not in accepted; accepted[key]=row
            latest=H[key][-1]
            assert np.array_equal(latest['mu'],np.array([row[f'mu{i}'] for i in range(6)],float))
            assert np.array_equal(latest['cov'].reshape(-1),np.array([row[f'c{i}'] for i in range(36)],float))
    return {k:H[k] for k in sorted(accepted)},count,H

def selected(hist,age,cut):
    end=hist[0]['arrival'] if age=='first' else cut if age=='cutoff' else hist[0]['arrival']+int(age)*10**9
    return max(i for i,r in enumerate(hist) if r['arrival']<=end)

def revision_summary(entries,source,mode,age,cut,cohort):
    sigmas={p:[] for p in PARTS}; errs={p:[] for p in PARTS}; ratios={p:[] for p in PARTS}
    for k in cohort:
        if k[0]!=source: continue
        h=entries[mode][k]; ix=selected(h['records'],age,cut)
        for part,sl in PARTS.items():
            a=np.sqrt(np.trace(h['C'][0,sl,sl])); b=np.sqrt(np.trace(h['C'][ix,sl,sl]))
            ea=np.linalg.norm(h['E'][0,sl]); eb=np.linalg.norm(h['E'][ix,sl])
            sigmas[part].append(b/a); errs[part].append(eb/ea)
            ratios[part].append((eb/b)/(ea/a))
    result={}
    for part in PARTS:
        s,e=np.array(sigmas[part]),np.array(errs[part]); smaller=s<1-1e-10
        result[part]=dict(n=len(s),sigma_last_over_first=stats(s),error_last_over_first=stats(e),
            normalized_discrepancy_last_over_first=stats(ratios[part]),
            sigma_contracts_fraction=float(np.mean(smaller)),error_improves_fraction=float(np.mean(e<1)),
            contracts_and_error_worsens_fraction=float(np.mean(smaller&(e>1))),
            error_worsens_given_contraction_fraction=float(np.mean(e[smaller]>1)) if smaller.any() else None,
            contracts_and_normalized_discrepancy_worsens_fraction=float(np.mean(smaller&(e>s))),
            error_vs_sigma_logratio_correlation=spearman(np.log(s),np.log(e)))
    return result

def numerical_tests():
    rng=np.random.default_rng(48); mx=0.; white=0.
    for _ in range(100):
        z=rng.normal(size=6)*.1; eps=rng.normal(size=6)*.001
        Rz=Rotation.from_rotvec(z[:3]); tz=ref.v_matrix(z[:3])@z[3:]
        Re=Rotation.from_rotvec(eps[:3]); te=ref.v_matrix(eps[:3])@eps[3:]
        e,_=ref.discrepancy(z,(Rz*Re,tz+Rz.apply(te)))
        mx=max(mx,float(np.max(abs(e-eps))))
        A=rng.normal(size=(6,6)); C=A@A.T+np.eye(6)
        L=np.linalg.cholesky(C); d=e@np.linalg.solve(C,e)
        white=max(white,abs(d-np.sum(np.linalg.solve(L,e)**2)))
        for sl in PARTS.values():
            _,energy=marginal(C[None,sl,sl],e[None,sl])
            assert np.isclose(energy[0],e[sl]@np.linalg.solve(C[sl,sl],e[sl]),rtol=1e-12)
    assert mx<1e-10 and white<1e-12
    assert spearman(np.arange(10),np.arange(10))>.99999
    assert spearman(np.arange(10),-np.arange(10))<-.99999
    return dict(se3_right_perturbation_max=mx,dense_whitening_max=white,synthetic_cases=100)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--out',required=True); args=ap.parse_args()
    out=Path(args.out); out.mkdir(parents=True,exist_ok=False); started=time.monotonic()
    manifest=json.loads((BASE/'manifest.json').read_text()); cut=manifest['cutoff_ns']; start=manifest['first_imu_ns']
    tracked=subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])
    protect=dict(manifest['protected'])
    extra=[REF_HELPER,REF_HELPER.parent/'ie_tc.npz',BASE/'manifest.json',BASE/'belief_comparison.csv',BASE/'belief_summary.json',BASE/'changes.csv',BASE/'publications.csv',BASE/'prediction_summary.json',BASE/'largest_event_beliefs.csv',BASE/'linear_summary.json',Path(__file__),Path(__file__).with_name('PROTOCOL.txt')]
    for m in ('off','on'):
        extra.extend([BASE/'inputs'/f'{m}_events.csv',BASE/'payloads'/f'{m}_final.csv'])
        assert sha(BASE/'inputs'/f'{m}_events.csv')==manifest['inputs'][m]['clipped_sha256']
    for p in extra: protect[str(p)]=sha(p)
    for p,h in protect.items(): assert sha(p)==h,p
    dump(out/'inputs_before.json',protect)
    H={}; all_h={}; counts={}; entries={}; refs={}; health={}
    for m in ('off','on'): H[m],counts[m],all_h[m]=load(m)
    assert H['off'].keys()==H['on'].keys() and len(H['off'])==1770
    common=sorted(H['off']); assert sum(k[0]=='glim_hesai' for k in common)==1189
    refs={k:ref.relative_reference(k[1],k[2]) for k in common}; assert all(r is not None for r in refs.values())
    print('Identity gate passed: 1,189 GLIM and 581 Kimera intervals.',flush=True)
    # Validate ALL input covariances, including 25 startup versions per mode not
    # accepted by the receiver. Numerical analysis uses accepted IDs only.
    for m in H:
        C=np.array([r['cov'] for h in all_h[m].values() for r in h]); eig=np.linalg.eigvalsh(C)
        health[m]=dict(input_versions=counts[m],accepted_versions=sum(map(len,H[m].values())),
            min_eigenvalue=float(eig.min()),max_symmetry=float(np.max(abs(C-C.transpose(0,2,1)))))
        assert np.isfinite(C).all() and eig.min()>0 and health[m]['max_symmetry']<1e-12
        entries[m]={}
        for k,h in H[m].items():
            E=np.array([ref.discrepancy(r['mu'],refs[k])[0] for r in h]); C=np.array([r['cov'] for r in h])
            entries[m][k]=dict(records=h,E=E,C=C)
        print(m,'all accepted revision residuals computed.',flush=True)
    report=dict(scope=dict(start_ns=start,cutoff_ns=cut,matched_intervals=1770,glim=1189,kimera=581,
                           no_estimator_runs=True,no_parameter_changes=True),health=health,groups={},revisions={},consecutive={},events={})
    selected_rows=[]; cohorts={}; old_rows={}
    with (BASE/'belief_comparison.csv').open() as f:
        for r in csv.DictReader(f): old_rows[(r['mode'],r['source'],int(r['from_ns']),int(r['to_ns']),r['age'])]=r
    max_prior_difference=0.
    for age in ('first','1','2','cutoff'):
        cohort=[k for k in common if age in ('first','cutoff') or all(H[m][k][0]['arrival']+int(age)*10**9<=cut for m in H)]
        cohorts[age]=cohort
        for m in H:
            for source in SOURCES:
                keys=[k for k in cohort if k[0]==source]; C=[]; E=[]; motion=[]; seconds=[]
                for k in keys:
                    idx=selected(H[m][k],age,cut); ent=entries[m][k]; c,e=ent['C'][idx],ent['E'][idx]
                    C.append(c);E.append(e);seconds.append((k[2]-start)*1e-9)
                    motion.append([np.linalg.norm(refs[k][1]),np.linalg.norm(refs[k][0].as_rotvec())])
                    row=dict(mode=m,source=source,from_ns=k[1],to_ns=k[2],age=age,
                             revision=H[m][k][idx]['revision'],sigma=[],error=[],dense=float(e@np.linalg.solve(c,e)))
                    old=old_rows[(m,*k,age)]
                    for part,sl in PARTS.items():
                        sigma=np.sqrt(np.trace(c[sl,sl])); err=np.linalg.norm(e[sl]); row['sigma'].append(float(sigma));row['error'].append(float(err))
                        sf='rotation_sigma_root_deg' if part=='rotation' else 'rho_sigma_root_m'
                        ef='rotation_error_deg' if part=='rotation' else 'rho_error_m'
                        unit=180/np.pi if part=='rotation' else 1
                        max_prior_difference=max(max_prior_difference,abs(sigma*unit-float(old[sf])),abs(err*unit-float(old[ef])))
                    selected_rows.append(row)
                C,E,motion,seconds=np.array(C),np.array(E),np.array(motion),np.array(seconds)
                group={}; blockenergy=np.zeros(len(E))
                for part,sl in PARTS.items():
                    sigma=np.sqrt(np.trace(C[:,sl,sl],axis1=1,axis2=2)); err=np.linalg.norm(E[:,sl],axis=1)
                    directional,energy=marginal(C[:,sl,sl],E[:,sl]); blockenergy+=energy
                    group[part]=dict(sigma=stats(sigma),error=stats(err),ranking=ranking(sigma,err,seconds,motion),directional=directional)
                dense=np.einsum('ni,ni->n',E,np.linalg.solve(C,E[...,None])[...,0])
                group['dense_squared_whitened_discrepancy']=stats(dense)
                group['dense_over_block_marginal_energy']=stats(dense/blockenergy)
                group['interval_duration_s']=stats([(k[2]-k[1])*1e-9 for k in keys])
                report['groups'][f'{m}_{source}_{age}']=group
        print('Selected age',age,'complete.',flush=True)
    assert max_prior_difference<1e-12
    for m in H:
        for s in SOURCES:
            for age in ('2','cutoff'):
                report['revisions'][f'{m}_{s}_{age}']=revision_summary(entries,s,m,age,cut,cohorts[age])
            bypart={p:[] for p in PARTS}
            for k,h in entries[m].items():
                if k[0]!=s or len(h['records'])<2: continue
                for part,sl in PARTS.items():
                    sig=np.sqrt(np.trace(h['C'][:,sl,sl],axis1=1,axis2=2)); err=np.linalg.norm(h['E'][:,sl],axis=1)
                    contraction=sig[1:]<sig[:-1]*(1-1e-10)
                    bypart[part].append(float(np.mean(contraction&(err[1:]>err[:-1]*(1+1e-10)))))
            report['consecutive'][f'{m}_{s}']={p:stats(v) for p,v in bypart.items()}
    # Attribute nothing causally: all listed updates occur in the same optimized
    # publication; observed state movement is reused, not an applied new solve.
    pred=json.loads((BASE/'prediction_summary.json').read_text())
    for m in H:
        worst=pred[m]['largest_current_correction']; a,b=worst['snapshot'],worst['next_snapshot']
        ledgers=[]
        for ix in (a,b):
            p=BASE/f'{m}_wnoa'/f'snapshot_{ix}'/'ledger.csv'; protect[str(p)]=sha(p)
            with p.open() as f: ledgers.append({(r['source'],int(r['from_ns']),int(r['to_ns'])):r for r in csv.DictReader(f)})
        event=dict(publication=worst,sources={})
        for s in SOURCES:
            changed=[]
            for k,new in ledgers[1].items():
                old=ledgers[0].get(k)
                if k[0]!=s or (old and old['revision']==new['revision']): continue
                ent=entries[m][k]; ni=next(i for i,r in enumerate(H[m][k]) if r['revision']==int(new['revision']))
                rr=dict(from_ns=k[1],to_ns=k[2],revision=int(new['revision']),kind='revised' if old else 'new')
                for part,sl in PARTS.items():
                    rr[part+'_error']=float(np.linalg.norm(ent['E'][ni,sl])); rr[part+'_sigma']=float(np.sqrt(np.trace(ent['C'][ni,sl,sl])))
                    if old:
                        oi=next(i for i,r in enumerate(H[m][k]) if r['revision']==int(old['revision']))
                        rr[part+'_sigma_ratio']=rr[part+'_sigma']/np.sqrt(np.trace(ent['C'][oi,sl,sl]))
                        rr[part+'_error_ratio']=rr[part+'_error']/np.linalg.norm(ent['E'][oi,sl])
                changed.append(rr)
            event['sources'][s]=dict(changes=changed,summary={})
            for part in PARTS:
                rev=[r for r in changed if r['kind']=='revised']
                event['sources'][s]['summary'][part]=dict(new=sum(r['kind']=='new' for r in changed),revised=len(rev),
                    sigma_ratio=stats([r[part+'_sigma_ratio'] for r in rev]),error_ratio=stats([r[part+'_error_ratio'] for r in rev]))
        report['events'][m]=event
    dump(out/'selected_records.json',selected_rows)
    dump(out/'summary.json',report)
    validation=dict(tests=numerical_tests(),max_prior_summary_difference=max_prior_difference,
                    selected_rows=len(selected_rows),protected_count=len(protect),
                    tracked_diff_unchanged=subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])==tracked,
                    head=subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip())
    assert validation['tracked_diff_unchanged']
    for p,h in protect.items(): assert sha(p)==h,p
    validation['all_protected_hashes_unchanged']=True
    validation['runtime_seconds']=time.monotonic()-started
    dump(out/'inputs_after.json',protect);dump(out/'validation.json',validation)
    print(json.dumps(validation,indent=2),flush=True)

if __name__=='__main__': main()
