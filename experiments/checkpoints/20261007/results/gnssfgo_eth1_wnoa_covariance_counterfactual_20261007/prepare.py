from pathlib import Path
import csv,json,hashlib,subprocess
import numpy as np
ROOT=Path(__file__).resolve().parent
BASE=Path('/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007')
REPO=Path('/home/yeranis/repos/gnssFGO')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()
def rows(p):return list(csv.DictReader(p.open()))
if __name__=='__main__':
    old=json.loads((BASE/'manifest.json').read_text());assert json.loads((BASE/'audit_validation.json').read_text())['passed']
    protected=old['protected'].copy()
    for p,h in protected.items():assert sha(p)==h,p
    for mode in ['off','on']:
        for p in (BASE/(mode+'_wnoa/snapshot_final')).iterdir():
            if p.is_file():protected[str(p)]=sha(p)
        p=BASE/'payloads'/f'{mode}_final.csv';protected[str(p)]=sha(p)
    for p in [BASE/'metrics.json',BASE/'audit_validation.json',Path('/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007/audit.cpp'),ROOT/'PROTOCOL.json']:
        protected[str(p)]=sha(p)
    payloads={m:rows(BASE/'payloads'/f'{m}_final.csv') for m in ['off','on']}
    by={m:{(r['source'],r['from_ns'],r['to_ns']):r for r in rr} for m,rr in payloads.items()}
    assert set(by['off'])==set(by['on']) and len(by['on'])==1770
    led={m:{(r['source'],r['from_ns'],r['to_ns']):r for r in rows(BASE/(m+'_wnoa/snapshot_final/ledger.csv'))} for m in by}
    donors=[];mapping=[];counts={};min_eig=1.;max_sym=0.
    for k,target in by['on'].items():
        donor=by['off'][k];a,b=led['on'][k],led['off'][k]
        assert (a['from_index'],a['to_index'])==(b['from_index'],b['to_index'])
        assert a['slot']==target['slot'] and b['slot']==donor['slot']
        assert int(a['arrival_ns'])<=old['cutoff_ns'] and int(b['arrival_ns'])<=old['cutoff_ns']
        C=np.array([donor[f'c{i}'] for i in range(36)],float).reshape(6,6)
        eig=np.linalg.eigvalsh(C);assert np.isfinite(C).all() and eig.min()>0
        sym=float(np.max(abs(C-C.T)));assert sym<1e-12
        min_eig=min(min_eig,float(eig.min()));max_sym=max(max_sym,sym)
        donors.append(dict(slot=target['slot'],**{field:donor[field] for field in ['source','from_ns','to_ns',*[f'c{i}' for i in range(36)]]}))
        mapping.append(dict(source=k[0],from_ns=k[1],to_ns=k[2],on_slot=target['slot'],off_slot=donor['slot'],
                            from_index=a['from_index'],to_index=a['to_index'],on_revision=a['revision'],off_revision=b['revision'],
                            on_availability_ns=a['arrival_ns'],off_availability_ns=b['arrival_ns']))
        counts[k[0]]=counts.get(k[0],0)+1
    for name,rr in [('donors.csv',donors),('mapping.csv',mapping)]:
        with (ROOT/name).open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rr[0]));w.writeheader();w.writerows(rr)
        protected[str(ROOT/name)]=sha(ROOT/name)
    assert counts=={'glim_hesai':1189,'kimera_front':581}
    r={'protected':protected,'matched':len(donors),'counts':counts,'min_donor_eigenvalue':min_eig,'max_donor_symmetry_error':max_sym,
       'head':subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip(),
       'tracked_diff_sha256':hashlib.sha256(subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])).hexdigest()}
    with (ROOT/'manifest.json').open('x') as f:json.dump(r,f,indent=2)
    print(json.dumps({k:v for k,v in r.items() if k!='protected'},indent=2))
