from pathlib import Path
import csv, hashlib, json, subprocess
import numpy as np

ROOT = Path(__file__).resolve().parent
PRIOR = Path('/home/yeranis/results/gnssfgo_eth1_wnoa_covariance_counterfactual_20261007')
BASE = Path('/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007')
REPO = Path('/home/yeranis/repos/gnssFGO')
CASES = ['glim_off_cov', 'kimera_off_cov']

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(2**20), b''): h.update(b)
    return h.hexdigest()

def rows(p):
    with Path(p).open() as f: return list(csv.DictReader(f))

def git_state():
    return dict(head=subprocess.check_output(['git','-C',str(REPO),'rev-parse','HEAD'],text=True).strip(),
                tracked_diff_sha256=hashlib.sha256(subprocess.check_output(['git','-C',str(REPO),'diff','HEAD','--binary'])).hexdigest())

if __name__ == '__main__':
    prior = json.loads((PRIOR/'manifest.json').read_text())
    assert json.loads((PRIOR/'validation.json').read_text())['passed']
    assert not (ROOT/'manifest.json').exists()
    protected = prior['protected'].copy()
    for p,h in protected.items(): assert sha(p) == h,p
    for p in PRIOR.rglob('*'):
        if p.is_file(): protected[str(p)] = sha(p)
    for p in ROOT.iterdir():
        if p.is_file(): protected[str(p)] = sha(p)
    donors=rows(PRIOR/'donors.csv'); mapping=rows(PRIOR/'mapping.csv')
    payload={r['slot']:r for r in rows(BASE/'payloads/on_final.csv')}
    assert len(donors)==len(mapping)==len(payload)==1770
    counts={}; eigmin=1.; symmax=0.
    for d,m in zip(donors,mapping):
        p=payload[d['slot']]
        assert (d['slot'],d['source'],d['from_ns'],d['to_ns']) == (m['on_slot'],m['source'],m['from_ns'],m['to_ns'])
        assert all(d[k]==p[k] for k in ['slot','source','from_ns','to_ns'])
        assert max(int(m['on_availability_ns']),int(m['off_availability_ns']))<=1727775127854656555
        c=np.array([d[f'c{i}'] for i in range(36)],float).reshape(6,6)
        e=np.linalg.eigvalsh(c).min(); s=np.max(abs(c-c.T))
        assert np.isfinite(c).all() and e>0 and s<1e-12
        eigmin=min(eigmin,float(e)); symmax=max(symmax,float(s))
        counts[d['source']]=counts.get(d['source'],0)+1
    assert counts=={'glim_hesai':1189,'kimera_front':581}
    state=git_state()
    assert all(state[k]==prior[k] for k in state)
    result=dict(protected=protected,counts=counts,matched=1770,min_donor_eigenvalue=eigmin,max_donor_symmetry_error=symmax,**state)
    with (ROOT/'manifest.json').open('x') as f: json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='protected'},indent=2))
