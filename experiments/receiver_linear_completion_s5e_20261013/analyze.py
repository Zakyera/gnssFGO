"""Read-only matched physical-time comparison. Original CSVs are never edited."""
import csv
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_linear_completion_s5e_20261013')
S5D=Path('/home/yeranis/results/gnssfgo_publication_s5d_20261013')
REF=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013/full_on1')
spec=importlib.util.spec_from_file_location('s5d_analysis',HERE.parent/'receiver_publication_s5d_20261013/analyze.py')
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
rows,stats,delta=prior.rows,prior.stats,prior.delta

def states(folder,i):
    a=np.genfromtxt(folder/'states.csv',delimiter=',',skip_header=1)
    n=len(a)//(i+1)
    assert a.shape==(n*(i+1),15)
    assert np.array_equal(a[:,0],np.repeat(np.arange(n),i+1))
    assert np.array_equal(a[:,1],np.tile(np.arange(i+1),n))
    return a[:,2:].reshape(n,i+1,13)

def main():
    assert json.loads((ROOT/'full_on.json').read_text())['passed']
    selected=json.loads((ROOT/'select.json').read_text())['indices']
    old={r['index']:r for r in json.loads((S5D/'analysis.json').read_text())['snapshots']}
    result=dict(passed=True,scope='18 one-publication snapshots, not repeated-policy validation',snapshots=[])
    for i in selected:
        folder=ROOT/'full_on'/f's5e_support_{i}';oracle=S5D/'full_on'/f's5d_support_{i}'
        new=states(folder,i);previous=states(oracle,i);m=rows(folder/'frames.csv');s=old[i]
        assert len(new)==2 and len(m)==2
        assert int(m[0]['physical_ns'])==int(rows(oracle/'frames.csv')[0]['physical_ns'])
        assert np.array_equal(new[0],previous[0]),(i,'initial snapshot mismatch')
        assert float(m[0]['normalized_gradient'])==s['initial_gradient'],(i,'clone gradient mismatch')
        assert float(m[0]['error'])==s['initial_objective'],(i,'clone objective mismatch')
        # Every original gradient/diagonal coordinate is matched, not just its maximum.
        original_g=[r for r in rows(oracle/'gradients.csv') if r['frame']=='0']
        new_g=[r for r in rows(folder/'gradients.csv') if r['frame']=='0']
        assert original_g==new_g,(i,'clone full gradient/diagonal mismatch')
        record=dict(index=i,time_s=s['time_s'],ordinary_gradient=float(m[0]['normalized_gradient']),
            full_backsub_gradient=float(m[1]['normalized_gradient']),first_pass_gradient=s['frames'][1]['gradient'],
            final_gradient=s['final_gradient'],ordinary_error=float(m[0]['error']),
            full_backsub_error=float(m[1]['error']),first_pass_error=s['frames'][1]['objective'],final_error=s['final_objective'],
            stationary=float(m[1]['normalized_gradient'])<=1e-5,original_publication_ms=s['publication_ms'],
            copy_ms=1000*float(m[0]['copy_seconds']),full_backsub_ms=1000*float(m[0]['full_solve_seconds']),
            verification_ms=1000*float(m[0]['verification_seconds']))
        record['backsub_objective_fraction']=(record['ordinary_error']-record['full_backsub_error'])/(record['ordinary_error']-record['final_error'])
        for name,a,b in [('ordinary_to_full',new[0],new[1]),('full_to_first',new[1],previous[1]),
                         ('full_to_final',new[1],previous[s['final_frame']])]:
            record[name]={k:dict(max=float(v.max()),rms=float(np.sqrt(np.mean(v*v))),latest=float(v[-1])) for k,v in delta(a,b).items()}
        result['snapshots'].append(record)
        print(i,'g',record['ordinary_gradient'],record['full_backsub_gradient'],'fullsolve_ms',record['full_backsub_ms'])
    snaps=result['snapshots'];result['stationary_count']=sum(s['stationary'] for s in snaps)
    result['summary']={k:stats([s[k] for s in snaps]) for k in ['ordinary_gradient','full_backsub_gradient',
      'backsub_objective_fraction','copy_ms','full_backsub_ms','verification_ms','original_publication_ms']}
    result['maximum_state_differences']={name:{k:max(s[name][k]['max'] for s in snaps) for k in snaps[0][name]} for name in ['ordinary_to_full','full_to_first','full_to_final']}
    with (ROOT/'analysis.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='snapshots'},indent=2))

if __name__=='__main__':main()
