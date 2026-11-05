"""Offline numerical-audit summaries; no factor or solver modification."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from scipy.linalg import svd
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_paired_receiver_trace_cpv22a_20261027')
def sha(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def summary(x):
    x=np.asarray(x,float)
    return {'n':len(x),'max':float(x.max()),'median':float(np.median(x)),'p95':float(np.percentile(x,95))}
out={'cases':{},'input_sha256':{}}
def pin(p):out['input_sha256'][str(p)]=sha(p);return p
for name in ('hesai','velodyne'):
    assert json.loads(pin(H/f'{name}_parity.json').read_text())['pass_']
    T=R/f'{name}_trace'
    with pin(T/'jacobians.csv').open() as f: rows=list(csv.DictReader(f))
    groups={}
    for cat in sorted({r['category'] for r in rows}):
        sel=[r for r in rows if r['category']==cat]
        groups[cat]={'factors':len({r['snapshot'] for r in sel}),'block_step_comparisons':len(sel),
                     'failed_blocks':sum(r['pass']!='1' for r in sel),
                     'max_absolute':summary([float(r['max_absolute']) for r in sel]),
                     'max_scaled':summary([float(r['max_scaled']) for r in sel]),
                     'residual_with_jacobian_difference':max(float(r['residual_with_jacobian_difference']) for r in sel)}
    with pin(T/'endpoint_checks.csv').open() as f: ends=list(csv.DictReader(f))
    endpoints={cat:{'factors':len(rr),'max_reconstructed_residual_difference':max(float(r['reconstructed_residual_difference']) for r in rr),
                    'max_boundary_pose_difference':max(float(r['boundary_pose_difference']) for r in rr),
                    'start_tau_s':summary([float(r['start_tau']) for r in rr]),
                    'end_tau_s':summary([float(r['end_tau']) for r in rr])}
               for cat in sorted({r['source'] for r in ends})
               for rr in [[r for r in ends if r['source']==cat]]}
    with pin(T/'linearizations.csv').open() as f: graphs=list(csv.DictReader(f))
    linear=[]
    for g in graphs:
        m,n=int(g['rows']),int(g['cols'])
        ab=np.fromfile(pin(T/g['rows_file']),dtype=np.float64).reshape(m,n+1)
        A,b=ab[:,:n],ab[:,n]
        with pin(T/g['columns_file']).open() as f: cols=list(csv.DictReader(f))
        dims=np.array([int(c['dimension']) for c in cols]);assert dims.sum()==n
        norms=np.linalg.norm(A,axis=0);assert (norms>0).all()
        normalized=A/norms
        u,s,vh=svd(normalized,full_matrices=False,check_finite=True,lapack_driver='gesdd')
        # Singular-value cutoff is only for diagnostics, not a change to a solve.
        tol=max(A.shape)*np.finfo(float).eps*s[0]
        rank=int((s>tol).sum())
        projected=u[:,:rank].T@b
        gn_scaled=vh[:rank].T@(projected/s[:rank])
        gn=gn_scaled/norms
        rawgrad=A.T@b;scaledgrad=normalized.T@b
        shifts=[];cursor=0
        for c in cols:
            dim=int(c['dimension']);block=gn[cursor:cursor+dim];cursor+=dim
            if c['key'].startswith('x'):shifts.append({'key':c['key'],'rot_norm':float(np.linalg.norm(block[:3])),'trans_norm':float(np.linalg.norm(block[3:]))})
        newest=max(shifts,key=lambda x:int(x['key'][1:]))
        linear.append({'newest':int(g['newest']),'snapshot':int(g['snapshot']),'event':int(g['event']),
                       'rows':m,'cols':n,'column_normalized_rank':rank,'rank_cutoff':float(tol),
                       'column_normalized_condition':float(s[0]/s[-1]),'singular_min':float(s[-1]),
                       'cost':float(g['cost']),'linearized_cost_at_zero':float(b@b/2),
                       'raw_gradient_inf':float(np.max(np.abs(rawgrad))),
                       'column_normalized_gradient_inf':float(np.max(np.abs(scaledgrad))),
                       'linearized_optimal_cost_reduction':float(projected@projected/2),
                       'newest_linear_diagnostic_increment':newest,
                       'largest_linear_pose_translation':max(shifts,key=lambda x:x['trans_norm'])})
    case={'jacobians':groups,'endpoints':endpoints,'linearizations':linear,
          'all_jacobian_checks_pass':all(g['failed_blocks']==0 for g in groups.values()),
          'all_endpoint_checks_pass':all(g['max_reconstructed_residual_difference']<1e-10 and g['max_boundary_pose_difference']<1e-10 for g in endpoints.values())}
    out['cases'][name]=case
    print(name,json.dumps(case,indent=2),flush=True)
out['interpretation']='Frozen current-chart linearized diagnostics only. Least-squares increments are not applied; no extra nonlinear solve or accepted candidate. Rank/conditioning is coordinate-normalized, not a universal certainty metric.'
pin(Path(__file__))
assert all(sha(Path(p))==h for p,h in out['input_sha256'].items())
with (H/'numerics.json').open('x') as f:json.dump(out,f,indent=2)
