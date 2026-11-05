"""Independent saved-matrix checks and scientific plots; no source execution."""
import hashlib
import json
import os
import re
from pathlib import Path
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
J = json.loads((H/'analysis.json').read_text())
Z = np.load(H/'matrices.npz')
assert not (H/'verification.json').exists()


def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def trace(C, block):
    return sum(C[:,j,j] for j in range(block*3,block*3+3))


checks = {}; events = {}
for d, data in J['directions'].items():
    C = Z[d+'_C_ON']; O = Z[d+'_C_OFF']; ii = Z[d+'_matched']; t = Z[d+'_time_s']
    L = np.linalg.cholesky(C)
    ch = float(np.max(abs(L@L.transpose(0,2,1)-C)))
    assert ch < 1e-18
    for label, A in [('applied',C),('matched_OFF_export',O),('matched_ON_inserted',C[ii])]:
        for block,key in enumerate(['rotation_trace_rad2','translation_trace_m2']):
            assert np.isclose(np.median(trace(A,block)),data[label][key]['median'],rtol=1e-14,atol=0)
            assert np.isclose(np.sum(np.linalg.eigvalsh(A[:,block*3:block*3+3,block*3:block*3+3]),axis=1),
                              trace(A,block),rtol=1e-12,atol=1e-22).all()
    # Re-read the original factor-matrix log independently of analyze.py.
    name = 'kimera_vio_ros-' if d=='G2K' else 'glim_rosnode-1-stdout.log'
    path = next(Path(p) for p in J['input_sha256'] if name in Path(p).name and p.endswith('stdout.log'))
    raw = {}
    for line in path.open():
        if 'CBS_ODOM_FACTOR_MATRIX_ROW,' not in line: continue
        f = re.sub(r'\x1b\[[0-9;]*m','',line.split('CBS_ODOM_FACTOR_MATRIX_ROW,')[1]).strip().split(',')
        assert f[0]==d and f[3]=='persistent_odom_applied'
        key = (int(f[1][1:]),int(f[2][1:])); assert key not in raw
        raw[key] = np.fromstring(f[4].replace(';',':'),sep=':').reshape(6,6)
    assert len(raw)==len(C)==1473
    assert all(np.array_equal(raw[tuple(r['receiver_ids'])],C[i]) for i,r in enumerate(data['records']))
    effpath = next(p for p in J['input_sha256'] if p.endswith('/effective_mode_check.json'))
    eff = json.loads(Path(effpath).read_text())
    assert float(eff['glim']['glim_cbs_k2g_odom_factor_covariance_scale'])==1
    assert float(eff['kimera']['cbs_l2k_odom_factor_covariance_scale'])==1
    assert eff['glim']['glim_cbs_health_aware_enable'] is False
    assert eff['kimera']['cbs_health_aware_enable']=='false'
    checks[d] = dict(factor_rows_verified=len(C),cholesky_reconstruction_max=ch,
                     raw_factor_log_matches_npz=True,diagonal_trace_eigen_sum_pass=True,scale_one=True)
    q = trace(C,1)[ii]/trace(O,1)
    low = np.flatnonzero(q<.1)
    events[d] = dict(translation_ON_OFF_below_0p1_count=len(low),
        cases=[dict(receiver_end_s=float(t[ii[j]]),ratio=float(q[j]),
                    ON_trace_m2=float(trace(C,1)[ii[j]]),OFF_trace_m2=float(trace(O,1)[j]),
                    receiver_ids=data['records'][ii[j]]['receiver_ids'],
                    source_ns=data['records'][ii[j]]['source_ns']) for j in low])

plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
fig,axes=plt.subplots(2,2,figsize=(13,7),sharex=True,layout='constrained')
for col,(d,label) in enumerate([('G2K','GLIM → Kimera · Alpha body'),('K2G','Kimera → GLIM · CPT7 body')]):
    C=Z[d+'_C_ON'];O=Z[d+'_C_OFF'];ii=Z[d+'_matched'];t=Z[d+'_time_s']
    for row,(unit,scale) in enumerate([('Rotation trace (rad²)',1),('Translation trace (m²)',1)]):
        ax=axes[row,col]
        # Scatter avoids falsely connecting missing/repeated intervals.
        ax.scatter(t[ii],trace(O,row)*scale,s=5,color='#888888',alpha=.5,label='OFF exports · same source intervals')
        ax.scatter(t,trace(C,row)*scale,s=4,color='#137fc4',alpha=.65,label='ON · actually inserted')
        ax.set_yscale('log');ax.set_ylabel(unit);ax.set_xlim(0,346.32)
        ax.axvspan(302.1,346.32,color='#dddddd',alpha=.4)
        if row==0:ax.set_title(label);ax.legend(fontsize=8,loc='best')
        else:ax.set_xlabel('Receiver interval endpoint · seconds from ETH1 start')
fig.suptitle('ETH1 split IMU · external factor covariance at insertion\n1,473 factors per direction; OFF is an export control, not an insertion',fontsize=13)
fig.savefig(H/'inserted_covariance_traces.png',dpi=170);plt.close(fig)

fig,axes=plt.subplots(2,1,figsize=(13,6),sharex=True,layout='constrained')
for ax,(d,label) in zip(axes,[('G2K','GLIM → Kimera'),('K2G','Kimera → GLIM')]):
    C=Z[d+'_C_ON'];O=Z[d+'_C_OFF'];ii=Z[d+'_matched'];t=Z[d+'_time_s'][ii]
    for block,color,label2 in [(0,'#9b4aab','Rotation'),(1,'#d78620','Translation')]:
        q=trace(C,block)[ii]/trace(O,block)
        ax.scatter(t,q,s=5,color=color,alpha=.5,label=label2)
    ax.axhline(1,color='black',lw=.8);ax.axhline(.1,color='red',lw=.7,ls='--')
    ax.set_yscale('log');ax.set_ylabel('ON / OFF trace');ax.set_title(label);ax.legend(loc='best')
    ax.set_xlim(0,346.32);ax.axvspan(302.1,346.32,color='#ddd',alpha=.4)
axes[-1].set_xlabel('Receiver interval endpoint · seconds from ETH1 start')
fig.suptitle('Matched source intervals · values below 1 mean smaller ON covariance\nA decrease alone does not establish overconfidence',fontsize=13)
fig.savefig(H/'inserted_covariance_ratios.png',dpi=170);plt.close(fig)

for p,d in J['input_sha256'].items():assert sha(p)==d,p
terminal={}
for name in ['eth1_alpha_bias_rw_full_off_cpv89_20261102','eth1_split_imu_persistent_on_cpv91_20261102_attempt1']:
    c=json.loads((H.parent/name/'complete.json').read_text())
    for p,d in c['output_sha256'].items():assert sha(p)==d,p
    terminal[name]=len(c['output_sha256'])
out=dict(passed=True,checks=checks,local_trace_decreases=events,
         input_hashes_verified=len(J['input_sha256']),terminal_hash_counts=terminal,
         output_sha256={str(H/n):sha(H/n) for n in ['analysis.json','matrices.npz','inserted_covariance_traces.png','inserted_covariance_ratios.png']},
         verifier_sha256=sha(__file__))
with (H/'verification.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
print(json.dumps(out,indent=2))
