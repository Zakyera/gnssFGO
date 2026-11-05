"""Independent saved-output checks and coverage-explicit presentation; no replay."""
import gzip
import importlib.util
import json
import sys
from pathlib import Path
import numpy as np
from scipy.linalg import logm

sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('audit',H/'analyze.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
O=m.OUT
assert not (O/'supplement.json').exists()
audit=json.loads((O/'audit.json').read_text());assert audit['passed']
assert all(m.g.sha(Path(p))==h for p,h in audit['input_sha256'].items())
paths=list(O.glob('*'))+[Path(__file__)]
hashes={str(p):m.g.sha(p) for p in paths if p.is_file()}
records=[json.loads(line) for line in gzip.open(O/'source_samples.jsonl.gz','rt')]
peers=[json.loads(line) for line in gzip.open(O/'peer_samples.jsonl.gz','rt')]
first={(x['mode'],x['source'],x['a'],x['b']):x for x in records if x['stage']=='all_first'}
matched=[first[x['mode'],x['source'],x['a'],x['b']] for x in records if x['stage']=='near4s']
assert len(matched)==11080
assert len({(x['mode'],x['source'],x['a'],x['b']) for x in matched})==11080

# Independently check saved peer residuals with a generic matrix logarithm,
# rather than the bespoke Pose3 log helper used by the main analysis.
g2k=[x for x in peers if x['direction']=='G2K']
ref=m.g.Reference(m.GT)
max_log=0.;max_white=0.
for i in np.linspace(0,len(g2k)-1,81,dtype=int):
    x=g2k[i]
    target=np.linalg.inv(ref.at(x['a']))@ref.at(x['b'])
    L=logm(np.linalg.inv(m.g.exp(x['mu']))@target)
    assert np.max(np.abs(np.imag(L)))<1e-12
    L=np.real(L);e=np.r_[L[2,1],L[0,2],L[1,0],L[:3,3]]
    old=np.r_[x['aligned']['phi'],x['aligned']['rho']]
    max_log=max(max_log,float(np.max(np.abs(e-old))))
    C=np.array(x['cov']);white=np.linalg.solve(np.linalg.cholesky(C),e)
    q=white@white
    max_white=max(max_white,float(abs(q-x['aligned']['joint_normalized_squared'])/(1+q)))
assert max_log<1e-10 and max_white<1e-7

confusion={}
for block,ef,vf in [('translation','rho_error_mm','translation_trace_m2'),('rotation','rotation_error_deg','rotation_trace_rad2')]:
    ae=np.array([x['aligned'][ef] for x in g2k]);pe=np.array([x['verified_pre'][ef] for x in g2k])
    lower=np.array([x['aligned'][vf]<x['verified_pre'][vf] for x in g2k]);better=ae<pe
    confusion[block]=dict(n=len(g2k),choose_glim_correct=int((lower&better).sum()),
        choose_glim_wrong=int((lower&~better).sum()),choose_kimera_correct=int((~lower&~better).sum()),
        choose_kimera_wrong=int((~lower&better).sum()),always_kimera_correct_fraction=float((~better).mean()))

adverse=sorted(g2k,key=lambda x:x['post_update']['rotation_error_deg']-x['verified_pre']['rotation_error_deg'],reverse=True)[:5]
examples=[{k:x[k] for k in ('cur','ri','rj','time_s','source_event_line','aligned','verified_pre','post_update','closer_to_incoming_after_update')}
          for x in adverse]

# Unlike the initial overview, never connect a line across empty matched bins.
# That omission means no matching physical OFF/ON intervals, not missing camera data.
plt=m.plt
fig,axs=plt.subplots(2,2,figsize=(13,8.5))
colors={'off':'#176db5','on':'#dc6b16'}
bin_counts={}
for row,source in enumerate(m.SOURCES):
    bin_counts[source]={}
    for mode in m.MODES:
        xs=[x for x in matched if x['source']==source and x['mode']==mode]
        values=[];counts=[]
        for t in range(0,430,10):
            rr=[x for x in xs if t<=x['time_s']<t+10];counts.append(len(rr))
            values.append((t+5,np.sqrt(np.mean([x['metric']['translation_trace_m2'] for x in rr]))*1000 if rr else np.nan,
                           np.sqrt(np.mean([x['metric']['rho_error_mm']**2 for x in rr])) if rr else np.nan))
        bin_counts[source][mode]=counts
        assert sum(counts)==len(xs)
        v=np.array(values)
        for col in (0,1):axs[row,col].plot(v[:,0],v[:,col+1],'-o',ms=2.5,color=colors[mode],label=mode.upper())
    assert bin_counts[source]['off']==bin_counts[source]['on']
    for col in (0,1):
        ax=axs[row,col]
        for b,n in enumerate(bin_counts[source]['off']):
            if n==0:ax.axvspan(b*10,(b+1)*10,color='#dddddd',alpha=.6)
        ax.set_title(('GLIM' if row==0 else 'Kimera')+(' — reported norm scale' if col==0 else ' — actual relative-error RMS'))
        ax.set_yscale('log');ax.set_xlim(0,430);ax.set_xlabel('Seconds after capture start')
        ax.set_ylabel('mm');ax.grid(alpha=.2);ax.legend()
fig.suptitle('LEICA-2: first-belief translation uncertainty versus error (10 s bins)',fontsize=15)
fig.text(.04,.025,'Grey = no eligible matched endpoints; no interpolation across gaps. Same OFF/ON cohort. Short relative errors, not ATE.',fontsize=10)
fig.tight_layout(rect=(0,.055,1,.96))
assert not (O/'translation_time_coverage.png').exists()
fig.savefig(O/'translation_time_coverage.png',dpi=160);plt.close(fig)

assert all(m.g.sha(Path(p))==h for p,h in hashes.items())
assert all(m.g.sha(Path(p))==h for p,h in audit['input_sha256'].items())
m.save(O/'supplement.json',dict(passed=True,generic_matrix_log_samples=81,max_error_vector_difference=max_log,
    max_whitened_q_relative_difference=max_white,selection_confusion=confusion,
    largest_adverse_rotation_update_examples=examples,matched_first_10s_bin_counts=bin_counts,
    presentation_correction='Use translation_time_coverage.png; translation_time.png connects empty bins and is superseded.',
    hashes_verified=hashes,all_original_inputs_unchanged=True))
print(json.dumps(dict(passed=True,max_log_difference=max_log,max_q_relative_difference=max_white,confusion=confusion),indent=2))
