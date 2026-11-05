"""Locate matched-event covariance changes without changing either source run."""
import json
from pathlib import Path
import analyze as a
import numpy as np
H=a.H
assert not (H/'covariance_timeline.json').exists()
a.pin(__file__)
events=json.loads(a.pin(H/'belief_events.json').read_text())
g2k={}
for label,folder in a.SOURCES.items():
    rr=[]
    for line in a.pin(folder/'estimators.log').open():
        if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' not in line:continue
        f=a.ANSI.sub('',line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1]).strip().split(',')
        assert len(f)==13
        C=a.g.matrix(f[12])
        rr.append(dict(update=int(f[0]),source_ids=list(map(int,f[1:3])),local_ids=list(map(int,f[3:5])),
            source_ns=list(map(int,f[5:7])),local_ns=list(map(int,f[7:9])),causal_ns=int(f[9]),
            raw=np.asarray(f[10].split(';'),float),C=C))
    assert len(rr)==1473
    g2k[label]=rr
paired=[]
for old,new in zip(g2k['old'],g2k['new']):
    assert old['source_ids']==new['source_ids'] and old['local_ids']==new['local_ids']
    assert old['source_ns']==new['source_ns'] and old['local_ns']==new['local_ns']
    p=dict(source_ids=old['source_ids'],receiver_ids=old['local_ids'],
        source_time_s=(old['source_ns'][1]-a.EPOCH)*1e-9,receiver_time_s=(old['local_ns'][1]-a.EPOCH)*1e-9,
        raw_log_difference=(new['raw']-old['raw']).tolist(),covariance_relative_difference=float(np.linalg.norm(new['C']-old['C'])/np.linalg.norm(old['C'])))
    for lab,z in [('old',old),('new',new)]:
        p[lab]=dict(update=z['update'],trace_rotation_rad2=float(np.trace(z['C'][:3,:3])),trace_translation_m2=float(np.trace(z['C'][3:,3:])))
    paired.append(p)
kt=np.asarray([e['source_times_s'][1] for e in events]);gt=np.asarray([e['receiver_time_s'] for e in paired])
kt_ratio=np.asarray([e['new']['trace_translation_m2']/e['old']['trace_translation_m2'] for e in events])
early=[]
for i,e in enumerate(events):
    if kt[i]>20:break
    early.append(dict(source_ids=e['source_ids'],source_time_s=float(kt[i]),old_local_ids=e['old']['local_ids'],new_local_ids=e['new']['local_ids'],
        new_over_old_translation_trace=float(kt_ratio[i]),
        old_translation_trace_m2=e['old']['trace_translation_m2'],new_translation_trace_m2=e['new']['trace_translation_m2'],
        old_aligned_translation_error_m=e['old']['aligned_error']['translation_m'],new_aligned_translation_error_m=e['new']['aligned_error']['translation_m'],
        old_pre_post_translation_log_m=[e['old']['merge']['values'][i] for i in [1,3]],
        new_pre_post_translation_log_m=[e['new']['merge']['values'][i] for i in [1,3]]))
result=dict(passed=True,g2k=paired,k2g_early=early,
    k2g_below_half_trace_count=int(np.sum(kt_ratio<.5)),k2g_below_tenth_trace_count=int(np.sum(kt_ratio<.1)),
    first_k2g_trace_difference=next(e for e in early if abs(e['new_over_old_translation_trace']-1)>1e-10),
    first_g2k_difference=next(e for e in paired if e['covariance_relative_difference']>1e-10),
    caveat='Event covariance changes are observed source outputs, not an insertion multiplier change. Temporal order does not prove causation, calibration or the cause of ATE regression.')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(3,1,figsize=(11,8))
for lab,col in [('old','#4477aa'),('new','#cc6677')]:
    ax[0].plot(kt,[e[lab]['trace_translation_m2'] for e in events],color=col,label=lab+' K→G')
    ax[2].plot(kt,[e[lab]['aligned_error']['translation_m']*100 for e in events],color=col,label=lab+' K→G')
ax[0].set_yscale('log');ax[0].set_ylabel('Translation covariance trace (m²)');ax[0].set_xlim(3.5,20)
ax[1].plot(kt,kt_ratio,color='#228833');ax[1].axhline(1,color='black',lw=.8);ax[1].set_yscale('log');ax[1].set_ylabel('New / old trace');ax[1].set_xlim(0,305)
ax[2].set_xlim(3.5,20);ax[2].set_ylabel('Aligned belief translation error (cm)')
for aa in ax:aa.set_xlabel('Sequence time (s)');aa.grid(alpha=.2)
ax[0].legend();ax[2].legend();fig.suptitle('Matched Kimera → GLIM beliefs: full-run medians hide early covariance differences')
fig.tight_layout();fig.savefig(H/'covariance_timeline.png',dpi=150);plt.close(fig)
for p,d in a.PINS.items():assert a.sha(p)==d,p
a.save('covariance_timeline.json',result);a.save('covariance_timeline_inputs.json',a.PINS)
a.save('covariance_timeline_verification.json',dict(passed=True,source_runs=0,receiver_runs=0,
    output_sha256={str(H/n):a.sha(H/n) for n in ['covariance_timeline.json','covariance_timeline_inputs.json','covariance_timeline.png']}))
print(json.dumps({k:v for k,v in result.items() if k not in ['g2k','k2g_early']},indent=2))
