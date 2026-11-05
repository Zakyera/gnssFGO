"""Read-only analysis of saved graphs and matrices; no estimator invocation."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import re
import numpy as np

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/gnssFGO')
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
EPOCH=1727775007855396309
SOURCES={
 'old':W/'runs/eth1_split_imu_persistent_on_cpv91_20261102_attempt1/prepared/on_source',
 'new':W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared/full_on_source'}
TERMINALS=[R/'experiments/eth1_split_imu_persistent_on_cpv91_20261102_attempt1/complete.json',
 R/'experiments/eth1_split_imu_live_pair_cpv95_20261103_continuation1/complete.json']
D96=R/'experiments/eth1_glim_on_regression_cpv96_20261103'
PINS={}
ANSI=re.compile(r'\x1b\[[0-9;]*m')
assert not (H/'analysis.json').exists(), 'Never overwrite prior analysis'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def pin(p):
    p=Path(p);PINS[str(p)]=sha(p);return p
def save(n,obj):
    with (H/n).open('x') as f:json.dump(obj,f,indent=2,allow_nan=False);f.write('\n')
def fields(line,tag):
    return ANSI.sub('',line.split(tag+',',1)[1]).strip().split(',')
def mat(token):return np.array([[float(x) for x in r.split(':')] for r in token.split(';')])
def trace(C):return dict(rotation_rad2=float(np.trace(C[:3,:3])),translation_m2=float(np.trace(C[3:,3:])))
pin(__file__);pin(H/'PROTOCOL.txt')
events=json.loads(pin(D96/'belief_events.json').read_text())
saved={};report={}
for label,S in SOURCES.items():
    first={};counts={};stamps={};retire=[];repeated_first_labels=[]
    for line in pin(S/'estimators.log').open():
        if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
            a=fields(line,'KIMERA_CBS_REVISION_EDGE_ROW');assert len(a)==64
            pair=tuple(map(int,a[2:4]));stamps[pair[1]]=float(a[5])-EPOCH*1e-9
            if a[7]=='first':
                if pair in first:
                    repeated_first_labels.append(dict(pair=list(pair),update=int(a[0])))
                else:
                    first[pair]=dict(update=int(a[0]),time_s=stamps[pair[1]],C=np.array(a[28:],float).reshape(6,6))
        elif 'KIMERA_CBS_OUTGOING_TIMING_ROW,' in line:
            a=fields(line,'KIMERA_CBS_OUTGOING_TIMING_ROW');counts[int(a[0])]=int(a[5])
        elif 'KIMERA_CBS_REVISION_POSE_RETIRE_ROW,' in line:
            a=fields(line,'KIMERA_CBS_REVISION_POSE_RETIRE_ROW');retire.append((int(a[0]),int(a[1])))
    node=next(p for p in S.glob('ros_logs/*/kimera_vio_ros-*-stdout.log') if p.parent.name!='latest')
    samples={};numeric=[];query_modes=Counter()
    for line in pin(node).open():
        if 'CBS_COVARIANCE_EXCHANGE_' in line:
            tag=re.search(r'CBS_COVARIANCE_EXCHANGE_\w+_ROW',line)
            if not tag:continue
            tag=tag.group();a=fields(line,tag);i=int(a[7])
            if i not in (0,1):continue
            q=samples.setdefault(i,dict(pair=[int(a[5][1:]),int(a[6][1:])],groups={},active=[],relative={},matrix={}))
            if tag.endswith('GLOBAL_FACTOR_GROUP_ROW'):
                gkey=a[8]+'|'+a[9]
                assert gkey not in q['groups']
                q['groups'][gkey]=dict(category=a[8],type=a[9],count=int(a[10]),error_sum=float(a[11]),hessian_rows=int(a[12]),
                 hessian_trace=float(a[13]),hessian_norm=float(a[14]),eta_norm=float(a[15]))
            elif tag.endswith('ACTIVE_FACTOR_ROW'):
                q['active'].append(dict(slot=int(a[8]),category=a[9],touch_from=int(a[10]),touch_to=int(a[11]),
                  keys=a[12],type=a[13],error=float(a[14]),endpoint_H_norm=float(a[15])))
            elif tag.endswith('RECEIVER_RELATIVE_ROW'):
                q['relative'][a[8]]=dict(trace=float(a[9]),rotation_trace=float(a[10]),translation_trace=float(a[11]),
                  schur_direct_norm=float(a[12]),jitter_used=int(a[13]),jitter=float(a[14]),status=a[15])
            elif tag.endswith('MATRIX_ROW'):
                M=mat(a[12]);assert M.shape==(int(a[10]),int(a[11]));q['matrix'].setdefault(a[8],{})[a[9]]=M
        elif 'CBS_ODOM_RELATIVE_COVARIANCE_ROW,' in line:
            a=fields(line,'CBS_ODOM_RELATIVE_COVARIANCE_ROW')
            assert len(a)==25,len(a)
            numeric.append(dict(pair=a[3:5],mode=a[5],trace=float(a[7]),schur_direct_norm=float(a[19]),
                min_eigen=float(a[11]),jitter_used=int(a[20]),jitter=float(a[21]),status=a[24]))
        elif 'CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,' in line:
            a=fields(line,'CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW');assert len(a)==4;query_modes.update([a[1]])
    sample_report={}
    for i,q in samples.items():
        pairs=tuple(q['pair']);fr=first[pairs];st={}
        for stage,m in q['matrix'].items():
            if stage not in q['relative']:continue
            J=m['lambda_y_joint_information_12x12'];SY=m['sigma_y_joint_covariance_12x12']
            Hbetween=np.hstack([m['H_from_between_jacobian_6x6'],m['H_to_between_jacobian_6x6']])
            direct=Hbetween@np.linalg.solve(J,Hbetween.T)
            C=m['sigma_rel_schur_covariance_6x6'];den=np.linalg.norm(C)
            err=float(np.linalg.norm(direct-C)/den)
            assert err<1e-7,(label,i,stage,err)
            assert np.linalg.eigvalsh((C+C.T)/2)[0]>0
            st[stage]=dict(**q['relative'][stage],independent_propagation_relative_error=err,
                saved_direct_relative_error=float(np.linalg.norm(C-m['sigma_rel_direct_full12x12_propagated_6x6'])/den),
                joint_inverse_residual=float(np.linalg.norm(J@SY-np.eye(12))),
                covariance_min_eigenvalue=float(np.linalg.eigvalsh(C)[0]),
                covariance_condition=float(np.linalg.cond(C)),joint_information_condition=float(np.linalg.cond(J)),
                versus_first_outgoing_relative_error=float(np.linalg.norm(C-fr['C'])/np.linalg.norm(fr['C'])))
        localstage='receiver_preinjection_excluding_explicit_external_factors'
        assert st[localstage]['versus_first_outgoing_relative_error']<1e-8
        aa=[a for a in q['active'] if a['category']=='smart_stereo']
        support=Counter(a['keys'] for a in aa)
        sample_report[i]=dict(pair=q['pair'],time_s=fr['time_s'],global_groups=q['groups'],stages=st,
            endpoint_smart=dict(count=len(aa),zero_Hessian_count=sum(a['endpoint_H_norm']==0 for a in aa),
             nonzero_Hessian_count=sum(a['endpoint_H_norm']>0 for a in aa),
             key_support_multiset=dict(support)),
            first_outgoing_native=trace(fr['C']))
    saved[label]=dict(first=first,counts=counts,stamps=stamps,samples=samples)
    report[label]=dict(samples=sample_report,repeated_first_labels=repeated_first_labels,
        first_retirement=dict(update=retire[0][0],retired_pose=retire[0][1],sequence_time_s=stamps[retire[0][0]]),
        outgoing_numeric=dict(count=len(numeric),modes=dict(Counter(n['mode'] for n in numeric)),
          status_counts=dict(Counter(n['status'] for n in numeric)),jitter_count=sum(n['jitter_used'] for n in numeric),
          minimum_covariance_eigenvalue=min(n['min_eigen'] for n in numeric),
          max_schur_direct_norm_over_trace=max(n['schur_direct_norm']/n['trace'] for n in numeric)),
        joint_marginal_query_modes=dict(query_modes))

common=sorted(set(saved['old']['first'])&set(saved['new']['first']))
early=[]
for pair in common:
    a=saved['old']['first'][pair];b=saved['new']['first'][pair]
    if a['time_s']>12:continue
    assert a['time_s']==b['time_s']
    ra=trace(a['C']);rb=trace(b['C'])
    e=dict(pair=list(pair),time_s=a['time_s'],old_native=ra,new_native=rb,
         translation_trace_ratio_new_old=rb['translation_m2']/ra['translation_m2'],
         covariance_relative_difference=float(np.linalg.norm(b['C']-a['C'])/np.linalg.norm(a['C'])))
    matched=[z for z in events if z['source_ids']==list(pair)]
    if matched:
        assert len(matched)==1
        z=matched[0];e['inserted_in_glim']={lab:dict(translation_trace_m2=z[lab]['trace_translation_m2'],
             receiver_ids=z[lab]['local_ids']) for lab in ('old','new')}
    early.append(e)
count_differences=[]
for k,ca in sorted(saved['old']['counts'].items()):
    if k not in saved['new']['counts']:continue
    cb=saved['new']['counts'][k];t=saved['old']['stamps'].get(k)
    if ca!=cb:count_differences.append(dict(update=k,time_s=t,old=ca,new=cb))
report['timeline']=dict(early_covariance=early,first_native_covariance_difference=next(e for e in early if e['covariance_relative_difference']>1e-10),
    first_native_translation_trace_below_half=next(e for e in early if e['translation_trace_ratio_new_old']<.5),
    first_local_graph_size_difference=count_differences[0],early_local_graph_size_differences=[e for e in count_differences if e['time_s'] is not None and e['time_s']<12])
q0=saved['old']['samples'][0];q1=saved['new']['samples'][0]
assert q0['groups']==q1['groups'] and q0['relative']==q1['relative']
support_old=Counter(report['old']['samples'][1]['endpoint_smart']['key_support_multiset'])
support_new=Counter(report['new']['samples'][1]['endpoint_smart']['key_support_multiset'])
report['support_multiset_sample1']=dict(shared_min_count=sum((support_old&support_new).values()),
    old_unmatched_count=sum((support_old-support_new).values()),new_unmatched_count=sum((support_new-support_old).values()),
    caution='These are pose-key support multisets, NOT landmark identities or exact measurement matches.')
report['limitations']=[
 'The first LOCAL graph-size difference is not automatically a live factor-count difference; graph size may include empty slots.',
 'The detailed audit DOES independently count live factor types at sample1; all prior retirement has not yet started.',
 'LOCAL excludes explicit peer factors but uses CBS-influenced state estimates to linearize nonlinear native factors.',
 'The complete original smart-factor observations, landmark identities and synchronized frontend bias-use trace are unavailable here.',
 'No fixed-factor/fixed-linearization counterfactual was run. Visual Hessian trace ratios alone cannot predict relative covariance ratios.',
 'No claim that lower covariance is calibrated or that this explains the whole-run GLIM ATE change.']

# Verify terminal output hashes without invoking their controllers or validators.
terminal_checks=[]
for p in TERMINALS:
    d=json.loads(pin(p).read_text())
    hashes=d.get('output_sha256',d.get('outputs_sha256'))
    assert isinstance(hashes,dict),list(d)
    for fn,digest in hashes.items():assert sha(fn)==digest,fn
    terminal_checks.append(dict(path=str(p),verified_outputs=len(hashes)))
for n in ['verification.json','supplement_verification.json','covariance_timeline_verification.json']:
    d=json.loads(pin(D96/n).read_text())
    for fn,digest in d['output_sha256'].items():assert sha(fn)==digest,fn
for p in [D96/'REPORT.txt',W/'src/cbs/src/bpsam/bpsam.cpp',
 W/'runs/retention_readiness_cpv73_20261031/VioBackend.cpp',
 W/'src/Kimera-VIO/src/pipeline/StereoImuPipeline.cpp',
 W/'src/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp',
 W/'src/Kimera-VIO/include/kimera-vio/frontend/VisionImuFrontend.h']:
    pin(p)
for p,digest in PINS.items():assert sha(p)==digest,p
save('analysis.json',report);save('inputs_sha256.json',PINS)
save('verification.json',dict(passed=True,source_runs=0,receiver_runs=0,optimizer_runs=0,
    terminal_checks=terminal_checks,cp96_output_manifests_verified=3,
    input_count=len(PINS),output_sha256={str(H/n):sha(H/n) for n in ['analysis.json','inputs_sha256.json']}))
print(json.dumps(dict(timeline={k:v for k,v in report['timeline'].items() if not k.startswith('early')},
    old=report['old'],new=report['new'],support=report['support_multiset_sample1']),indent=2))
