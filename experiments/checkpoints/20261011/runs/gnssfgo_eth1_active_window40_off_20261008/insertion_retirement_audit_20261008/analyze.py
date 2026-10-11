"""Frozen-data G->K insertion/retirement diagnosis, no estimator execution."""
import csv
import hashlib
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, '/home/yeranis/repos/gnssFGO/tools')
from eig2_single_source_preservation import exp_se3, log_se3, inverse, read_online_csv
from eig2_kimera_revision_export import parse_log

ROOT = Path(__file__).resolve().parent
OFF = ROOT.parent
ON = OFF.parent/'gnssfgo_eth1_active_window40_prepared_20261008'
T0 = 1727775008.003475


def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''): h.update(b)
    return h.hexdigest()


def summary(values):
    a = np.asarray(values, float)
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                rms=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(a,95)),
                max=float(a.max()), min=float(a.min())) if len(a) else dict(count=0)


def pose_error(estimate, truth):
    return dict(t_m=float(np.linalg.norm(estimate[:3,3]-truth[:3,3])),
                r_deg=float(np.degrees(np.linalg.norm(log_se3(inverse(truth)@estimate)[:3]))))


def write_json(name, value):
    with (ROOT/name).open('x') as f: json.dump(value, f, indent=2, allow_nan=False)


def write_csv(name, rows):
    with (ROOT/name).open('x',newline='') as f:
        w=csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def parse_snapshots(path):
    history, retire = parse_log(path)
    snapshots = defaultdict(dict)
    times = {}
    for (i,j), versions in history.items():
        for v in versions:
            u=v['update']
            for k,stamp,p in [(i,v['ti'],v['from_pose']),(j,v['tj'],v['to_pose'])]:
                if k in snapshots[u]: assert np.max(np.abs(snapshots[u][k]-p)) < 1e-10
                snapshots[u][k]=p; times[k]=stamp
    return snapshots,times,history,retire


def parse_lifecycle(path):
    inserts={}; removals=[]; stages={}; pre={}; post={}; mats={}; life={}; u=None
    for ln,line in enumerate(path.open(errors='replace'),1):
        if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,' in line:
            v=line.split('CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,')[1].strip().split(',')
            u=int(float(v[1]));life[u]=dict(line=ln,lag=float(v[2]),active_keys=int(v[3]),marginal_keys=int(v[4]),active_factors_before=int(v[5]),removal_count=int(v[6]))
        for tag,dest in [('CBS_ODOM_PREINJECTION_RESIDUAL_ROW,',pre),('CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW,',post)]:
            if tag in line:
                v=line.split(tag)[1].strip().split(','); edge=(int(v[3][1:]),int(v[4][1:]));dest[(u,*edge)]=dict(line=ln,source=v[6],norm=float(v[7]),rotation=float(v[8]),rho=float(v[9]))
        if 'CBS_ODOM_FACTOR_MATRIX_ROW,' in line:
            v=line.split('CBS_ODOM_FACTOR_MATRIX_ROW,')[1].strip().split(',')
            edge=(int(v[1][1:]),int(v[2][1:]));mats[(u,*edge)]=np.array([[float(x) for x in row.split(':')] for row in v[4].split(';')])
        if 'CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,' not in line: continue
        v=line.split('CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW,')[1].strip().split(',')
        e=re.fullmatch(r'x(\d+)->x(\d+)',v[2])
        if not e:continue
        edge=tuple(map(int,e.groups()))
        if v[3]=='active_window_stage_insert':stages[edge]=dict(update=u,line=ln)
        if v[3]=='active_window_slot_tracked':
            assert edge not in inserts
            inserts[edge]=dict(update=u,line=ln,slot=int(v[4].split('=')[1]))
        if v[3]=='active_window_remove_scheduled':removals.append(dict(update=u,line=ln,i=edge[0],j=edge[1],slot=int(v[4].split('=')[1])))
    return inserts,removals,stages,pre,post,mats,life


def main():
    assert not (ROOT/'summary.json').exists(), 'Do not overwrite retained diagnosis'
    logs={m:b/('pilot_'+m+'_source')/'estimators.log' for m,b in [('off',OFF),('on',ON)]}
    klog,=list((ON/'pilot_on_source/ros_logs').glob('*/*kimera*stdout.log'))
    inputs=[*logs.values(),klog,ROOT/'wire_beliefs.json',ROOT/'reference.npz']
    odom={m:read_online_csv(b/('pilot_'+m+'_source')/'kimera_odometry.csv') for m,b in [('off',OFF),('on',ON)]}
    inputs += [b/('pilot_'+m+'_source')/'kimera_odometry.csv' for m,b in [('off',OFF),('on',ON)]]
    hashes={str(p):sha(p) for p in inputs}
    on,kt,kh,ret=parse_snapshots(logs['on']); off,ot,oh,oret=parse_snapshots(logs['off'])
    assert kt==ot
    ins,rem,stages,pre,post,mats,life=parse_lifecycle(klog)
    wire=json.loads((ROOT/'wire_beliefs.json').read_text())
    beliefs=defaultdict(list)
    for b in wire['beliefs']:beliefs[(b['i'],b['j'])].append(b)
    matches={}
    for ln,line in enumerate(logs['on'].open(errors='replace'),1):
        if 'CBS_ODOM_MATCH_ROW_G2K,' not in line:continue
        v=line.split('CBS_ODOM_MATCH_ROW_G2K,')[1].strip().split(',')
        if v[11]!='sender_odom_queued':continue
        sender=tuple(map(int,re.fullmatch(r'p:g:(\d+)->p:g:(\d+)',v[0]).groups()))
        receiver=tuple(map(int,re.fullmatch(r'p:k:(\d+)->p:k:(\d+)',v[3]).groups()))
        assert receiver not in matches
        matches[receiver]=dict(line=ln, sender=sender,ti=float(v[1]),tj=float(v[2]),ri=float(v[4]),rj=float(v[5]),from_abs_dt=float(v[6]),to_abs_dt=float(v[7]),tolerance=float(v[8]))
    assert set(matches)==set(ins) and len(ins)==175 and len(rem)==156
    ref=np.load(ROOT/'reference.npz'); rt=ref['times'];rp=ref['positions'];rq=ref['quaternions']
    slerp=Slerp(rt-T0,Rotation.from_quat(rq))
    def ref_pose(t):
        index=np.searchsorted(rt,t)
        assert 0<index<len(rt) and rt[index]-rt[index-1]<.006
        a=(t-rt[index-1])/(rt[index]-rt[index-1]);T=np.eye(4)
        T[:3,:3]=slerp([t-T0]).as_matrix()[0];T[:3,3]=(1-a)*rp[index-1]+a*rp[index]
        return T
    def gt(i,j):return inverse(ref_pose(kt[i]))@ref_pose(kt[j])
    # Previous active snapshots are the pre-update calculateEstimate() values;
    # verify against logged residuals before using them as a before/after trace.
    events=[];trace=[]
    for (i,j),ev in sorted(ins.items(),key=lambda kv:kv[1]['update']):
        u=ev['update'];match=matches[(i,j)];candidates=beliefs[match['sender']]
        assert len(candidates)==1 and candidates[0]['revision']==0
        b=candidates[0];Z=exp_se3(np.array(b['mu']));C=np.array(b['covariance']).reshape(6,6)
        assert abs(b['ti']-match['ti'])<1e-6 and abs(b['tj']-match['tj'])<1e-6
        assert np.max(np.abs(C-mats[(u,i,j)]))<1e-15
        assert i in on[u-1] and j in on[u-1] and i in on[u] and j in on[u]
        before=inverse(on[u-1][i])@on[u-1][j];after=inverse(on[u][i])@on[u][j]
        before_res=log_se3(inverse(Z)@before);after_res=log_se3(inverse(Z)@after)
        logged_pre=pre[(u,i,j)];logged_post=post[(u,i,j)]
        precheck=max(abs(np.linalg.norm(before_res)-logged_pre['norm']),abs(np.linalg.norm(before_res[:3])-logged_pre['rotation']),abs(np.linalg.norm(before_res[3:])-logged_pre['rho']))
        postcheck=max(abs(np.linalg.norm(after_res)-logged_post['norm']),abs(np.linalg.norm(after_res[:3])-logged_post['rotation']),abs(np.linalg.norm(after_res[3:])-logged_post['rho']))
        assert precheck<1e-7,(u,precheck)
        receiver_truth=gt(i,j);sender_truth=inverse(ref_pose(b['ti']))@ref_pose(b['tj'])
        eb=pose_error(before,receiver_truth);ea=pose_error(after,receiver_truth)
        es=pose_error(Z,sender_truth);er=pose_error(Z,receiver_truth);tim=pose_error(sender_truth,receiver_truth)
        off_before=inverse(off[u-1][i])@off[u-1][j];off_after=inverse(off[u][i])@off[u][j]
        ob=pose_error(off_before,receiver_truth);oa=pose_error(off_after,receiver_truth)
        body_error=log_se3(inverse(Z)@receiver_truth)
        stateoff=log_se3(inverse(off_before)@before)
        row=dict(update=u,time_s=odom['on'][u][0]*1e-9-T0,i=i,j=j,sender_i=b['i'],sender_j=b['j'],
                 sender_start_s=b['ti']-T0,sender_end_s=b['tj']-T0,receiver_start_s=kt[i]-T0,receiver_end_s=kt[j]-T0,
                 from_offset_ms=1000*(b['ti']-kt[i]),to_offset_ms=1000*(b['tj']-kt[j]),
                 wire_arrival_s=b['record_ns']*1e-9-T0,slot=ev['slot'],
                 sender_own_interval_t_m=es['t_m'],sender_own_interval_r_deg=es['r_deg'],
                 sender_on_receiver_interval_t_m=er['t_m'],sender_on_receiver_interval_r_deg=er['r_deg'],
                 timing_only_t_m=tim['t_m'],timing_only_r_deg=tim['r_deg'],
                 kimera_pre_t_m=eb['t_m'],kimera_post_t_m=ea['t_m'],kimera_pre_r_deg=eb['r_deg'],kimera_post_r_deg=ea['r_deg'],
                 change_t_m=ea['t_m']-eb['t_m'],change_r_deg=ea['r_deg']-eb['r_deg'],
                 off_pre_t_m=ob['t_m'],off_post_t_m=oa['t_m'],off_pre_r_deg=ob['r_deg'],off_post_r_deg=oa['r_deg'],
                 off_change_t_m=oa['t_m']-ob['t_m'],off_change_r_deg=oa['r_deg']-ob['r_deg'],
                 pre_off_on_relative_diff_t_m=float(np.linalg.norm(stateoff[3:])),
                 pre_off_on_relative_diff_r_deg=float(np.degrees(np.linalg.norm(stateoff[:3]))),
                 pre_residual_check=precheck,post_final_residual_check=postcheck,
                 pre_sender_residual_rotation_deg=float(np.degrees(np.linalg.norm(before_res[:3]))),
                 post_sender_residual_rotation_deg=float(np.degrees(np.linalg.norm(after_res[:3]))),
                 pre_sender_residual_rho_m=float(np.linalg.norm(before_res[3:])),post_sender_residual_rho_m=float(np.linalg.norm(after_res[3:])),
                 rot_cov_sqrt_trace_deg=float(np.degrees(np.sqrt(np.trace(C[:3,:3])))),
                 rho_cov_sqrt_trace_m=float(np.sqrt(np.trace(C[3:,3:]))),
                 receiver_gt_whitened_squared=float(body_error@np.linalg.solve(C,body_error)),
                 removals_same_update=life[u]['removal_count'],main_log_line=match['line'],stage_log_line=stages[(i,j)]['line'])
        events.append(row)
        if u<=12:
            trace.append(dict(**row,measurement_matrix=Z.tolist(),covariance=C.tolist(),
                              receiver_pre_matrix=before.tolist(),receiver_post_matrix=after.tolist(),
                              reference_receiver_matrix=receiver_truth.tolist(),reference_sender_matrix=sender_truth.tolist()))
    # Native ROS incremental outputs: same timestamps, no alignment needed for
    # relative errors. Compare every normal consecutive interval, not just ATE.
    native=[];all_diffs=[]
    for u in sorted(odom['on']):
        assert odom['on'][u][0]==odom['off'][u][0]
        diff=log_se3(inverse(odom['off'][u][1])@odom['on'][u][1])
        all_diffs.append(dict(update=u,time_s=odom['on'][u][0]*1e-9-T0,t_m=float(np.linalg.norm(diff[3:])),r_deg=float(np.degrees(np.linalg.norm(diff[:3])))))
        if u-1 not in odom['on'] or (odom['on'][u][0]-odom['on'][u-1][0])*1e-9>.5:continue
        truth=inverse(ref_pose(odom['on'][u-1][0]*1e-9))@ref_pose(odom['on'][u][0]*1e-9)
        errs={m:pose_error(inverse(odom[m][u-1][1])@odom[m][u][1],truth) for m in ['off','on']}
        row=dict(update=u,time_s=odom['on'][u][0]*1e-9-T0,off_t_m=errs['off']['t_m'],on_t_m=errs['on']['t_m'],
                 off_r_deg=errs['off']['r_deg'],on_r_deg=errs['on']['r_deg'],
                 delta_t_m=errs['on']['t_m']-errs['off']['t_m'],delta_r_deg=errs['on']['r_deg']-errs['off']['r_deg'],
                 inserted=sum(v['update']==u for v in ins.values()),removed=life[u]['removal_count'])
        if u in on and u-1 in on[u]:
            expected=inverse(on[u][u-1])@on[u][u]
            actual=inverse(odom['on'][u-1][1])@odom['on'][u][1]
            row['increment_vs_snapshot_max_abs']=float(np.max(np.abs(log_se3(inverse(expected)@actual))))
        else:row['increment_vs_snapshot_max_abs']=None
        native.append(row)
    # Retired endpoint no longer exists after removal: assess ONLY surviving
    # physical edges present in both snapshots. Never reuse a stale retired pose.
    retire_steps=[]
    for u in sorted({v['update'] for v in rem}):
        common=sorted(set(on[u-1]) & set(on[u]))
        before_errors=[];after_errors=[];offb=[];offa=[]
        for i,j in zip(common[:-1],common[1:]):
            if j!=i+1 or kt[j]-kt[i]>.5:continue
            truth=gt(i,j)
            before_errors.append(pose_error(inverse(on[u-1][i])@on[u-1][j],truth))
            after_errors.append(pose_error(inverse(on[u][i])@on[u][j],truth))
            offb.append(pose_error(inverse(off[u-1][i])@off[u-1][j],truth))
            offa.append(pose_error(inverse(off[u][i])@off[u][j],truth))
        assert before_errors
        row=dict(update=u,time_s=odom['on'][u][0]*1e-9-T0,removed_edges=';'.join(str((v['i'],v['j'])) for v in rem if v['update']==u),
                 inserted=sum(v['update']==u for v in ins.values()),surviving_edges=len(before_errors))
        for component in ['t_m','r_deg']:
            for name,values in [('on_pre',before_errors),('on_post',after_errors),('off_pre',offb),('off_post',offa)]:
                row[name+'_'+component+'_rms']=summary([v[component] for v in values])['rms']
        retire_steps.append(row)
    # Unit checks for diagnostic geometry only.
    tangent=np.array([.1,-.2,.05,.4,-.1,.3])
    assert np.max(np.abs(log_se3(exp_se3(tangent))-tangent))<1e-12
    assert max(e['post_final_residual_check'] for e in events)<1e-7
    assert all(sha(Path(p))==h for p,h in hashes.items())
    first_remove=min(v['update'] for v in rem)
    def first_where(rows, predicate):return next((r for r in rows if predicate(r)),None)
    result=dict(
      scope='Retained ETH1 temporary-factor 40 s OFF/ON; no reruns or estimator changes',
      reference='Continuous CPT7 IE-TC; shares CPT7, not independent prism GT',
      counts=dict(accepted=len(events),removed=len(rem),remaining=len(ins)-len(rem),native_intervals=len(native)),
      first_insertion=events[0],first_retirement=retire_steps[0],
      first_marginalization_update=min(u for u,v in life.items() if v['marginal_keys']>0),
      first_native_output_difference=first_where(all_diffs,lambda r:r['t_m']>1e-9 or r['r_deg']>1e-7),
      first_native_worse_translation_1mm=first_where(native,lambda r:r['delta_t_m']>.001),
      first_native_worse_rotation_005deg=first_where(native,lambda r:r['delta_r_deg']>.05),
      first_targeted_edge_translation_worse_1mm=first_where(events,lambda r:r['change_t_m']>.001),
      first_targeted_edge_rotation_worse_005deg=first_where(events,lambda r:r['change_r_deg']>.05),
      thresholds='Diagnostic visibility only, not filter gates: 1 mm translation, 0.05 deg rotation; all events retained.',
      event_statistics={k:summary([r[k] for r in events]) for k in ['from_offset_ms','to_offset_ms','sender_own_interval_t_m','sender_own_interval_r_deg','sender_on_receiver_interval_t_m','sender_on_receiver_interval_r_deg','timing_only_t_m','timing_only_r_deg','kimera_pre_t_m','kimera_post_t_m','kimera_pre_r_deg','kimera_post_r_deg','change_t_m','change_r_deg','pre_residual_check','post_final_residual_check','rot_cov_sqrt_trace_deg','rho_cov_sqrt_trace_m']},
      insertion_counts=dict(translation_error_increased=sum(r['change_t_m']>1e-9 for r in events),rotation_error_increased=sum(r['change_r_deg']>1e-9 for r in events),
                            sender_better_translation_than_receiver_pre=sum(r['sender_on_receiver_interval_t_m']<r['kimera_pre_t_m'] for r in events),
                            sender_better_rotation_than_receiver_pre=sum(r['sender_on_receiver_interval_r_deg']<r['kimera_pre_r_deg'] for r in events)),
      pre_retirement_native_statistics={k:summary([r[k] for r in native if r['update']<first_remove]) for k in ['off_t_m','on_t_m','off_r_deg','on_r_deg']},
      all_native_statistics={k:summary([r[k] for r in native]) for k in ['off_t_m','on_t_m','off_r_deg','on_r_deg']},
      max_native_increment_snapshot_log_diff=max(r['increment_vs_snapshot_max_abs'] or 0 for r in native),
      limitations=['Same-update native visual/IMU factors and nonlinear solve also change; no isolated CBS-off counterfactual within ON graph.',
                   'A removed endpoint is unavailable after marginalization; post-retirement comparison is on surviving edges only.',
                   'Timestamp tolerance matching reuses sender measurement on receiver timestamps; timing-only reference diagnostic is not an applied correction.'],
      input_sha256=hashes)
    write_csv('accepted_insertions.csv',events);write_csv('native_relative_intervals.csv',native)
    write_csv('retirement_surviving_edges.csv',retire_steps);write_csv('native_off_on_pose_differences.csv',all_diffs)
    write_json('early_event_trace.json',trace);write_json('summary.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ['input_sha256','event_statistics']},indent=2))


if __name__=='__main__':main()
