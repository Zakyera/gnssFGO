"""Saved-data extension: CBS temporal support and actual revised active-window motion."""
import analyze as a
import csv, importlib.util, json, re
import numpy as np

out=a.H/'support_audit.json'
assert not out.exists()
node=next(a.S.glob('ros_logs/*/glim_rosnode-*-stdout.log'))
updates=[];inject=[];poses={};mismatches=[];scans=[]
for no,raw in enumerate(a.pin(node).open(),1):
    l=a.ANSI.sub('',raw).strip()
    if not any(x in l for x in ['GLIM_CBS_TIMING_ROW,on_smoother_update,','GLIM_CBS_ODOM_INJECT_ROW,',
        'GLIM_CBS_ACTIVE_POSE_ROW,','CBS_ODOM_MATCH_ROW_K2G,','GLIM_ODOM_IMU_TIMING_ROW,']):continue
    m=re.search(r'\[\d+\.\d+, (\d+\.\d+)\]',l)
    t=float(m.group(1))-a.START*1e-9 if m else None
    if 'GLIM_CBS_TIMING_ROW,on_smoother_update,' in l and 500<t<514:
        f=a.fields(l,'GLIM_CBS_TIMING_ROW')
        updates.append(dict(line=no,t=t,update=int(f[1]),matched=int(f[2]),injected=int(f[3]),pending=int(f[6]),
                            oldest_pending_age_s=float(f[7]) if f[7]!='nan' else None,
                            removed=int(f[13]),active_external=int(f[14])))
    if 'GLIM_CBS_ODOM_INJECT_ROW,' in l and 500<t<514:inject.append(dict(line=no,t=t,text=l))
    if 'CBS_ODOM_MATCH_ROW_K2G,' in l and 500<t<514:mismatches.append(dict(line=no,t=t,text=l))
    if 'GLIM_CBS_ACTIVE_POSE_ROW,' in l:
        f=a.fields(l,'GLIM_CBS_ACTIVE_POSE_ROW');u=int(f[1])
        if 5006<=u<=5023:poses.setdefault(u,[]).append((int(f[3]),float(f[4]),np.array(list(map(float,f[5:8])))))
    if 'GLIM_ODOM_IMU_TIMING_ROW,' in l:
        f=a.fields(l,'GLIM_ODOM_IMU_TIMING_ROW')
        if 5006<=int(f[1])<=5023:scans.append(dict(id=int(f[1]),t=float(f[0])-a.START*1e-9,points=int(f[2]),
            integrated_imu=int(f[3]),active_frames=int(f[5]),main_ms=float(f[17]),scan_ms=float(f[23])))

mod=a.H.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'
spec=importlib.util.spec_from_file_location('ref',a.pin(mod));g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
audit=a.load(a.P/'input_audit.json');ref=g.Reference(a.pin(a.Path(audit['bag_files']['cpt7_ie_tc']['path'])))
active_summary=[]
for u,rows in poses.items():
    rows=sorted(rows);edges=[]
    for (ka,ta,pa),(kb,tb,pb) in zip(rows,rows[1:]):
        d=float(np.linalg.norm(pb-pa));ga=ref.at(int(round(ta*1e9)));gb=ref.at(int(round(tb*1e9)))
        gd=float(np.linalg.norm(gb[:3,3]-ga[:3,3]))
        edges.append(dict(from_id=ka,to_id=kb,dt_s=tb-ta,step_m=d,reference_step_m=gd))
    active_summary.append(dict(update=u,rows=len(rows),largest=sorted(edges,key=lambda e:e['step_m'],reverse=True)[:3]))

# Best-case timestamp support using all saved native GLIM stamps. This cannot
# prove exact delivery order, but absence even with future stamps is decisive.
native=list(csv.DictReader(a.pin(a.S/'glim_odometry.csv').open()))
stamps=np.array([int(r['field.header.stamp']) for r in native],np.int64)
seen=set();beliefs=[]
for row in csv.reader(a.pin(a.P/'full_on_receiver/events.csv').open()):
    if row[0]!='B' or row[2]!='kimera_front':continue
    key=tuple(row[6:8])
    if key in seen:continue
    seen.add(key);tf,tt=map(int,row[8:10]);lo=(tf-a.START)*1e-9;hi=(tt-a.START)*1e-9
    if not (lo>=501.4 and hi<=512.1):continue
    aa=np.flatnonzero(abs(stamps-tf)<=120_000_000);bb=np.flatnonzero(abs(stamps-tt)<=120_000_000)
    possible=[]
    for i in aa:
        for j in bb:
            if i>=j:continue
            ds=tt-tf;dr=int(stamps[j]-stamps[i])
            if abs(ds-dr)<=60_000_000 and .5<=ds/dr<=2:possible.append((int(i),int(j)))
    beliefs.append(dict(from_id=int(key[0]),to_id=int(key[1]),start_s=lo,end_s=hi,
                        first_receiver_availability_s=(int(row[1])-a.START)*1e-9,
                        best_case_matching_pairs=possible))
result=dict(task='CP-V19',updates=updates,injections=inject,duration_mismatches=mismatches,
  sparse_scan_updates=scans,revised_active_window_motion=active_summary,
  best_case_K2G_timestamp_support=beliefs,kimera_beliefs=len(beliefs),
  temporally_matchable=sum(bool(b['best_case_matching_pairs']) for b in beliefs),
  semantics='Active-window adjacent poses are from one update, not stitched first publications. Timestamp audit is best-case offline geometry, not an estimator replay or delivery reconstruction.',
  input_sha256=a.pins)
with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:result[k] for k in ['kimera_beliefs','temporally_matchable','sparse_scan_updates','revised_active_window_motion']},indent=2))
