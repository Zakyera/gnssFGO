"""Bounded same-input retirement comparison using existing IE-TC decoder."""
import csv,json,math,re,sys
from pathlib import Path
import numpy as np
import rosbag
R=Path(__file__).resolve().parent
W=Path('/workspace/cbs_gtsam4.3')
sys.path.insert(0,str(W/'src/cbsms/tools'))
import score_grandtour_relative_pose as rel
gtdata=rel.sort_unique(*rel.load_reference_bag(W/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag','/boxi/inertial_explorer/tc/odometry')[:3])
def gt(t):return rel.interpolate_pose(*gtdata,t,.05)
def between(a,b):return a[1].T@(b[0]-a[0]),a[1].T@b[1]
def err(a,b):return float(np.linalg.norm(a[0]-b[0])),math.degrees(math.acos(float(np.clip((np.trace(a[1].T@b[1])-1)/2,-1,1))))
def pose(v):
    v=list(map(float,v));return np.array(v[:3]),rel.quaternion_to_matrix(v[4:7]+[v[3]])
def stats(v):
    v=np.array(v);return dict(n=len(v),rms=float(np.sqrt(np.mean(v*v))),max=float(np.max(v)),median=float(np.median(v))) if len(v) else None
frozen=json.loads((R/'frozen_inputs.json').read_text())['rows']
report={'reference':'IE-TC in CPT7 frame, shares IMU; not independent GT','cases':{}}
native_cases={}
for mode in ['remove','retain']:
    D=R/(mode+'_capture')
    assert json.loads((D/'validation.json').read_text())['valid']
    native={};snapshots={};guards=[]
    with rosbag.Bag(str(D/'outputs.bag')) as bag:
        for _,m,_ in bag.read_messages(topics=['/kimera_vio_ros/odometry']):
            p=m.pose.pose;native[m.header.seq]=(m.header.stamp.to_nsec()*1e-9,
                (np.array([p.position.x,p.position.y,p.position.z]),rel.quaternion_to_matrix([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])))
    for line in (D/'kimera.log').open():
        if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
            v=line.split('KIMERA_CBS_REVISION_EDGE_ROW,')[1].strip().split(',')
            u,i,j=int(v[0]),int(v[2]),int(v[3]);snapshots.setdefault(u,{})[i]=pose(v[8:15]);snapshots[u][j]=pose(v[15:22])
        if ',active_window_rollback,' in line or ',active_window_staged_factor_dropped,' in line or 'action=drop_before_stage' in line:guards.append(line.strip())
    costs=np.atleast_2d(np.loadtxt(D/'objective.csv',delimiter=','));byu={int(row[0]):row for row in costs}
    rows=[]
    for u,(t,p) in native.items():
        row={'update':int(u),'time':t-1727775007}
        if u-1 in native:
            tp,pp=native[u-1]
            if t-tp<.5:row['native_increment_error_m'],row['native_increment_error_deg']=err(between(pp,p),between(gt(tp),gt(t)))
        if u in byu:
            c=byu[u];row.update(cost_before=c[4],cost_after=c[5],imu_before=c[6],imu_after=c[7],prior_before=c[8],prior_after=c[9],retirements=int(c[2]),prior_count=int(c[3]))
        rows.append(row)
    byrow={r['update']:r for r in rows}
    for f in frozen:
        u=f['update'];i,j=f['receiver'];truth=between(gt(f['times'][2]*1e-9),gt(f['times'][3]*1e-9))
        for name,sn in [('before',snapshots[u-1]),('after',snapshots[u])]:
            byrow[u]['target_'+name+'_m'],byrow[u]['target_'+name+'_deg']=err(between(sn[i],sn[j]),truth)
    bins={}
    for lo,hi in [(0,75),(75,90),(90,100)]:
        q=[r for r in rows if lo<=r['time']<hi and 'native_increment_error_m' in r]
        bins[str(lo)+'_'+str(hi)]={'translation_m':stats([r['native_increment_error_m'] for r in q]),'rotation_deg':stats([r['native_increment_error_deg'] for r in q])}
    increases=[r for r in rows if r.get('cost_after',0)>r.get('cost_before',0)*1.000001+1e-6]
    report['cases'][mode]=dict(bins=bins,cost_increase_count=len(increases),first_cost_increases=increases[:5],
        largest_cost_increases=sorted(increases,key=lambda r:r['cost_after']-r['cost_before'],reverse=True)[:5],
        update438=byrow.get(438),first_native_step_over_5cm=next((r for r in rows if r.get('native_increment_error_m',0)>.05),None),
        first_retirement=next((r for r in rows if r.get('retirements',0)),None),guards=guards,rows=rows)
    native_cases[mode]=native
a=native_cases['remove'];b=native_cases['retain']
assert a.keys()==b.keys() and all(a[u][0]==b[u][0] for u in a)
first=report['cases']['remove']['first_retirement']['update']
report['pre_retirement_parity']=dict(first_retirement_update=first,
    max_position_difference_m=max(float(np.linalg.norm(a[u][1][0]-b[u][1][0])) for u in a if u<first),
    max_rotation_difference_deg=max(err(a[u][1],b[u][1])[1] for u in a if u<first))
with (R/'analysis.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in report.items() if k!='cases'},indent=2))
for mode,c in report['cases'].items():print(mode,json.dumps({k:v for k,v in c.items() if k not in ['rows','guards']},indent=2))
