"""Native policy gate: exact raw input, unchanged payload, physical coverage."""
from pathlib import Path
import collections,csv,hashlib,io,json,re,shutil,sys
import numpy as np
import rosbag
R=Path(__file__).resolve().parent;mode=sys.argv[1];D=R/(mode+'_capture')
assert mode in ('temporary','persistent')
assert (D/'complete.json').exists() and not (D/'failed.txt').exists()
assert json.loads((D/'cleanup.json').read_text())['all_exited']
manifest=json.loads((R/'raw_inputs.json').read_text());epoch=manifest['start_ns']
lines=(D/'kimera.log').read_text().splitlines()
for marker in ('CPV73_READINESS_FAILURE','CPV70_DELIVERY_FAILURE','CPV40_FATAL','active_window_rollback',
               'process has died','terminate called','Check failed','CPV76 incompatible','CPV76 invalid',
               'CPV76 marginal','CPV76 missing','CPV76 wrong','CPV76 changed','CPV76 replaced',
               'CPV76 native','CPV76 dangling','CPV76 persistent residual'):
    assert not any(marker in l for l in lines),marker
assert not any('CPV73_READINESS_ROW,' in l or 'CPV70_DELIVERY_ROW,' in l for l in lines)
def serialized(msg):
    s=io.BytesIO();msg.serialize(s);return hashlib.sha256(s.getvalue()).hexdigest()
with rosbag.Bag(str(R/'raw_inputs.bag')) as bag:
    inputs=[dict(arrival_ns=t.to_nsec(),sha256=serialized(m),belief_count=len(m.beliefs)) for _,m,t in bag.read_messages()]
assert inputs==manifest['rows']
poses=[];received=[];native=[]
with rosbag.Bag(str(D/'outputs.bag')) as bag:
    for topic,msg,t in bag.read_messages():
        if topic=='/kimera/cbs/odom_belief_in':received.append(serialized(msg))
        if topic=='/kimera_vio_ros/odometry':
            poses.append(msg.header.stamp.to_nsec());p=msg.pose.pose
            vals=[p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]
            assert np.isfinite(vals).all() and abs(np.linalg.norm(vals[3:])-1)<1e-5
            native.append([msg.header.seq,msg.header.stamp.to_nsec(),msg.header.frame_id,msg.child_frame_id]+vals)
assert received==[x['sha256'] for x in inputs],('wire loss/change/order',len(received),len(inputs))
assert len(poses)>290 and all(a<b for a,b in zip(poses,poses[1:])),('coverage',len(poses))
assert 0<=(poses[0]-epoch)*1e-9<=1 and 59<=(poses[-1]-epoch)*1e-9<=60.1
wire=collections.defaultdict(list)
for b in manifest['beliefs']:wire[tuple(b['sender'])].append(b)
align=[];bounds=[];transport=[];adds=[];decisions=collections.Counter()
for line in lines:
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
        v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].split(',');assert len(v)==13
        r=dict(update=int(v[0]),sender=list(map(int,v[1:3])),receiver=list(map(int,v[3:5])),
            times=list(map(int,v[5:10])),raw=list(map(float,v[10].split(';'))),
            aligned=list(map(float,v[11].split(';'))),cov=list(map(float,re.split('[;:]',v[12]))))
        candidates=[b for b in wire[tuple(r['sender'])] if b['cov']==r['cov'] and np.max(abs(np.array(b['raw'])-r['raw']))<1e-12]
        assert candidates,'raw alignment payload not from input'
        assert any(abs(b['from_stamp_sec']*1e9-r['times'][0])<512 and abs(b['to_stamp_sec']*1e9-r['times'][1])<512 for b in candidates)
        assert poses[r['update']]==r['times'][4]
        assert [poses[i] for i in r['receiver']]==r['times'][2:4]
        assert max(r['times'][:4])<=r['times'][4],'noncausal mean alignment'
        align.append(r)
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        v=line.split('FL4M_FROZEN_FACTOR_ROW,',1)[1].split(',')
        bounds.append((list(map(int,v[:5])),list(map(float,v[5].split(';'))),list(map(float,re.split('[;:]',v[6])))))
    if 'KIMERA_CBS_SOURCE_TRANSPORT_ROW,' in line:
        v=line.split('KIMERA_CBS_SOURCE_TRANSPORT_ROW,',1)[1].split(',')
        transport.append((list(map(int,v[1:3])),list(map(float,re.split('[;:]',v[4]))),list(map(float,re.split('[;:]',v[5])))))
    if 'CBS_BPSAM_ODOM_ADD_ROW_G2K,' in line:
        v=line.split('CBS_BPSAM_ODOM_ADD_ROW_G2K,',1)[1].split(',')
        adds.append(dict(sender=list(map(int,re.findall(r'\d+',v[0]))),receiver=list(map(int,re.findall(r'\d+',v[1]))),status=int(v[2]),reason=v[-1]))
    if 'CBS_ODOM_MATCH_ROW_G2K,' in line:
        v=line.split('CBS_ODOM_MATCH_ROW_G2K,',1)[1].split(',');decisions[v[11]]+=1
assert len(align)==len(bounds)==len(transport)>250
for r,(ids,mu,cov),(key,rawcov,usedcov) in zip(align,bounds,transport):
    assert ids==[r['update']]+r['sender']+r['receiver']
    assert mu==r['aligned'] and cov==r['cov']==rawcov==usedcov and key==r['sender']
assert sum(a['status']==0 for a in adds)>250,('admissions',collections.Counter(a['status'] for a in adds))
allowed={(tuple(r['sender']),tuple(r['receiver'])) for r in align}
assert all((tuple(a['sender']),tuple(a['receiver'])) in allowed for a in adds)
objective=np.loadtxt(D/'objective.csv',delimiter=',')
assert len(objective)>=len(poses)-3 and np.isfinite(objective).all() and set(objective[:,1])=={0}
if mode=='persistent':assert np.sum(objective[:,2])==0,'unexpected temporary retirement in persistent mode'
assert not (D/'estimators.log').exists() and not (D/'kimera_odometry.csv').exists()
shutil.copyfile(D/'kimera.log',D/'estimators.log')
with (D/'kimera_odometry.csv').open('x') as f:
    w=csv.writer(f);w.writerow(['field.header.seq','field.header.stamp','field.header.frame_id','field.child_frame_id']+
        ['field.pose.pose.position.'+k for k in 'xyz']+['field.pose.pose.orientation.'+k for k in 'wxyz']);w.writerows(native)
report=dict(valid=True,mode=mode,pose_count=len(poses),first_s=(poses[0]-epoch)*1e-9,last_s=(poses[-1]-epoch)*1e-9,
    pose_span_s=(poses[-1]-poses[0])*1e-9,maximum_gap_s=max(np.diff(poses))*1e-9,
    gaps_over_half_second=[dict(after_index=i,gap_s=(b-a)*1e-9) for i,(a,b) in enumerate(zip(poses,poses[1:])) if b-a>500000000],
    incoming_messages=len(received),incoming_beliefs=manifest['selected_beliefs'],exact_complete_raw_wire=True,
    aligned_candidates=len(align),exact_native_mean_alignment_receipts=True,exact_covariance=True,
    native_admission_status_counts=dict(collections.Counter(a['status'] for a in adds)),
    native_decision_counts=dict(decisions),native_admissions=adds,
    forced_update_ids=False,planned_temporary_retirements=int(objective[:,2].sum()),
    persistent_absorption_count=None,absorption_note='CP76 runtime assertions active; no per-boundary log count',
    cost_updates=len(objective),cost_increases=int(np.sum(objective[:,5]>objective[:,4]*1.000001+1e-6)))
with (D/'validation.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='native_admissions'},indent=2))
