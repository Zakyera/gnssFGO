"""Independent raw TF/time and source-log-to-event checks; no estimator run."""
import collections
import csv
import json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation
import analyze as a

H=Path(__file__).resolve().parent
assert not (H/'frame_time_checks.json').exists()
pin=a.pin
pin(Path(__file__))
result={'cases':{}}
def pose(v):
    m=np.eye(4);m[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();m[:3,3]=v[:3];return m
V=a.CASES['velodyne'][0]
pre=a.json_load(V/'input_audit.json')
tfpath=pin(pre['bag_files']['tf_minimal']['path'])
assert a.j.sha(tfpath)==pre['bag_files']['tf_minimal']['sha256']
edges={};messages=0
with AnyReader([tfpath]) as reader:
    for conn,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic=='/tf_static']):
        messages+=1
        for x in reader.deserialize(raw,conn.msgtype).transforms:
            p=x.transform.translation;q=x.transform.rotation
            m=pose([p.x,p.y,p.z,q.x,q.y,q.z,q.w]);key=(x.header.frame_id,x.child_frame_id)
            if key in edges:assert np.max(np.abs(edges[key]-m))<1e-12
            edges[key]=m
def chain(start,end):
    todo=collections.deque([(start,np.eye(4),[start])]);seen=set()
    while todo:
        node,T,path=todo.popleft()
        if node==end:return T,path
        if node in seen:continue
        seen.add(node)
        for (x,y),v in edges.items():
            if x==node:todo.append((y,T@v,path+[y]))
            elif y==node:todo.append((x,T@np.linalg.inv(v),path+[x]))
    raise RuntimeError('TF path missing')
result['tf_static_messages_verified']=messages
epoch=1732549034915071665
raw_bags={'velodyne':Path(pre['bag_files']['anymal_velodyne']['path']),
          'hesai':Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/LEICA-2/2024-11-25-16-36-19_hesai.bag')}
source_files=[
 a.RUNS/'e1b_local_profile_20261012/overlay/odometry_estimation_imu.cpp',
 a.RUNS/'local_revision_covariance_cpi1_20261019/cbs_bridge.cpp',
 Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim/src/glim/util/time_keeper.cpp'),
 Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim/src/glim/preprocess/cloud_preprocessor.cpp'),
 Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim/src/glim/common/cloud_deskewing.cpp'),
 a.E/'receiver_finalization_cpk19_20261021/Receiver.h',
 Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')]
for p in source_files:pin(p)
for name,(P,_) in a.CASES.items():
    src=P/'full_off_source';root=P/'full_off_receiver'
    prot=a.json_load(P/'protected_manifest.json')
    protected={str(p):a.j.sha(p)==prot[str(p)] for p in source_files if str(p) in prot}
    assert all(protected.values())
    cfg=a.jsonc(src/'glim_config/config_sensors.json')['sensors']
    T,path=chain(name+'_lidar','cpt7_imu');err=float(np.max(np.abs(T-pose(cfg['T_lidar_imu']))));assert err<1e-8
    out={'extrinsic_tf_chain':path,'extrinsic_max_matrix_difference':err,'protected_source_matches':protected}
    stamps=a.a.load_curve(pin(src/'glim_odometry.csv'))[0]
    sensor_rows=[]
    bag=raw_bags[name];before=bag.stat()
    topic='/anymal/velodyne/points' if name=='velodyne' else '/boxi/hesai/points'
    with AnyReader([bag]) as reader:
        for conn,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic==topic],start=epoch,stop=epoch+2_000_000_000):
            m=reader.deserialize(raw,conn.msgtype)
            assert m.header.frame_id==name+'_lidar' and not m.is_bigendian
            fields={f.name:f for f in m.fields}
            field=next(k for k in ('t','time','timestamp') if k in fields)
            f=fields[field];assert f.count==1
            kinds={1:'i1',2:'u1',3:'<i2',4:'<u2',5:'<i4',6:'<u4',7:'<f4',8:'<f8'}
            dt=np.dtype({'names':['time'],'formats':[kinds[f.datatype]],'offsets':[f.offset],'itemsize':m.point_step})
            times=np.frombuffer(m.data,dtype=dt,count=m.width*m.height)['time'].astype(float)
            assert np.isfinite(times).all()
            mn,mx=times.min(),times.max();header=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            scale=1e-9 if mn>1e16 else 1.
            if mx<1.:
                normalized=header+(mn*scale if mn<0 else 0.)
                local=(times-(mn if mn<0 else 0.))*scale
                expected=header+times*scale
            else:
                normalized=mn*scale;local=(times-mn)*scale;expected=times*scale
            conservation=float(np.max(np.abs(normalized+local-expected)))
            assert conservation<=5e-7
            nearest=int(np.argmin(np.abs(stamps*1e-9-normalized)))
            difference=abs(stamps[nearest]*1e-9-normalized)
            sensor_rows.append({'record_s':(t-epoch)*1e-9,'time_field':field,'min_raw_time':float(mn),
               'max_raw_time':float(mx),'normalized_minus_header_s':float(normalized-header),
               'scan_duration_s':float(local.max()-local.min()),'point_count':len(times),
               'absolute_time_conservation_max_s':conservation,
               'native_timestamp_match':bool(difference<=5e-6),'nearest_native_time_error_s':float(difference)})
    assert (before.st_size,before.st_mtime_ns)==(bag.stat().st_size,bag.stat().st_mtime_ns)
    out['raw_time_samples']=sensor_rows
    out['raw_bag_identity']={'path':str(bag),'size':before.st_size,'mtime_ns':before.st_mtime_ns,
        'scope':'First2s raw point arrays read; no full bag hash repeated here. Frozen full bag hash is in preflight.'}
    protocol=a.module('protocol_'+name,root/'transport_code/live_receiver_protocol.py');pin(protocol.__file__)
    expected={}
    with pin(root/'events.csv').open() as f:
        for row in csv.reader(f):
            if row[0]=='B' and int(row[5])==0:
                kind='glim' if row[2].startswith('glim_') else 'kimera'
                expected[(kind,int(row[6]),int(row[7]))]=(int(row[8]),int(row[9]),np.array(row[10:],float))
    seen=set();checked=collections.Counter();maxdelta=0.
    paths={'glim':next(src.glob('ros_logs/*/glim_rosnode-*-stdout.log')),'kimera':src/'estimators.log'}
    for kind,p in paths.items():
        pin(p)
        with p.open(errors='strict') as f:
            for line in f:
                markers=('GLIM_CBS_EXPORT_BELIEF_ROW,','GLIM_CBS_REVISION_BELIEF_ROW,') if kind=='glim' else ('KIMERA_CBS_REVISION_EDGE_ROW,',)
                marker=next((s for s in markers if s in line),None)
                if marker is None:continue
                fields=line.split(marker,1)[1].split(',')
                ij=fields[1:3] if kind=='glim' else fields[2:4]
                key=(kind,int(ij[0]),int(ij[1]))
                if key in seen:continue
                seen.add(key)
                if key not in expected:continue
                b=protocol.parse_belief(kind,line);ti,tj,values=expected[key]
                assert (b['ti'],b['tj'])==(ti,tj)
                delta=float(np.max(np.abs(np.array(b['mean']+b['covariance'])-values)))
                assert delta==0.;maxdelta=max(maxdelta,delta);checked[kind]+=1
    assert seen.issuperset(expected)
    assert sum(checked.values())==len(expected)
    out['all_first_beliefs_log_to_transport']={'counts':dict(checked),'mean_covariance_max_difference':maxdelta,
        'endpoint_timestamps_exact':True,'no_transport_rotation_rescaling_or_time_shift':True}
    result['cases'][name]=out
    print(name,'TF/time/log checks pass',flush=True)
result['inputs_unchanged']=all(a.j.sha(p)==s for p,s in a.j.HASHES.items());assert result['inputs_unchanged']
result['input_sha256']=dict(a.j.HASHES)
a.save('frame_time_checks.json',result)
print(json.dumps({k:{q:v for q,v in d.items() if q not in ('raw_time_samples','protected_source_matches')} for k,d in result['cases'].items()},indent=2))
