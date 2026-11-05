"""Planning-only bag-index, bounded cloud-sample and first-static-TF inspection.
No ROS, estimators, containers, playback or model changes.
"""
from pathlib import Path
import collections,json,statistics
import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader
from rosbags.highlevel import AnyReader

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D=W/'src/datasets/GrandTour'
SPEC=[('ETH1','2024-10-01-11-29-55','',1727775007855396309,1727775354175396309),
 ('EIG-2','2024-11-03-13-59-54','EIG-2',1730638816563546034,1730639171583546034),
 ('SPX-3','2024-11-02-17-43-10','SPX-3',1730565899872888651,1730566096892888651),
 ('LEICA-2','2024-11-25-16-36-19','LEICA-2',1732549034915071665,1732549463665071665)]
def T(v):
    m=np.eye(4);m[:3,3]=v[:3];m[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();return m
def chain(edges,a,b):
    q=collections.deque([(a,np.eye(4),[])]);seen=set()
    while q:
        node,m,path=q.popleft()
        if node==b:return dict(matrix=m.tolist(),path=path)
        if node in seen:continue
        seen.add(node)
        for (p,c),v in edges.items():
            if p==node:q.append((c,m@T(v),path+[[p,c]]))
            elif c==node:q.append((p,m@np.linalg.inv(T(v)),path+[[c,p]]))
    return dict(missing=True)
result={}
for seq,mission,sub,start,end in SPEC:
    root=D/sub;p=root/f'{mission}_anymal_velodyne.bag'
    r=dict(path=str(p),present=p.exists(),hesai_reference_start_ns=start,hesai_reference_end_ns=end)
    if not p.exists():result[seq]=r;print(seq,'MISSING',flush=True);continue
    st=p.stat();r.update(bytes=st.st_size,mtime_ns=st.st_mtime_ns)
    with Reader(p) as bag:
        r['topics']=[dict(topic=c.topic,type=c.msgtype,count=c.msgcount) for c in bag.connections]
        cs=[c for c in bag.connections if c.topic=='/anymal/velodyne/points'];assert len(cs)==1
        stamps=[x.time for x in bag.indexes[cs[0].id]]
    chosen=[t for t in stamps if start<=t<end];assert len(chosen)>10
    gaps=np.diff(chosen)/1e9
    r['record_index']=dict(first_ns=stamps[0],last_ns=stamps[-1],selected_count=len(chosen),
        selected_first_ns=chosen[0],selected_last_ns=chosen[-1],median_interval_s=float(np.median(gaps)),
        max_interval_s=float(max(gaps)),monotonic=bool(np.all(gaps>0)),
        start_margin_s=(chosen[0]-start)/1e9,end_margin_s=(end-chosen[-1])/1e9)
    r['samples']=[]
    with AnyReader([p]) as bag:
        cs=[c for c in bag.connections if c.topic=='/anymal/velodyne/points']
        for stamp in [chosen[0],chosen[len(chosen)//2],chosen[-1]]:
            c,t,raw=next(bag.messages(connections=cs,start=stamp,stop=stamp+1))
            m=bag.deserialize(raw,c.msgtype);s=m.header.stamp
            fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields]
            formats={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
            dtype=np.dtype(dict(names=[f.name for f in m.fields],
                formats=[('>' if m.is_bigendian else '<')+formats[f.datatype] for f in m.fields],
                offsets=[f.offset for f in m.fields],itemsize=m.point_step))
            assert all(f.count==1 for f in m.fields)
            a=np.ndarray(shape=(m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
            numeric={}
            for field in ['time','timestamp','ring','intensity']:
                if field in a.dtype.names:
                    v=a[field];ok=np.isfinite(v)
                    numeric[field]=dict(min=float(v[ok].min()),max=float(v[ok].max()),finite=int(ok.sum()))
            xyz=np.column_stack([a[f] for f in ['x','y','z']]);ok=np.all(np.isfinite(xyz),axis=1)
            r['samples'].append(dict(record_ns=t,header_ns=s.sec*10**9+s.nanosec,
                record_minus_header_s=(t-s.sec*10**9-s.nanosec)/1e9,frame=m.header.frame_id,
                width=m.width,height=m.height,point_step=m.point_step,fields=fields,
                point_count=len(a),finite_xyz=int(ok.sum()),range_median_m=float(np.median(np.linalg.norm(xyz[ok],axis=1))),
                numeric=numeric))
    edges={}
    tf=root/f'{mission}_tf_minimal.bag'
    with AnyReader([tf]) as bag:
        cs=[c for c in bag.connections if c.topic=='/tf_static']
        c,t,b=next(bag.messages(connections=cs))
        for x in bag.deserialize(b,c.msgtype).transforms:
            p0=x.transform.translation;q=x.transform.rotation
            edges[(x.header.frame_id,x.child_frame_id)]=[p0.x,p0.y,p0.z,q.x,q.y,q.z,q.w]
    frame=r['samples'][0]['frame'];r['first_static_tf']=dict(path=str(tf),edge_count=len(edges),
        T_lidar_cpt7=chain(edges,frame,'cpt7_imu'))
    assert (st.st_size,st.st_mtime_ns)==(p.stat().st_size,p.stat().st_mtime_ns)
    result[seq]=r
    print(seq,json.dumps(dict(index=r['record_index'],sample=r['samples'][0],tf=r['first_static_tf']['T_lidar_cpt7'])),flush=True)
with (H/'input_inspection.json').open('x') as f:
    json.dump(dict(scope='Planning only; indices plus 3 clouds and first static TF; not full payload/time/calibration validation',sequences=result),f,indent=2,allow_nan=False)
