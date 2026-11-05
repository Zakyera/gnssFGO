"""Complete read-only cloud-format/point-time evidence; not capture admission."""
from pathlib import Path
import hashlib,json
import numpy as np
from rosbags.highlevel import AnyReader

H=Path(__file__).resolve().parent
p=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/ARC-3/2024-11-18-13-48-19_anymal_velodyne.bag')
before=p.stat()
want=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',18,7)]
dtype=np.dtype(dict(names=[v[0] for v in want],formats=['<f4']*4+['<u2','<f4'],offsets=[v[1] for v in want],itemsize=22))
clouds=[];failures=[]
with AnyReader([p]) as r:
    for c,t,b in r.messages(connections=[c for c in r.connections if c.topic=='/anymal/velodyne/points']):
        m=r.deserialize(b,c.msgtype);h=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        shape=(h==t and m.header.frame_id=='velodyne_lidar' and m.height==1 and
            [(f.name,f.offset,f.datatype) for f in m.fields]==want and all(f.count==1 for f in m.fields)
            and m.point_step==22 and not m.is_bigendian)
        if not shape:failures.append(dict(record_ns=t,reason='shape'));continue
        a=np.frombuffer(m.data,dtype=dtype,count=m.width)
        finite=len(a)>0 and all(np.isfinite(a[f]).all() for f in ('x','y','z','time'))
        if not finite:failures.append(dict(record_ns=t,reason='empty_or_nonfinite'));continue
        mn=float(a['time'].min());mx=float(a['time'].max())
        clouds.append(dict(record_ns=t,header_ns=h,count=len(a),min_offset_s=mn,max_offset_s=mx,
            point_time_span_s=mx-mn,normalized_stamp_s=h/1e9+mn,
            strict_cpv4_point_time_pass=bool(-.11<mn<0 and mn<mx<.01 and .08<mx-mn<.11)))
with p.open('rb') as f:digest=hashlib.file_digest(f,'sha256').hexdigest()
after=p.stat();assert (before.st_size,before.st_mtime_ns)==(after.st_size,after.st_mtime_ns)
spans=np.array([x['point_time_span_s'] for x in clouds]);counts=np.array([x['count'] for x in clouds])
result=dict(admitted=False,path=str(p),sha256=digest,bytes=after.st_size,mtime_ns=after.st_mtime_ns,
    total_clouds=len(clouds)+len(failures),structural_failures=failures,
    point_count_min_median_max=[int(counts.min()),float(np.median(counts)),int(counts.max())],
    point_time_span_min_median_max_s=[float(spans.min()),float(np.median(spans)),float(spans.max())],
    normalized_stamp_regressions=int((np.diff([x['normalized_stamp_s'] for x in clouds])<=0).sum()),
    strict_point_time_failures=[x for x in clouds if not x['strict_cpv4_point_time_pass']],clouds=clouds)
with (H/'cloud_diagnosis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='clouds'},indent=2))
