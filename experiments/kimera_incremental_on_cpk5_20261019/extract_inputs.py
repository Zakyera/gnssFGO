"""Copy only saved raw peer messages; no change to payload, stamps or covariance."""
import hashlib,io,json,struct
from pathlib import Path
import rosbag
import numpy as np
R=Path(__file__).resolve().parent;W=Path('/workspace/cbs_gtsam4.3')
S=W/'runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared/live120_on_source'
O=W/'runs/kimera_incremental_control_cpk4b_20261019'
TOPIC='/kimera/cbs/odom_belief_in'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def serial(m):
    out=io.BytesIO();m.serialize(out);return out.getvalue()
assert not (R/'raw_glim_inputs.bag').exists(),'Refuse overwrite'
rows=[];digest=hashlib.sha256();lags=[]
with rosbag.Bag(str(S/'recordings/cbs.bag')) as src,rosbag.Bag(str(R/'raw_glim_inputs.bag'),'w') as dst:
    for topic,m,t in src.read_messages(topics=[TOPIC]):
        assert len(m.beliefs)==1
        b=m.beliefs[0];c=np.asarray(b.covariance).reshape(6,6)
        assert b.revision==0 and b.to_pose_index==b.from_pose_index+2
        assert np.isfinite(c).all() and np.isfinite(b.relative_mu).all()
        assert np.max(np.abs(c-c.T))<1e-12 and np.linalg.eigvalsh(c).min()>-1e-12
        assert t.to_sec()>=b.to_stamp_sec>b.from_stamp_sec
        raw=serial(m);digest.update(struct.pack('<Q',t.to_nsec())+raw)
        rows.append(dict(arrival_ns=t.to_nsec(),from_s=b.from_stamp_sec,to_s=b.to_stamp_sec,
                         from_index=int(b.from_pose_index),to_index=int(b.to_pose_index),
                         payload_sha256=hashlib.sha256(raw).hexdigest()))
        lags.append(t.to_sec()-b.to_stamp_sec)
        dst.write(topic,m,t)
check=hashlib.sha256()
with rosbag.Bag(str(R/'raw_glim_inputs.bag')) as bag:
    for _,m,t in bag.read_messages():check.update(struct.pack('<Q',t.to_nsec())+serial(m))
assert digest.hexdigest()==check.hexdigest() and len(rows)==1189
inside=[x for x in rows if 1727775007.955396<=x['arrival_ns']*1e-9<1727775118.955396]
assert len(inside)==1100
out=dict(passed=True,topic=TOPIC,source_bag=str(S/'recordings/cbs.bag'),source_sha256=sha(S/'recordings/cbs.bag'),
         extracted_sha256=sha(R/'raw_glim_inputs.bag'),exact_payload_and_record_time_digest=digest.hexdigest(),
         all_messages=len(rows),within111s=len(inside),first=rows[0],last=rows[-1],
         lag_min_s=min(lags),lag_max_s=max(lags),only_revision_zero=True,rows=rows)
(R/'input_provenance.json').write_text(json.dumps(out,indent=2))
manifest={str(p):sha(p) for p in R.rglob('*') if p.is_file()}
for p,h in json.loads((O/'pre_capture_receipt.json').read_text())['manifest'].items():
    if '/kimera_failed_factor_cpk1_20261019/' in p:
        path=p.replace('/home/yeranis/repos/V4RL','/workspace')
        assert sha(path)==h
        manifest[path]=h
(R/'pre_capture_receipt.json').write_text(json.dumps(dict(manifest=manifest,one_playback=True,profile_unchanged=True),indent=2))
print(json.dumps({k:v for k,v in out.items() if k!='rows'},indent=2))
