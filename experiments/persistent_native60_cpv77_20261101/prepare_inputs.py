"""Copy every raw wire message in the physical window, byte-exact at record time."""
from pathlib import Path
import hashlib,io,json
import rosbag
R=Path(__file__).resolve().parent;W=Path('/workspace/cbs_gtsam4.3')
S=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared/full_on_source'
DS=W/'src/datasets/GrandTour/EIG-2';mission='2024-11-03-13-59-54'
start=1730638816563546034;end=start+60_000_000_000;base=start-5_700_000_000
assert not (R/'raw_inputs.bag').exists()
metadata={}
for kind in ('hesai','cpt7_imu','alphasense'):
    with rosbag.Bag(str(DS/(mission+'_'+kind+'.bag'))) as bag:
        metadata[kind]=dict(start_seconds=bag.get_start_time(),end_seconds=bag.get_end_time())
# Public API start_time is float; allow nanosecond float conversion precision.
assert abs(min(x['start_seconds'] for x in metadata.values())*1e9-base)<512,metadata
rows=[];beliefs=[];total=0
with rosbag.Bag(str(S/'recordings/cbs.bag')) as source,rosbag.Bag(str(R/'raw_inputs.bag'),'w') as out:
    for topic,msg,t in source.read_messages(topics=['/kimera/cbs/odom_belief_in']):
        total+=1
        if not start<=t.to_nsec()<end:continue
        stream=io.BytesIO();msg.serialize(stream);payload=stream.getvalue()
        rows.append(dict(arrival_ns=t.to_nsec(),sha256=hashlib.sha256(payload).hexdigest(),belief_count=len(msg.beliefs)))
        for b in msg.beliefs:
            beliefs.append(dict(sender=[b.from_pose_index,b.to_pose_index],arrival_ns=t.to_nsec(),
                from_stamp_sec=b.from_stamp_sec,to_stamp_sec=b.to_stamp_sec,raw=list(b.relative_mu),cov=list(b.covariance)))
        out.write(topic,msg,t)
assert rows and all(a['arrival_ns']<=b['arrival_ns'] for a,b in zip(rows,rows[1:]))
assert len(beliefs)>500,'unexpectedly incomplete raw stream'
with (R/'raw_inputs.json').open('x') as f:
    json.dump(dict(start_ns=start,end_ns=end,base_ns=base,duration=60.,source=str(S/'recordings/cbs.bag'),
        original_all_window_messages=total,selected_messages=len(rows),selected_beliefs=len(beliefs),rows=rows,beliefs=beliefs,
        accepted_filter=False,mean_alignment='native at admission, no saved aligned replacement',raw_bag_metadata=metadata),f,indent=2)
print('COMPLETE RAW INPUT',len(rows),'messages',len(beliefs),'beliefs; no accepted-only filter')
