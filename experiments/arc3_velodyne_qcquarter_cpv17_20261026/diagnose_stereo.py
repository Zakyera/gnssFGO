"""Read-only ARC-3 stereo/timing diagnosis after preflight, no estimator."""
from pathlib import Path
import bisect, json, struct
import numpy as np
from rosbags.rosbag1 import Reader

H=Path(__file__).resolve().parent
D=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/ARC-3')
M='2024-11-18-13-48-19'
topics=['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']
rows={t:[] for t in topics}
with Reader(D/(M+'_alphasense.bag')) as r:
    for c,t,b in r.messages(connections=[c for c in r.connections if c.topic in topics]):
        seq,sec,ns=struct.unpack_from('<III',b)
        rows[c.topic].append([t,sec*10**9+ns,seq])
epoch=min(x[0] for v in rows.values() for x in v)
summary={}
for topic,a in rows.items():
    h=np.array([x[1] for x in a],dtype=np.int64)
    gaps=np.diff(h)
    summary[topic]=dict(count=len(a),first=a[0],last=a[-1],header_regressions=int((gaps<=0).sum()),
        max_gap_s=float(gaps.max()/1e9),max_record_header_offset_ns=max(abs(x[0]-x[1]) for x in a),
        gaps_over_110ms=[dict(before=a[i],after=a[i+1],gap_s=float(gaps[i]/1e9),relative_s=(a[i][1]-epoch)/1e9) for i in np.flatnonzero(gaps>110_000_000)])
left={x[1]:x for x in rows[topics[0]]};right={x[1]:x for x in rows[topics[1]]}
unmatched={"left_only":[left[t] for t in sorted(left.keys()-right.keys())],"right_only":[right[t] for t in sorted(right.keys()-left.keys())]}
matched=sorted(left.keys()&right.keys())
result=dict(epoch_ns=epoch,topics=summary,unmatched=unmatched,common_count=len(matched),
 common_first_ns=matched[0],common_last_ns=matched[-1],common_max_gap_s=max(np.diff(matched))/1e9,
 common_gaps_over_110ms=[dict(before_ns=matched[i],after_ns=matched[i+1],gap_s=(matched[i+1]-matched[i])/1e9,relative_s=(matched[i]-epoch)/1e9) for i in range(len(matched)-1) if matched[i+1]-matched[i]>110_000_000])
with (H/'stereo_diagnosis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
