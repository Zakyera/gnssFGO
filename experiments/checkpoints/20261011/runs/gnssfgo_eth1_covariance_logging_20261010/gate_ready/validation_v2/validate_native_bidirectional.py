"""Read-only native alignment and exact wire covariance gate (ROS Python)."""
from pathlib import Path
import json
import re
import sys
import numpy as np
import rosbag

ROOT=Path(__file__).resolve().parent.parent
tag=sys.argv[1]
assert tag in ['gate_on','pilot_on']
source=ROOT/(tag+'_source')
assert json.loads((ROOT/'capture_review.json').read_text())['captured_data_complete']
logs=[p for p in (source/'ros_logs').glob('*/*stdout.log') if p.parent.name!='latest']
glog,=[p for p in logs if 'glim_rosnode' in p.name]
klog,=[p for p in logs if 'kimera_vio' in p.name]
clean=lambda s:re.sub(r'\x1b\[[0-9;]*m','',s)
gtext=clean(glog.read_text()); ktext=clean(klog.read_text()); main=clean((source/'estimators.log').read_text())
for text in [gtext,ktext,main]:
    for token in ['MEAN_ALIGNMENT_UNAVAILABLE','failed to build external odometry factor',
                  'mean bridge:', 'terminate called']:
        assert token not in text, token
wire={'g':{},'k':{}}
with rosbag.Bag(str(source/'recordings/cbs.bag')) as bag:
    for topic,msg,stamp in bag.read_messages():
        if topic not in ['/kimera/cbs/odom_belief_out','/kimera/cbs/odom_belief_in']:continue
        sender='k' if topic.endswith('odom_belief_out') else 'g'
        for b in msg.beliefs:
            payload=dict(mu=list(b.relative_mu),cov=list(b.covariance),
                         ti=b.from_stamp_sec,tj=b.to_stamp_sec)
            key=(b.from_pose_index,b.to_pose_index)
            wire[sender].setdefault(key,[]).append(payload)

def vector(s):return np.array([float(x) for x in re.split('[:;]',s)])
checks={}
for kind,text,sender in [('glim',gtext,'k'),('kimera',main,'g')]:
    marker=kind.upper()+'_CBS_MEAN_ALIGNMENT_ROW,'
    rows=[]; mats={}
    factor_text=gtext if kind=='glim' else ktext
    for line in factor_text.splitlines():
        if 'CBS_ODOM_FACTOR_MATRIX_ROW,' not in line:continue
        v=line.split('CBS_ODOM_FACTOR_MATRIX_ROW,')[1].split(',')
        if v[0] not in ['K2G','G2K']:continue
        key=(int(v[1][1:]),int(v[2][1:]))
        mats.setdefault(key,[]).append(vector(v[4]))
    for line in text.splitlines():
        if marker not in line:continue
        v=line.split(marker)[1].split(',')
        if kind=='kimera':
            update=int(v[0]);v=v[1:]
        else:update=None
        si,sj,ri,rj=map(int,v[:4])
        times=list(map(float,v[4:9]));a,b,c,d,now=times
        assert max(a,b,c,d)<=now
        raw,aligned,cov=map(vector,v[9:12])
        originals=wire[sender][(si,sj)]
        assert any(np.array_equal(cov,o['cov']) and np.max(np.abs(raw-o['mu']))<1e-12
                   for o in originals), (kind,'wire payload mismatch',si,sj)
        assert (ri,rj) in mats
        # Each adapter call that reached matrix construction must retain all 36
        # entries. Existing duplicate/lifetime guards may reject AFTER construction.
        assert any(np.array_equal(cov,m) for m in mats[(ri,rj)]), (kind,'factor covariance mismatch',ri,rj)
        rows.append(dict(sender=[si,sj],receiver=[ri,rj],update=update,times=times,
                         mean_log_change=float(np.linalg.norm(aligned-raw))))
    assert rows,(kind,'No aligned factors')
    checks[kind]=dict(aligned_calls=len(rows),factor_matrix_calls=sum(map(len,mats.values())),
        all_36_covariance_entries_exact=True,causal_endpoint_cutoffs_valid=True,
        rows=rows)
result=dict(passed=True,tag=tag,wire_ids={k:len(v) for k,v in wire.items()},
    no_missing_history_or_raw_fallback=True,receivers=checks)
with (ROOT/(tag+'_native_validation.json')).open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({**result,'receivers':{k:{a:b for a,b in v.items() if a!='rows'} for k,v in checks.items()}},indent=2))
