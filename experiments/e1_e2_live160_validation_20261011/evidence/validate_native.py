"""Same frozen capture, stricter lifecycle-aware admission accounting. ROS Python."""
import argparse
import json
from pathlib import Path
import re
import rosbag
from admission_contract import numbers, kimera_outcomes, verify_candidates

p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
a=p.parse_args();source=a.capture/'prepared/live160_on_source'
assert (source/'complete.marker').is_file() and not a.out.exists()
logs=[p for p in (source/'ros_logs').glob('*/*stdout.log') if p.parent.name!='latest']
glog,=[p for p in logs if 'glim_rosnode' in p.name]
klog,=[p for p in logs if 'kimera_vio' in p.name]
clean=lambda s:re.sub(r'\x1b\[[0-9;]*m','',s)
gtext=clean(glog.read_text());ktext=clean(klog.read_text());main=clean((source/'estimators.log').read_text())
for text in [gtext,ktext,main]:
    for token in ['MEAN_ALIGNMENT_UNAVAILABLE','failed to build external odometry factor','mean bridge:','terminate called']:
        assert token not in text,token
wire={'g':{},'k':{}}
with rosbag.Bag(str(source/'recordings/cbs.bag')) as bag:
    for topic,msg,stamp in bag.read_messages():
        if topic not in ['/kimera/cbs/odom_belief_out','/kimera/cbs/odom_belief_in']:continue
        sender='k' if topic.endswith('odom_belief_out') else 'g'
        for b in msg.beliefs:
            wire[sender].setdefault((b.from_pose_index,b.to_pose_index),[]).append(dict(
                mu=tuple(b.relative_mu),cov=tuple(b.covariance),ti=b.from_stamp_sec,tj=b.to_stamp_sec))
checks={}
for kind,text,sender in [('glim',gtext,'k'),('kimera',main,'g')]:
    marker=kind.upper()+'_CBS_MEAN_ALIGNMENT_ROW,'
    candidates=[]
    for ln,line in enumerate(text.splitlines(),1):
        if marker not in line:continue
        v=line.split(marker,1)[1].strip().split(',');update=None
        if kind=='kimera':update=int(v.pop(0))
        si,sj,ri,rj=map(int,v[:4])
        candidates.append(dict(sender=(si,sj),receiver=(ri,rj),update=update,
            times=tuple(map(float,v[4:9])),raw=numbers(v[9]),aligned=numbers(v[10]),cov=numbers(v[11]),line=ln))
    assert candidates
    if kind=='kimera':outcomes=kimera_outcomes(ktext.splitlines())
    else:
        matrices={}
        for ln,line in enumerate(gtext.splitlines(),1):
            if 'CBS_ODOM_FACTOR_MATRIX_ROW,K2G,' not in line:continue
            v=line.split('CBS_ODOM_FACTOR_MATRIX_ROW,',1)[1].strip().split(',')
            key=(None,int(v[1][1:]),int(v[2][1:]));assert key not in matrices
            matrices[key]=(numbers(v[4]),ln)
        outcomes={}
        for c in candidates:
            key=(None,*c['receiver']);cov,ln=matrices[key]
            assert ln>c['line']
            outcomes[key]=dict(sender=c['sender'],cov=c['cov'],converted=c['cov'],factor_cov=cov,
                               outcome='constructed',outcome_line=ln)
        assert len(matrices)==len(outcomes)
    rows=verify_candidates(candidates,outcomes,wire[sender])
    checks[kind]=dict(aligned_calls=len(rows),constructed=sum(r['outcome']=='constructed' for r in rows),
        pre_stage_rejected=sum(r['outcome']=='drop_before_stage' for r in rows),
        all_wire_covariances_exact=True,all_constructed_covariances_exact=True,
        causal_endpoint_cutoffs_valid=True,rows=rows)
result=dict(passed=True,tag='live160_on',receivers=checks,wire_ids={k:len(v) for k,v in wire.items()},
    caveat='Construction is not necessarily final admission: post-stage rollback is validated separately.')
with a.out.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({**result,'receivers':{k:{a:b for a,b in v.items() if a!='rows'} for k,v in checks.items()}},indent=2))
