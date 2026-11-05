"""Read-only exact five-topic window contract, host and actual Noetic runtime.
Does not publish messages, launch a master, or run an estimator/receiver.
"""
import argparse,datetime,hashlib,json,math,re,sys
from decimal import Decimal
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--host',action='store_true');parser.add_argument('--noetic',action='store_true');args=parser.parse_args()
assert args.host != args.noetic
P=Path(__file__).resolve().parent
H=Path('/home/yeranis/repos/gnssFGO/experiments/leica2_split_imu_live_pair_cpv112_20261105_attempt1')
a=json.loads((P/'input_audit.json').read_text());want=json.loads((P/'expected_headers.json').read_text())
base=min(v['start_ns'] for v in a['bag_files'].values());start=a['epoch_ns'];end=a['end_ns']
offset_ns=start-base;offset=format(Decimal(offset_ns)/Decimal(10**9),'.9f')
assert (base,start,end,offset)==(1732549028815072565,1732549034915071665,1732549463665071665,'6.099999100')
assert end-start==428750000000 and a['offset_s']==6.0
assert math.floor(float(offset)*10**9+.5)==offset_ns
for mode in ('off','on'):
    env=(P/mode/'source.env').read_text();runner=(P/mode/'source_runner_on.sh').read_text()
    assert re.search(r'^export CBSMS_BAG_START_SEC=6\.0$',env,re.M)
    assert re.search(r'^export CBSMS_BAG_PLAYBACK_START_SEC='+re.escape(offset)+r'$',env,re.M)
    assert '--start="${playback_start_sec}"' in runner and '--start="${bag_start_sec}"' not in runner
    assert '"${playback_start_sec}" == 6.099999100' in runner
    assert hashlib.sha256((P/mode/'expected_headers.json').read_bytes()).digest()==hashlib.sha256((P/'expected_headers.json').read_bytes()).digest()
old_start=base+6000000000;old_end=old_start+428750000000
stamps={topic:[] for topic in want};origins=[];selected=[]
if args.host:
    sys.path.insert(0,'/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
    from rosbags.rosbag1 import Reader
    for v in a['bag_files'].values():
        with Reader(Path(v['path'])) as r:
            origins.append(r.start_time);assert r.start_time==v['start_ns']
            conns=[c for c in r.connections if c.topic in want]
            selected.append(next(r.messages(connections=conns))[1])
            for c,t,raw in r.messages(connections=conns,start=old_start,stop=end):stamps[c.topic].append(t)
else:
    import rosbag,genpy
    assert json.loads((P/'playback_window.json').read_text())['passed']
    for v in a['bag_files'].values():
        path=v['path'].replace('/home/yeranis/repos/V4RL/cbs_gtsam4.3','/workspace/cbs_gtsam4.3')
        with rosbag.Bag(path,'r') as r:
            origins.append(next(r.read_messages(raw=True)).timestamp.to_nsec())
            assert origins[-1]==v['start_ns']
            selected.append(next(r.read_messages(topics=list(want),raw=True)).timestamp.to_nsec())
            for topic,raw,t in r.read_messages(topics=list(want),start_time=genpy.Time(old_start//10**9,old_start%10**9),end_time=genpy.Time(end//10**9,end%10**9),raw=True):
                if old_start<=t.to_nsec()<end:stamps[topic].append(t.to_nsec())
assert min(origins)==base and min(selected)==a['playback_epoch_ns']
counts={};negative={}
for topic,values in stamps.items():
    actual=[t for t in values if start<=t<end]
    wrong=[t for t in values if old_start<=t<old_end]
    assert actual==want[topic],(topic,len(actual),len(want[topic]))
    assert all(x<y for x,y in zip(actual,actual[1:])),topic
    counts[topic]=dict(count=len(actual),first_ns=actual[0],last_ns=actual[-1],exact_expected_timestamps=True)
    negative[topic]=dict(old_offset_rejected=wrong!=want[topic],old_count=len(wrong),unexpected_early=len(set(wrong)-set(want[topic])),missing_tail=len(set(want[topic])-set(wrong)))
assert all(v['old_offset_rejected'] for v in negative.values())
assert counts['/boxi/hesai/points']['last_ns']==1732549463660625935
out=dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),runtime='host rosbags' if args.host else 'actual Noetic rosbag',all_bag_epoch_ns=base,selected_topic_epoch_ns=a['playback_epoch_ns'],selected_offset_s='6.0',rosbag_start_s=offset,start_ns=start,end_ns=end,origin_difference_ns=a['playback_epoch_ns']-base,counts=counts,negative_old_offset=negative,source_runs=0,receiver_runs=0,playback_runs=0,header_basis='Original raw audit establishes header=record timestamp; original frozen bags rechecked at admission')
if args.host:
    for path in (P/'playback_window.json',H/'playback_window_test.json'):
        with path.open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
