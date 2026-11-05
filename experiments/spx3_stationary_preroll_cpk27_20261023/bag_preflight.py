"""Read-only SPX-3 inputs/window audit; no estimator or ROS playback."""
from pathlib import Path
import hashlib
import json
import math
import struct
from rosbags.rosbag1 import Reader

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D = W/'src/datasets/GrandTour/SPX-3'
MISSION = '2024-11-02-17-43-10'
OLD = W/'runs/gnssfgo_spx3_local_cov60_cpr5_20261019/prepared/preflight/validation.json'
TOPICS = {'hesai': ['/boxi/hesai/points'], 'cpt7_imu': ['/boxi/cpt7/imu'],
          'alphasense': ['/boxi/alphasense/front_left/image_raw/compressed',
                        '/boxi/alphasense/front_right/image_raw/compressed'],
          'cpt7_ie_tc': ['/boxi/inertial_explorer/tc/odometry']}

assert not (H/'bag_preflight.json').exists(), 'Do not overwrite an input audit'
previous = json.loads(OLD.read_text())
bags = {}; stamps = {}
for kind, topics in TOPICS.items():
    path = D/f'{MISSION}_{kind}.bag'
    with Reader(path) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        assert sorted(c.topic for c in connections) == sorted(topics)
        for c in connections:
            stamps[c.topic] = [entry.time for entry in reader.indexes[c.id]]
            assert stamps[c.topic] and all(b > a for a,b in zip(stamps[c.topic],stamps[c.topic][1:]))
        info = dict(path=str(path),bytes=path.stat().st_size,mtime_ns=path.stat().st_mtime_ns,
                    topics={c.topic:dict(type=c.msgtype,count=len(stamps[c.topic]),
                         first_record_ns=stamps[c.topic][0],last_record_ns=stamps[c.topic][-1]) for c in connections})
    with path.open('rb') as f: digest = hashlib.file_digest(f,'sha256').hexdigest()
    assert digest == previous['bag_sha256'][str(path)], ('bag changed',path)
    info['sha256'] = digest; bags[kind] = info
    print('Verified '+kind,flush=True)

epoch = min(stamps[t][0] for kind,topics in TOPICS.items() if kind != 'cpt7_ie_tc' for t in topics)
start = epoch + 29_000_000_000
last_hesai = stamps[TOPICS['hesai'][0]][-1]
duration_ns = 28_000_000_000
end = start+duration_ns
assert duration_ns == 28_000_000_000 and start == 1730565899872888651
selected = {t:[x for x in values if start<=x<end] for t,values in stamps.items()}
for t,values in selected.items():
    assert values
    bags[next(k for k,ts in TOPICS.items() if t in ts)]['topics'][t]['selected'] = dict(
        count=len(values),first_record_ns=values[0],last_record_ns=values[-1],
        maximum_record_gap_ns=max(b-a for a,b in zip(values,values[1:])))
last_hesai = selected[TOPICS['hesai'][0]][-1]
assert selected[TOPICS['cpt7_imu'][0]][-1] >= last_hesai-20_000_000
assert stamps[TOPICS['cpt7_ie_tc'][0]][0] < start and stamps[TOPICS['cpt7_ie_tc'][0]][-1] >= end
headers = {}
with Reader(D/f'{MISSION}_alphasense.bag') as reader:
    conns = [c for c in reader.connections if c.topic in TOPICS['alphasense']]
    for c in conns: headers[c.topic] = []
    for connection,record_time,raw in reader.messages(connections=conns,start=start,stop=end):
        _,sec,nsec = struct.unpack_from('<III',raw)
        stamp = sec*1_000_000_000+nsec
        assert abs(stamp-record_time) < 50_000_000
        headers[connection.topic].append(stamp)
left,right = (headers[t] for t in TOPICS['alphasense'])
assert left == right, 'Stereo header timestamps differ'
assert len(left) == len(selected[TOPICS['alphasense'][0]])
result = dict(passed=True,mission=MISSION,bag_files=bags,
    window=dict(selected_topic_epoch_ns=epoch,start_ns=start,end_ns=end,last_hesai_record_ns=last_hesai,
                bag_offset_s=29.0,duration_s=duration_ns*1e-9,seed_clock_seconds=start//1_000_000_000-1),
    stereo_pairs=len(left),frontend_min=len(left)-3,frontend_max=len(left),
    stereo_headers_identical=True,reference_covers_complete_window=True,
    earlier_134s_crop_not_reused=True,source_parameters_changed=False)
with (H/'bag_preflight.json').open('x') as f: json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
