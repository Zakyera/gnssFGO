"""Read retained ROS artifacts, export exact messages/reference for diagnosis."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import rosbag

ROOT = Path('/workspace/cbs_gtsam4.3')
sys.path.insert(0, str(ROOT/'src/cbsms/tools'))
from score_grandtour_relative_pose import load_reference_bag
OFF = ROOT/'runs/gnssfgo_eth1_active_window40_off_20261008'
ON = ROOT/'runs/gnssfgo_eth1_active_window40_prepared_20261008'
OUT = OFF/'insertion_retirement_audit_20261008'
bag = ON/'pilot_on_source/recordings/cbs.bag'
reference = ROOT/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag'
assert not (OUT/'wire_beliefs.json').exists()
assert not (OUT/'reference.npz').exists()
beliefs = []
outputs = []
with rosbag.Bag(str(bag), 'r') as stream:
    for topic, message, stamp in stream.read_messages(topics=['/kimera/cbs/odom_belief_in', '/kimera_vio_ros/odometry']):
        if topic.endswith('odometry'):
            outputs.append(dict(update=int(message.header.seq), record_ns=stamp.to_nsec(), header_ns=message.header.stamp.to_nsec()))
        else:
            for b in message.beliefs:
                assert b.source_agent == ord('g')
                beliefs.append(dict(i=int(b.from_pose_index), j=int(b.to_pose_index),
                                    revision=int(b.revision), ti=b.from_stamp_sec, tj=b.to_stamp_sec,
                                    record_ns=stamp.to_nsec(), header_ns=b.header.stamp.to_nsec(),
                                    mu=list(b.relative_mu), covariance=list(b.covariance), relax_factor=b.relax_factor))
t,x,q,frames,children = load_reference_bag(reference, '/boxi/inertial_explorer/tc/odometry')
sel = (t >= 1727775007.9) & (t <= 1727775048.1)
assert frames == ['enu_origin'] and children == ['cpt7_imu']
assert np.diff(t[sel]).max() < .006
with (OUT/'reference.npz').open('xb') as f:
    np.savez(f, times=t[sel], positions=x[sel], quaternions=q[sel])
with (OUT/'wire_beliefs.json').open('x') as f:
    json.dump(dict(beliefs=beliefs, kimera_outputs=outputs), f, indent=2)
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
with (OUT/'extraction_provenance.json').open('x') as f:
    json.dump(dict(bag=str(bag), bag_sha256=sha(bag), reference=str(reference), reference_sha256=sha(reference),
                   beliefs=len(beliefs), kimera_outputs=len(outputs), reference_rows=int(sel.sum()),
                   caveat='IE-TC is a dataset GNSS/INS reference sharing CPT7, not independent prism GT.'), f, indent=2)
print('Extracted',len(beliefs),'actual GLIM wire beliefs,',len(outputs),'Kimera ROS outputs,',int(sel.sum()),'reference poses; no estimator run.')
