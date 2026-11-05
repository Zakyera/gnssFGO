"""Read-only available-data/calibration inventory. Does NOT qualify a launch."""
import datetime
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
from rosbags.highlevel import AnyReader

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D = W / 'src/datasets/GrandTour/LEICA-2'
M = '2024-11-25-16-36-19'
P103 = W / 'runs/eth1_prior_fix_live_cpv103_20261103/prepared/on'

def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def cfg(path):
    return yaml.safe_load(path.read_text().replace('%YAML:1.0', ''))

assert not (H / 'available_data_review.json').exists()
assert not (H / 'start.json').exists()
inventory = {}
for suffix in ('hesai', 'alphasense', 'cpt7_imu', 'cpt7_ie_tc', 'tf_minimal', 'alphasense_imu'):
    path = D / f'{M}_{suffix}.bag'
    row = dict(path=str(path), exists=path.is_file())
    if path.is_file():
        with AnyReader([path]) as reader:
            row.update(bytes=path.stat().st_size, mtime_ns=path.stat().st_mtime_ns,
                       first_record_ns=reader.start_time, last_record_exclusive_ns=reader.end_time,
                       connections=[dict(topic=c.topic, type=c.msgtype, count=c.msgcount)
                                    for c in reader.connections])
    inventory[suffix] = row
assert inventory['alphasense_imu']['exists']
assert not any(c['type'] == 'sensor_msgs/msg/Imu' for c in inventory['alphasense']['connections'])

edges = {}
tf = D / f'{M}_tf_minimal.bag'
with AnyReader([tf]) as reader:
    for conn, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic == '/tf_static']):
        for tr in reader.deserialize(raw, conn.msgtype).transforms:
            v = tr.transform
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([v.rotation.x, v.rotation.y, v.rotation.z, v.rotation.w]).as_matrix()
            T[:3, 3] = [v.translation.x, v.translation.y, v.translation.z]
            key = (tr.header.frame_id, tr.child_frame_id)
            if key in edges:
                assert np.max(np.abs(edges[key] - T)) < 1e-12
            edges[key] = T

def transform(start, end):
    queue = [(start, np.eye(4))]
    seen = {start}
    for node, T in queue:
        if node == end:
            return T
        for (parent, child), edge in edges.items():
            if parent == node and child not in seen:
                seen.add(child)
                queue.append((child, T @ edge))
            if child == node and parent not in seen:
                seen.add(parent)
                queue.append((parent, T @ np.linalg.inv(edge)))
    raise ValueError((start, end))

cameras = {}
with AnyReader([D / f'{M}_alphasense.bag']) as reader:
    for side in ('Left', 'Right'):
        topic = f'/boxi/alphasense/front_{side.lower()}/camera_info'
        conn, _, raw = next(reader.messages(connections=[c for c in reader.connections if c.topic == topic]))
        msg = reader.deserialize(raw, conn.msgtype)
        path = P103 / f'input_profiles/kimera_profile/{side}CameraParams.yaml'
        profile = cfg(path)
        mission_T = transform('alphasense_imu', f'alphasense_front_{side.lower()}')
        profile_T = np.asarray(profile['T_BS']['data']).reshape(4, 4)
        k = np.asarray(msg.K).reshape(3, 3)
        intrinsics = [k[0, 0], k[1, 1], k[0, 2], k[1, 2]]
        errors = dict(extrinsic_max_abs=float(np.max(np.abs(mission_T - profile_T))),
                      intrinsics_max_abs=float(np.max(np.abs(np.array(intrinsics) - profile['intrinsics']))),
                      distortion_max_abs=float(np.max(np.abs(np.asarray(msg.D) - profile['distortion_coefficients']))))
        cameras[side] = dict(topic=topic, frame=msg.header.frame_id,
                             resolution=[int(msg.width), int(msg.height)],
                             distortion_model=msg.distortion_model,
                             T_alpha_camera=mission_T.tolist(), comparison_to_CP103=errors,
                             profile_sha256=sha(path))

sensor_path = P103 / 'input_profiles/glim_config/config_sensors.json'
clean = re.sub(r'/\*.*?\*/', '', sensor_path.read_text(), flags=re.S)
clean = re.sub(r'//[^\n]*', '', clean)
pose = json.loads(clean)['sensors']['T_lidar_imu']
saved_T = np.eye(4)
saved_T[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
saved_T[:3, 3] = pose[:3]
lidar_T = transform('hesai_lidar', 'cpt7_imu')
lidar_error = float(np.max(np.abs(saved_T - lidar_T)))

seals = {}
for name in ('eth1_prior_fix_live_cpv103_20261103', 'eth1_covariance_provenance_cpv107_20261104'):
    path = E / name / 'complete.json'
    seal = json.loads(path.read_text())
    assert seal['passed']
    for file, digest in seal['output_sha256'].items():
        assert sha(file) == digest, file
    seals[name] = dict(seal_sha256=sha(path), verified_output_hashes=len(seal['output_sha256']))
dso = P103 / 'private_build/libkimera_vio.so'
assert sha(dso) == '985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09'
result = dict(created=datetime.datetime.now().astimezone().isoformat(), launch_ready=False,
              status='inventory_present_pending_raw_audit', captures_started=0,
              inventory=inventory, free_bytes=shutil.disk_usage(D).free,
              TF=dict(path=str(tf), sha256=sha(tf), static_edges=len(edges)),
              cameras=cameras, T_alpha_cpt7=transform('alphasense_imu', 'cpt7_imu').tolist(),
              T_cpt7_prism=transform('cpt7_imu', 'prism').tolist(),
              T_alpha_prism=transform('alphasense_imu', 'prism').tolist(),
              T_hesai_cpt7=lidar_T.tolist(), CP103_lidar_extrinsic_max_abs=lidar_error,
              CP103_candidate_DSO_sha256=sha(dso), preserved_terminal_outputs=seals,
              limitation='Inventory and calibration comparisons only; raw selected-window audit and launch admission still required')
with (H / 'available_data_review.json').open('x') as stream:
    json.dump(result, stream, indent=2, allow_nan=False)
print(json.dumps(dict(status=result['status'], cameras=cameras,
                      lidar_extrinsic_max_abs=lidar_error, preserved_terminal_outputs=seals,
                      free_GiB=result['free_bytes'] / 2**30), indent=2))
