#!/usr/bin/env python3
"""Lossless diagnostic-log transport; no source-estimator or covariance changes."""
import math
import re
from decimal import Decimal
from pathlib import Path

def stamp_ns(seconds):
    return int((Decimal(seconds) * Decimal(1000000000)).to_integral_value())


def parse_online_pose(header, line):
    """Publish shared pose fields without assuming IMU biases exist in every model."""
    prefix = ['state_index', 'timestamp_ns', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']
    fields = line.split(',')
    if header is None or header[:9] != prefix or len(set(header)) != len(header) or len(fields) != len(header):
        raise ValueError('invalid receiver online-state row/header')
    values = list(map(float, fields[2:]))
    if not all(map(math.isfinite, values)):
        raise ValueError('nonfinite receiver state')
    stamp = int(fields[1])
    if stamp <= 0:
        raise ValueError('invalid receiver timestamp')
    return stamp, values[:3], values[3:7]


def parse_belief(kind, line):
    """Return the exact logged mean/covariance, or None for unrelated log lines."""
    import numpy as np
    if ('GLIM_CBS_REVISION_ERROR_ROW,' in line or 'KIMERA_CBS_REVISION_EDGE_ERROR_ROW,' in line
            or 'Kimera CBS getOdometryBeliefs failed' in line):
        raise ValueError('source revision extraction failed: '+line.strip())
    markers = {'glim': ('GLIM_CBS_EXPORT_BELIEF_ROW,', 'GLIM_CBS_REVISION_BELIEF_ROW,'),
               'kimera': ('KIMERA_CBS_REVISION_EDGE_ROW,',)}
    if kind not in markers:
        raise ValueError('unsupported source logger '+kind)
    marker = next((m for m in markers[kind] if m in line), None)
    if marker is None:
        return None
    fields = re.split('[\x1b\r\n]', line.split(marker, 1)[1])[0].split(',')
    if kind == 'glim':
        if len(fields) != 10:
            raise ValueError('invalid GLIM revision row width')
        source, i, j, rev, record, update, ti, tj, mu, cov = fields
        mean = [float(x) for x in mu.split(';')]
        matrix = [[float(x) for x in row.split(':')] for row in cov.split(';')]
    else:
        if len(fields) != 64:
            raise ValueError('invalid Kimera revision row width')
        update, source, i, j, ti, tj, rev, status = fields[:8]
        record = None
        if status not in ('first', 'active'):
            raise ValueError('unknown Kimera snapshot status')
        mean = list(map(float, fields[22:28]))
        matrix = np.array(list(map(float, fields[28:64]))).reshape(6, 6).tolist()
    covariance = np.asarray(matrix)
    if len(mean) != 6 or covariance.shape != (6, 6) or not all(map(math.isfinite, mean)) \
            or not np.isfinite(covariance).all():
        raise ValueError('invalid/nonfinite source belief')
    if np.max(np.abs(covariance-covariance.T)) > 1e-12 or np.linalg.eigvalsh(covariance)[0] <= 0:
        raise ValueError('source covariance is not symmetric positive definite')
    ti, tj = stamp_ns(ti), stamp_ns(tj)
    if ti >= tj:
        raise ValueError('invalid source endpoint timestamps')
    return dict(i=int(i), j=int(j), ti=ti, tj=tj, source_revision=int(rev),
                logger_source=source, source_record=record, source_update=update,
                mean=mean, covariance=covariance.reshape(-1).tolist())


class Versions:
    """Transport sequence numbers, not an assumption about logger version resets.

    Some Kimera snapshots repeat source revision zero with a different payload.
    Keep every changed snapshot in observed order and retain the raw version in
    the receipt audit. Repeated identical payloads are deduplicated, not factors.
    """
    def __init__(self):
        self.ids = {}

    def accept(self, source, session, belief, arrival):
        if arrival < belief['tj']:
            raise ValueError('belief endpoint is in the future of ROS receipt clock')
        key = source, session, belief['i'], belief['j']
        payload = tuple(belief['mean']+belief['covariance'])
        times = belief['ti'], belief['tj']
        previous = self.ids.get(key)
        if previous:
            ordinal, revision, old_times, old_payload = previous
            if times != old_times:
                raise ValueError('stable source ID changed endpoint timestamps')
            if payload == old_payload:
                return None
            revision += 1
        else:
            ordinal, revision = len(self.ids), 0
        self.ids[key] = ordinal, revision, times, payload
        return ['B', arrival, source, session, ordinal, revision, belief['i'], belief['j'],
                *times, *payload]


class LineTail:
    """Read only completed log rows; rotation/truncation is not silently ignored."""
    def __init__(self, path):
        self.path = Path(path)
        self.file = self.path.open('rb')
        self.identity = self.path.stat().st_ino
        self.partial = b''

    def read(self):
        if self.path.stat().st_ino != self.identity or self.path.stat().st_size < self.file.tell():
            raise RuntimeError('source log replaced/truncated: '+str(self.path))
        data = self.partial+self.file.read()
        parts = data.split(b'\n')
        self.partial = parts.pop()
        return [line.decode('utf-8', errors='strict') for line in parts]


def validate_config(config):
    if config.get('receiver_model', 'imu_wnoj') not in ('imu_wnoj', 'source_wnoj', 'source_wnoa'):
        raise ValueError('receiver_model must be imu_wnoj, source_wnoj or source_wnoa')
    # Optional executable environment is explicit and retained in the manifest.
    # It does not change the established production launch defaults.
    environment = config.get('receiver_environment', {})
    allowed = {'LD_LIBRARY_PATH', 'OMP_NUM_THREADS', 'WNOJ_STRICT_PUBLICATION',
               'WNOJ_INCREMENTAL_PROFILE'}
    if not isinstance(environment, dict) or any(
            key not in allowed or not isinstance(value, str) or '\x00' in value
            for key, value in environment.items()):
        raise ValueError('invalid receiver_environment')
    policy = config.get('receiver_publication_policy', 'strict_nominal')
    if policy not in ('strict_nominal', 'fl4a_fixed_lag_incremental'):
        raise ValueError('unknown receiver_publication_policy')
    if policy == 'fl4a_fixed_lag_incremental':
        # Private FL4a adapter only. No nonlinear stationarity certificate is claimed.
        expected = '/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live120_on_fl4v_20261017/prepared/receiver_fl4e.sh'
        if config.get('receiver_model') != 'source_wnoa' or config.get('receiver_binary') != expected:
            raise ValueError('FL4a policy requires the pinned fixed-lag wrapper')
        if config.get('receiver_fixed_lag_seconds') != 7 or any(key.startswith('WNOJ_') for key in environment):
            raise ValueError('FL4a requires 7s lag and cannot claim strict nominal publication')
    if config.get('receiver_model') == 'source_wnoa' and policy == 'strict_nominal' and (
            environment.get('WNOJ_STRICT_PUBLICATION') != '1' or
            environment.get('WNOJ_INCREMENTAL_PROFILE') != 'nominal'):
        raise ValueError('source_wnoa requires the validated strict nominal publication policy')
    required = ('sequence', 'ros_master_uri', 'imu_topic', 'imu_frame', 'sources')
    for field in required:
        if not config.get(field):
            raise ValueError('missing explicit config field '+field)
    if config.get('source_protocol') != 'classic':
        raise ValueError('this setup requires classic source CBS, never live_revision')
    ids = set()
    for source in config['sources']:
        for field in ('id', 'session'):
            if not re.fullmatch('[A-Za-z0-9_-]+', source.get(field, '')):
                raise ValueError('invalid source '+field)
        canonical = {'g': 'glim', 'k': 'kimera'}.get(source['id'], source['id'])
        identity = canonical
        if identity in ids:
            raise ValueError('duplicate active source ID; restart in a new capture/session')
        ids.add(identity)
        if source.get('kind') not in ('glim', 'kimera') or not source.get('log_glob'):
            raise ValueError('source must identify an existing diagnostic logger and log_glob')
        if source.get('belief_body_frame') != config['imu_frame']:
            raise ValueError('source beliefs must already use the calibrated central IMU body frame')
    if config.get('support_period_ns', 100000000) != 100000000 \
            or config.get('startup_samples', 59) != 59:
        raise ValueError('this frozen preset uses 100 ms supports and 59 startup samples')
