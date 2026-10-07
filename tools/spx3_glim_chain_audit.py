#!/usr/bin/env python3
"""Compose frozen SPX-3 GLIM i-to-i+2 beliefs without any dynamics factors."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def pose_exp(xi):
    phi, rho = xi[:3], xi[3:]
    theta = np.linalg.norm(phi)
    k = skew(phi)
    if theta < 1e-8:
        v = np.eye(3) + 0.5 * k + (1.0 / 6.0) * k @ k
    else:
        v = (np.eye(3) + (1.0 - np.cos(theta)) / theta**2 * k
             + (theta - np.sin(theta)) / theta**3 * k @ k)
    return v @ rho, Rotation.from_rotvec(phi)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--beliefs', type=Path, required=True)
    parser.add_argument('--native', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()

    with args.native.open(newline='') as stream:
        native = list(csv.DictReader(stream))
    with args.beliefs.open(newline='') as stream:
        beliefs = list(csv.DictReader(stream))

    poses = {}
    stamps = {}
    native_poses = {}
    for index, row in enumerate(native):
        p = np.array([float(row[f'field.pose.pose.position.{c}']) for c in 'xyz'])
        q = np.array([float(row[f'field.pose.pose.orientation.{c}']) for c in 'xyzw'])
        native_poses[index] = (p, Rotation.from_quat(q))
    native_edge_trans = []
    native_edge_rot = []
    for index in (0, 1):
        row = native[index]
        position = np.array([float(row[f'field.pose.pose.position.{c}']) for c in 'xyz'])
        quaternion = np.array([float(row[f'field.pose.pose.orientation.{c}']) for c in 'xyzw'])
        poses[index] = (position, Rotation.from_quat(quaternion))

    for edge in beliefs:
        start = int(edge['from_index'])
        end = int(edge['to_index'])
        if start not in poses or end in poses:
            raise ValueError(f'not a two-tree forward chain: {start} -> {end}')
        if start in stamps and stamps[start] != int(edge['from_timestamp_ns']):
            raise ValueError('source timestamp inconsistency')
        stamps[start] = int(edge['from_timestamp_ns'])
        stamps[end] = int(edge['to_timestamp_ns'])
        p0, r0 = poses[start]
        mu = np.array([float(edge[f'relative_mu_{i}']) for i in range(6)])
        dp, dr = pose_exp(mu)
        pn0, rn0 = native_poses[start]
        pn1, rn1 = native_poses[end]
        native_edge_trans.append(float(np.linalg.norm(dp - rn0.inv().apply(pn1 - pn0))))
        native_edge_rot.append(float(np.rad2deg((dr.inv() * rn0.inv() * rn1).magnitude())))
        poses[end] = (p0 + r0.apply(dp), r0 * dr)

    assert len(poses) == len(beliefs) + 2
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('w') as stream:
        for index, (position, rotation) in sorted(poses.items()):
            timestamp = stamps[index]
            data = [timestamp * 1e-9, *position, *rotation.as_quat()]
            stream.write(' '.join(f'{value:.17g}' for value in data) + '\n')
    with args.out.with_name('native_support.tum').open('w') as stream:
        for index, (position, rotation) in sorted(native_poses.items()):
            if index not in stamps:
                continue
            data = [stamps[index] * 1e-9, *position, *rotation.as_quat()]
            stream.write(' '.join(f'{value:.17g}' for value in data) + '\n')
    print(f'composed {len(beliefs)} exact factors over {len(poses)} poses in two gauge-fixed chains')
    translation_separation = [float(np.linalg.norm(poses[i][0] - native_poses[i][0])) for i in sorted(poses)]
    rotation_separation = [float(np.rad2deg((poses[i][1].inv() * native_poses[i][1]).magnitude())) for i in sorted(poses)]
    def stats(values):
        return {'median': float(np.median(values)), 'rms': float(np.sqrt(np.mean(np.square(values)))),
                'p95': float(np.percentile(values, 95)), 'max': float(np.max(values))}
    report = {'belief_vs_native_edge_translation_m': stats(native_edge_trans),
              'belief_vs_native_edge_rotation_deg': stats(native_edge_rot),
              'composition_vs_native_position_m': stats(translation_separation),
              'composition_vs_native_orientation_deg': stats(rotation_separation)}
    args.out.with_suffix('.diagnostics.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, sort_keys=True))


if __name__ == '__main__':
    main()
