#!/usr/bin/env python3
"""Read-only EIG-2 GLIM edge/trajectory preservation diagnostic.

Requires NumPy and SciPy. Writes diagnostic CSV/JSON and two TUM trajectories;
does not modify any source trajectory or belief export.
"""

import argparse
import csv
import json
import math
from collections import defaultdict, deque
from decimal import Decimal
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def timestamp_ns(value):
    return int((Decimal(value) * Decimal(1_000_000_000)).to_integral_value())


def pose(position, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = position
    return result


def inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def skew(vector):
    return np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]], [-vector[1], vector[0], 0]])


def exp_se3(tangent):
    phi, rho = tangent[:3], tangent[3:]
    theta = np.linalg.norm(phi)
    a = skew(phi)
    if theta < 1e-8:
        jacobian = np.eye(3) + 0.5 * a + a @ a / 6
    else:
        jacobian = np.eye(3) + (1 - math.cos(theta)) / theta**2 * a + (theta - math.sin(theta)) / theta**3 * a @ a
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(phi).as_matrix()
    result[:3, 3] = jacobian @ rho
    return result


def log_se3(transform):
    phi = Rotation.from_matrix(transform[:3, :3]).as_rotvec()
    theta = np.linalg.norm(phi)
    a = skew(phi)
    if theta < 1e-8:
        jacobian = np.eye(3) + 0.5 * a + a @ a / 6
    else:
        jacobian = np.eye(3) + (1 - math.cos(theta)) / theta**2 * a + (theta - math.sin(theta)) / theta**3 * a @ a
    return np.r_[phi, np.linalg.solve(jacobian, transform[:3, 3])]


def read_tum(path):
    times, poses = [], []
    for line in path.read_text().splitlines():
        row = line.split()
        times.append(timestamp_ns(row[0]))
        poses.append(pose(np.array([float(x) for x in row[1:4]]), [float(x) for x in row[4:8]]))
    return times, poses


def read_online_csv(path):
    rows = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            index = int(row["field.header.seq"])
            p = [float(row[f"field.pose.pose.position.{axis}"]) for axis in "xyz"]
            q = [float(row[f"field.pose.pose.orientation.{axis}"]) for axis in "xyzw"]
            rows[index] = (int(row["field.header.stamp"]), pose(p, q), int(row["%time"]))
    return rows


def read_beliefs(path):
    result = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            tangent = np.array([float(row[f"relative_mu_{i}"]) for i in range(6)])
            result.append((int(row["ordinal"]), int(row["from_index"]), int(row["to_index"]), int(row["from_timestamp_ns"]), int(row["to_timestamp_ns"]), int(row["record_timestamp_ns"]), exp_se3(tangent)))
    return result


def stats(values):
    values = np.asarray(values)
    return {"count": len(values), "median": float(np.median(values)), "rms": float(np.sqrt(np.mean(values**2))), "p95": float(np.percentile(values, 95)), "maximum": float(np.max(values))}


def write_tum(path, times, poses):
    with path.open("w") as stream:
        for time, transform in zip(times, poses):
            q = Rotation.from_matrix(transform[:3, :3]).as_quat()
            p = transform[:3, 3]
            stream.write(f"{Decimal(time) / Decimal(1_000_000_000)} {p[0]:.12f} {p[1]:.12f} {p[2]:.12f} {q[0]:.12f} {q[1]:.12f} {q[2]:.12f} {q[3]:.12f}\n")


def pose_dict(transform):
    return {"position": transform[:3, 3].tolist(),
            "quaternion_xyzw": Rotation.from_matrix(transform[:3, :3]).as_quat().tolist()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--beliefs", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    times, native = read_tum(args.run_dir / "odom_imu.txt")
    online = read_online_csv(args.run_dir / "glim_odometry.csv")
    beliefs = read_beliefs(args.beliefs)
    if sorted(online) != list(range(len(native))):
        raise RuntimeError("online indices do not match native rows")
    adjacent = defaultdict(list)
    endpoint_time_errors_ns = []
    for ordinal, i, j, ti, tj, record, measurement in beliefs:
        if i >= len(native) or j >= len(native):
            raise RuntimeError(f"edge {ordinal} index out of range")
        endpoint_time_errors_ns.extend([abs(times[i] - ti), abs(times[j] - tj)])
        if endpoint_time_errors_ns[-2] > 500 or endpoint_time_errors_ns[-1] > 500:
            raise RuntimeError(f"edge {ordinal} timestamp mismatch beyond 500 ns: {(times[i], times[j])} vs {(ti, tj)}")
        adjacent[i].append(j)
        adjacent[j].append(i)
    components = []
    unvisited = set(range(len(native)))
    while unvisited:
        root = min(unvisited)
        queue = deque([root])
        unvisited.remove(root)
        component = []
        while queue:
            current = queue.popleft()
            component.append(current)
            for neighbor in adjacent[current]:
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    queue.append(neighbor)
        components.append(sorted(component))

    native_edges = [(ordinal, i, j, ti, tj, record, inverse(native[i]) @ native[j]) for ordinal, i, j, ti, tj, record, _ in beliefs]

    def reconstruct(edges):
        result = [None] * len(native)
        for component in components:
            result[component[0]] = native[component[0]].copy()  # diagnostic gauge, not belief information
        by_start = defaultdict(list)
        for _, i, j, _, _, _, measurement in edges:
            by_start[i].append((j, measurement))
        for index in range(len(native)):
            if result[index] is None:
                raise RuntimeError(f"unreached endpoint {index}")
            for j, measurement in by_start[index]:
                if result[j] is not None:
                    raise RuntimeError("cycle or reused endpoint")
                result[j] = result[index] @ measurement
        return result

    native_recon = reconstruct(native_edges)
    exported_recon = reconstruct(beliefs)
    per_edge = []
    for ordinal, i, j, ti, tj, record, measurement in beliefs:
        native_delta = inverse(native[i]) @ native[j]
        online_delta = inverse(online[i][1]) @ online[j][1]
        error_native = log_se3(inverse(measurement) @ native_delta)
        error_online = log_se3(inverse(measurement) @ online_delta)
        per_edge.append({"ordinal": ordinal, "from_index": i, "to_index": j, "from_timestamp_ns": ti, "to_timestamp_ns": tj, "record_timestamp_ns": record, "native_translation_m": float(np.linalg.norm(error_native[3:])), "native_rotation_deg": float(np.linalg.norm(error_native[:3]) * 180 / np.pi), "online_translation_m": float(np.linalg.norm(error_online[3:])), "online_rotation_deg": float(np.linalg.norm(error_online[:3]) * 180 / np.pi)})
    online_position = []
    online_rotation = []
    online_time_mismatches = 0
    for index, native_pose in enumerate(native):
        online_time, online_pose, _ = online[index]
        online_time_mismatches += int(online_time != times[index])
        error = log_se3(inverse(native_pose) @ online_pose)
        online_position.append(float(np.linalg.norm(error[3:])))
        online_rotation.append(float(np.linalg.norm(error[:3]) * 180 / np.pi))

    def pose_errors(estimate):
        translation, rotation = [], []
        for a, b in zip(estimate, native):
            delta = log_se3(inverse(b) @ a)
            translation.append(np.linalg.norm(delta[3:]))
            rotation.append(np.linalg.norm(delta[:3]) * 180 / np.pi)
        return {"translation_m": stats(translation), "rotation_deg": stats(rotation)}

    summary = {
        "native_rows": len(native), "online_rows": len(online), "belief_rows": len(beliefs),
        "endpoint_timestamp_max_rounding_ns": max(endpoint_time_errors_ns),
        "components": [{"first_index": c[0], "last_index": c[-1], "nodes": len(c), "step": 2 if len(c) > 1 and c[1] - c[0] == 2 else None} for c in components],
        "online_timestamp_mismatches": online_time_mismatches,
        "online_vs_dump": {"translation_m": stats(online_position), "rotation_deg": stats(online_rotation)},
        "native_derived_reconstruction": pose_errors(native_recon),
        "exported_reconstruction_with_two_native_gauges": pose_errors(exported_recon),
        "export_vs_native_edge": {"translation_m": stats([row["native_translation_m"] for row in per_edge]), "rotation_deg": stats([row["native_rotation_deg"] for row in per_edge])},
        "export_vs_online_edge": {"translation_m": stats([row["online_translation_m"] for row in per_edge]), "rotation_deg": stats([row["online_rotation_deg"] for row in per_edge])},
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    with (args.out / "edge_disagreement.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(per_edge[0]))
        writer.writeheader()
        writer.writerows(per_edge)
    with (args.out / "worst20.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(per_edge[0]))
        writer.writeheader()
        writer.writerows(sorted(per_edge, key=lambda x: x["native_translation_m"], reverse=True)[:20])
    selected = set(row["ordinal"] for row in sorted(
        per_edge, key=lambda x: x["native_translation_m"], reverse=True)[:3])
    representative = []
    for ordinal, i, j, ti, tj, record, measurement in beliefs:
        if ordinal not in selected:
            continue
        representative.append({
            "ordinal": ordinal, "from_index": i, "to_index": j,
            "from_timestamp_ns": ti, "to_timestamp_ns": tj,
            "record_timestamp_ns": record,
            "exported_relative": pose_dict(measurement),
            "native_from_shutdown_dump": pose_dict(native[i]),
            "native_to_shutdown_dump": pose_dict(native[j]),
            "online_from_at_its_own_publication": pose_dict(online[i][1]),
            "online_to_at_its_own_publication": pose_dict(online[j][1]),
            "publication_time_from_pose": None,
            "publication_time_to_pose": None,
            "publication_endpoint_audit_available": False,
        })
    (args.out / "representative_edges.json").write_text(
        json.dumps(representative, indent=2) + "\n")
    write_tum(args.out / "native_derived_two_gauges.tum", times, native_recon)
    write_tum(args.out / "exported_two_native_gauges.tum", times, exported_recon)
    for component_id, component in enumerate(components):
        write_tum(args.out / f"native_component_{component_id}.tum", [times[i] for i in component], [native[i] for i in component])
        write_tum(args.out / f"exported_component_{component_id}.tum", [times[i] for i in component], [exported_recon[i] for i in component])
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
