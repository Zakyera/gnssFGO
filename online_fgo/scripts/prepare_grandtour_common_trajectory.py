#!/usr/bin/env python3
"""Put native GLIM, native Kimera, and fused poses on one timestamp grid."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in quaternion)


def slerp(first, second, alpha):
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(
            tuple(a + alpha * (b - a) for a, b in zip(first, second))
        )
    angle = math.acos(dot)
    denominator = math.sin(angle)
    left_weight = math.sin((1.0 - alpha) * angle) / denominator
    right_weight = math.sin(alpha * angle) / denominator
    return tuple(
        left_weight * a + right_weight * b for a, b in zip(first, second)
    )


def load_tum(path):
    trajectory = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            values = [float(value) for value in stripped.split()]
            if len(values) < 8:
                raise ValueError(f"{path}:{line_number}: expected at least 8 columns")
            trajectory.append((values[0], tuple(values[1:4]), tuple(values[4:8])))
    return sorted(trajectory, key=lambda pose: pose[0])


def load_odometry_csv(path):
    trajectory = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            trajectory.append(
                (
                    float(row["field.header.stamp"]) * 1.0e-9,
                    (
                        float(row["field.pose.pose.position.x"]),
                        float(row["field.pose.pose.position.y"]),
                        float(row["field.pose.pose.position.z"]),
                    ),
                    (
                        float(row["field.pose.pose.orientation.x"]),
                        float(row["field.pose.pose.orientation.y"]),
                        float(row["field.pose.pose.orientation.z"]),
                        float(row["field.pose.pose.orientation.w"]),
                    ),
                )
            )
    return sorted(trajectory, key=lambda pose: pose[0])


def interpolate(trajectory, timestamp, max_gap_sec):
    low = 0
    high = len(trajectory)
    while low < high:
        middle = (low + high) // 2
        if trajectory[middle][0] < timestamp:
            low = middle + 1
        else:
            high = middle
    right = low
    if (
        right < len(trajectory)
        and abs(trajectory[right][0] - timestamp) <= 1.0e-7
    ):
        time, position, quaternion = trajectory[right]
        return (
            timestamp,
            position,
            normalize_quaternion(quaternion),
            abs(time - timestamp),
        )
    if right == 0 or right == len(trajectory):
        return None
    left = right - 1
    interval = trajectory[right][0] - trajectory[left][0]
    if interval <= 0.0 or interval > max_gap_sec:
        return None
    alpha = (timestamp - trajectory[left][0]) / interval
    position = tuple(
        (1.0 - alpha) * a + alpha * b
        for a, b in zip(trajectory[left][1], trajectory[right][1])
    )
    quaternion = slerp(trajectory[left][2], trajectory[right][2], alpha)
    return timestamp, position, quaternion, min(
        timestamp - trajectory[left][0], trajectory[right][0] - timestamp
    )


def write_tum(path, poses):
    with path.open("w", encoding="utf-8") as stream:
        for timestamp, position, quaternion, _ in poses:
            values = (timestamp,) + position + quaternion
            stream.write(
                "{:.9f} {}\n".format(
                    values[0],
                    " ".join("{:.16f}".format(value) for value in values[1:]),
                )
            )


def path_length(poses):
    total = 0.0
    for first, second in zip(poses, poses[1:]):
        total += math.sqrt(
            sum((b - a) * (b - a) for a, b in zip(first[1], second[1]))
        )
    return total


def split_segments(trajectory, max_gap_sec):
    segments = []
    start = 0
    for index in range(1, len(trajectory)):
        if trajectory[index][0] - trajectory[index - 1][0] > max_gap_sec:
            segments.append(trajectory[start:index])
            start = index
    segments.append(trajectory[start:])
    return segments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--glim", required=True, type=Path)
    parser.add_argument("--kimera", required=True, type=Path)
    parser.add_argument("--fused", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--max-gap-sec", type=float, default=0.25)
    args = parser.parse_args()

    inputs = {
        "native_glim": load_tum(args.glim),
        "native_kimera": load_odometry_csv(args.kimera),
        "fused_gnssfgo": load_tum(args.fused),
    }
    if any(len(poses) < 2 for poses in inputs.values()):
        raise RuntimeError("each input must contain at least two poses")

    segments = {
        name: split_segments(poses, args.max_gap_sec)
        for name, poses in inputs.items()
    }
    largest_segments = {
        name: max(
            source_segments,
            key=lambda segment: segment[-1][0] - segment[0][0],
        )
        for name, source_segments in segments.items()
    }
    common_start = max(segment[0][0] for segment in largest_segments.values())
    common_end = min(segment[-1][0] for segment in largest_segments.values())
    target_times = [
        pose[0]
        for pose in inputs["native_glim"]
        if common_start <= pose[0] <= common_end
    ]
    if len(target_times) < 2:
        raise RuntimeError("common interval has fewer than two GLIM timestamps")

    args.out.mkdir(parents=True, exist_ok=True)
    output_paths = {}
    outputs = {}
    max_nearest_time_error = {}
    for name, trajectory in inputs.items():
        poses = [
            interpolate(trajectory, timestamp, args.max_gap_sec)
            for timestamp in target_times
        ]
        if any(pose is None for pose in poses):
            missing = sum(pose is None for pose in poses)
            raise RuntimeError(
                f"{name}: {missing} common timestamps cannot be interpolated"
            )
        outputs[name] = poses
        max_nearest_time_error[name] = max(pose[3] for pose in poses)
        output_path = args.out / f"{name}_common.tum"
        write_tum(output_path, poses)
        output_paths[name] = output_path

    summary = {
        "schema_version": 1,
        "purpose": "native GLIM/native Kimera/gnssFGO fusion common-grid evaluation",
        "common_grid_source": "native GLIM timestamps",
        "common_start_sec": target_times[0],
        "common_end_sec": target_times[-1],
        "common_duration_sec": target_times[-1] - target_times[0],
        "common_pose_count": len(target_times),
        "maximum_interpolation_interval_sec": args.max_gap_sec,
        "continuous_segment_policy": (
            "intersection of each source's longest segment"
        ),
        "inputs": {
            "native_glim": {
                "path": str(args.glim),
                "sha256": sha256(args.glim),
                "poses": len(inputs["native_glim"]),
            },
            "native_kimera": {
                "path": str(args.kimera),
                "sha256": sha256(args.kimera),
                "poses": len(inputs["native_kimera"]),
            },
            "fused_gnssfgo": {
                "path": str(args.fused),
                "sha256": sha256(args.fused),
                "poses": len(inputs["fused_gnssfgo"]),
            },
        },
        "outputs": {},
    }
    for name in inputs:
        summary["inputs"][name]["segments"] = [
            {
                "start_sec": segment[0][0],
                "end_sec": segment[-1][0],
                "duration_sec": segment[-1][0] - segment[0][0],
                "poses": len(segment),
            }
            for segment in segments[name]
        ]
    for name, output_path in output_paths.items():
        summary["outputs"][name] = {
            "path": str(output_path),
            "sha256": sha256(output_path),
            "poses": len(outputs[name]),
            "path_length_m": path_length(outputs[name]),
            "maximum_nearest_source_timestamp_distance_sec": max_nearest_time_error[name],
        }
    (args.out / "common_grid_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
