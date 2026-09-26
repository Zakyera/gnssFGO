#!/usr/bin/env python3
"""Summarize the SPX-3 GLIM-only source revision preservation controls."""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from eig2_single_source_preservation import exp_se3, inverse, log_se3


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = np.asarray(values, dtype=float)
    return {"median": float(np.median(values)),
            "mean": float(np.mean(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
            "rms": float(np.sqrt(np.mean(values ** 2)))}


def metric(products, name):
    prism = json.loads((products / "evaluation" /
                        f"score_{name}_prism.json").read_text())
    rpe = json.loads((products / "evaluation" /
                      f"score_{name}_rpe.json").read_text())
    return {"ate_m": prism["ape_rmse_m"],
            "matched_poses": prism["matched_poses"],
            "matched_coverage_s": prism["matched_elapsed_coverage_s"],
            "t_rpe_m": rpe["translation_error_m"]["rmse"],
            "r_rpe_deg": rpe["rotation_error_deg"]["rmse"],
            "rpe_pairs": rpe["valid_pairs"]}


def trajectory_difference(a, b):
    one, two = np.loadtxt(a), np.loadtxt(b)
    if one.shape != two.shape or np.max(np.abs(one[:, 0] - two[:, 0])) > 1e-6:
        return {"same_grid": False, "poses_a": int(len(one)),
                "poses_b": int(len(two))}
    translation = np.linalg.norm(one[:, 1:4] - two[:, 1:4], axis=1)
    q1 = one[:, 4:8] / np.linalg.norm(one[:, 4:8], axis=1)[:, None]
    q2 = two[:, 4:8] / np.linalg.norm(two[:, 4:8], axis=1)[:, None]
    rotation = np.rad2deg(2 * np.arccos(np.clip(
        np.abs(np.sum(q1 * q2, axis=1)), 0, 1)))
    return {"same_grid": True, "translation_m": stats(translation),
            "rotation_deg": stats(rotation)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--products", type=Path, required=True)
    parser.add_argument("--frozen-beliefs", type=Path, required=True)
    parser.add_argument("--frozen-trajectory", type=Path, required=True)
    args = parser.parse_args()
    products = args.products
    first = rows(products / "first.csv")
    last = rows(products / "last_revision.csv")
    revision = rows(products / "revisions.csv")
    frozen = rows(args.frozen_beliefs)
    if len(first) != len(last) or len(first) != len(frozen):
        raise ValueError("first/last/frozen ID populations differ")
    identity = lambda row: (row["source_agent"], int(row["from_index"]),
                            int(row["to_index"]))
    if [identity(row) for row in first] != [identity(row) for row in last] or \
            [identity(row) for row in first] != [identity(row) for row in frozen]:
        raise ValueError("stable ID ordering differs")
    by_id = defaultdict(list)
    for row in revision:
        by_id[identity(row)].append(row)
    if len(by_id) != len(first):
        raise ValueError("revision stream missing IDs")
    version_counts, first_latencies, last_latencies = [], [], []
    ratio_rotation, ratio_translation = [], []
    first_rotation_trace, first_translation_trace = [], []
    last_rotation_trace, last_translation_trace = [], []
    mean_change_translation, mean_change_rotation = [], []
    for old, new in zip(first, last):
        key = identity(old)
        history = by_id[key]
        if [int(item["revision"]) for item in history] != list(range(len(history))):
            raise ValueError(f"nonconsecutive revisions: {key}")
        version_counts.append(len(history))
        endpoint = int(old["to_timestamp_ns"])
        first_latencies.append((int(history[0]["record_timestamp_ns"]) - endpoint) / 1e9)
        last_latencies.append((int(history[-1]["record_timestamp_ns"]) - endpoint) / 1e9)
        old_rot = sum(float(old[f"covariance_{k}_{k}"]) for k in range(3))
        new_rot = sum(float(new[f"covariance_{k}_{k}"]) for k in range(3))
        old_trans = sum(float(old[f"covariance_{k}_{k}"]) for k in range(3, 6))
        new_trans = sum(float(new[f"covariance_{k}_{k}"]) for k in range(3, 6))
        first_rotation_trace.append(old_rot)
        first_translation_trace.append(old_trans)
        last_rotation_trace.append(new_rot)
        last_translation_trace.append(new_trans)
        ratio_rotation.append(new_rot / old_rot)
        ratio_translation.append(new_trans / old_trans)
        old_mu = np.array([float(old[f"relative_mu_{k}"]) for k in range(6)])
        new_mu = np.array([float(new[f"relative_mu_{k}"]) for k in range(6)])
        delta = log_se3(inverse(exp_se3(old_mu)) @ exp_se3(new_mu))
        mean_change_translation.append(float(np.linalg.norm(delta[3:])))
        mean_change_rotation.append(float(np.rad2deg(np.linalg.norm(delta[:3]))))
    mean_keys = [f"relative_mu_{k}" for k in range(6)]
    cov_keys = [f"covariance_{r}_{c}" for r in range(6) for c in range(6)]
    frozen_mean = np.array([[float(row[key]) for key in mean_keys] for row in frozen])
    first_mean = np.array([[float(row[key]) for key in mean_keys] for row in first])
    frozen_cov = np.array([[float(row[key]) for key in cov_keys] for row in frozen])
    first_cov = np.array([[float(row[key]) for key in cov_keys] for row in first])
    endpoint_ns_diff = [abs(int(a["to_timestamp_ns"]) - int(b["to_timestamp_ns"]))
                        for a, b in zip(first, frozen)]
    receiver = {}
    for case in ("r0", "r1", "r2"):
        receiver[case] = metric(products, f"full_{case}")
        receiver[case]["graph"] = json.loads(
            (products / f"full_{case}" / "summary.json").read_text())
    pose = {case: [metric(products, f"pose_{case}_{parity}")
                   for parity in (0, 1)] for case in ("r0", "r1", "r2")}
    online = {case: metric(products, f"online_{case}") for case in ("r0", "r1")}
    event_rows = rows(products / "full_r1" / "revision_events.csv")
    result = {
        "native_same_grid": metric(products, "native"),
        "receiver": receiver, "online_receiver": online,
        "pose_only_by_parity": pose,
        "source": {
            "belief_ids": len(first), "revision_messages": len(revision),
            "versions_per_id": stats(version_counts),
            "first_publication_latency_s": stats(first_latencies),
            "last_valid_latency_s": stats(last_latencies),
            "last_over_first_rotation_cov_trace_ratio": stats(ratio_rotation),
            "last_over_first_translation_cov_trace_ratio": stats(ratio_translation),
            "first_rotation_cov_trace_rad2": stats(first_rotation_trace),
            "last_rotation_cov_trace_rad2": stats(last_rotation_trace),
            "first_translation_cov_trace_m2": stats(first_translation_trace),
            "last_translation_cov_trace_m2": stats(last_translation_trace),
            "first_to_last_relative_mean_translation_change_m": stats(mean_change_translation),
            "first_to_last_relative_mean_rotation_change_deg": stats(mean_change_rotation),
            "max_fresh_vs_frozen_endpoint_timestamp_difference_ns": max(endpoint_ns_diff),
            "fresh_vs_frozen_mean_coordinate_difference": {
                "max_abs": float(np.max(np.abs(first_mean - frozen_mean))),
                "rms": float(np.sqrt(np.mean((first_mean - frozen_mean) ** 2)))},
            "fresh_vs_frozen_covariance_entry_difference": {
                "max_abs": float(np.max(np.abs(first_cov - frozen_cov))),
                "rms": float(np.sqrt(np.mean((first_cov - frozen_cov) ** 2)))},
        },
        "r1_factor_events": {
            name: sum(row["outcome"] == name for row in event_rows)
            for name in ("insert", "replace", "duplicate", "stale")},
        "trajectory_differences": {
            "r1_minus_r2": trajectory_difference(
                products / "full_r1" / "trajectory.tum",
                products / "full_r2" / "trajectory.tum"),
            "fresh_r0_minus_frozen_r0": trajectory_difference(
                products / "full_r0" / "trajectory.tum",
                args.frozen_trajectory),
        },
    }
    (products / "result_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
