#!/usr/bin/env python3
"""Summarize the bounded GLIM-only revision experiment without refitting factors."""

import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def stats(values):
    values = np.asarray(values, dtype=float)
    return {"mean": float(np.mean(values)), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "maximum": float(np.max(values)),
            "rms": float(np.sqrt(np.mean(values ** 2)))}


def read_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def trajectory_difference(a, b):
    one = np.loadtxt(a)
    two = np.loadtxt(b)
    if one.shape != two.shape or np.max(np.abs(one[:, 0] - two[:, 0])) > 1e-6:
        raise ValueError("trajectory time grids differ")
    translation = np.linalg.norm(one[:, 1:4] - two[:, 1:4], axis=1)
    q1 = one[:, 4:8] / np.linalg.norm(one[:, 4:8], axis=1)[:, None]
    q2 = two[:, 4:8] / np.linalg.norm(two[:, 4:8], axis=1)[:, None]
    angle = 2 * np.arccos(np.clip(np.abs(np.sum(q1 * q2, axis=1)), 0, 1))
    return {"translation_m": stats(translation),
            "rotation_deg": stats(np.rad2deg(angle))}


def online_ate_progress(products, run):
    sys.path.insert(0, "/workspace/cbs_gtsam4.3/src/cbsms/tools")
    from score_grandtour_prism import (  # pylint: disable=import-outside-toplevel
        convert_positions_to_prism, interpolate_reference,
        load_prism_reference, load_tum, rigid_kabsch,
    )
    bag = Path("/workspace/cbs_gtsam4.3/src/datasets/GrandTour/EIG-2/"
               "2024-11-03-13-59-54_ap20_prism_position.bag")
    gt_times, gt_points = load_prism_reference(bag, "/boxi/ap20/prism_position")
    curves = {}
    for case in ("r0", "r1"):
        timestamps, positions, rotations = load_tum(
            products / f"full_{case}/online_trajectory.tum")
        source_prism = convert_positions_to_prism(positions, rotations, "cpt7_imu")
        indices, references = interpolate_reference(timestamps, gt_times, gt_points, 0.2)
        if len(indices) != len(timestamps):
            raise ValueError("online trajectory lacks prism coverage")
        alignment_r, alignment_t = rigid_kabsch(source_prism, references)
        aligned = source_prism @ alignment_r.T + alignment_t
        errors = np.linalg.norm(aligned - references, axis=1)
        curves[case] = (timestamps - timestamps[0], errors)
    if np.max(np.abs(curves["r0"][0] - curves["r1"][0])) > 1e-6:
        raise ValueError("online receiver time grids differ")
    rows = []
    elapsed = curves["r0"][0]
    for end in list(range(5, 60, 5)) + [float(elapsed[-1])]:
        valid = elapsed <= end + 1e-8
        rows.append({"playback_elapsed_sec": end,
                     "poses": int(np.count_nonzero(valid)),
                     "r0_cumulative_ate_m": float(np.sqrt(
                         np.mean(curves["r0"][1][valid] ** 2))),
                     "r1_cumulative_ate_m": float(np.sqrt(
                         np.mean(curves["r1"][1][valid] ** 2)))})
    with (products / "online_ate_vs_playback.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return rows


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: result_summary RUN_DIRECTORY")
    run = Path(sys.argv[1])
    products = run / "revision_products"
    evaluation = products / "evaluation"
    receiver = {}
    for case in ("r0", "r1", "r2", "r3"):
        prism = json.loads((evaluation / f"score_full_{case}_prism.json").read_text())
        rpe = json.loads((evaluation / f"score_full_{case}_rpe.json").read_text())
        fgo = json.loads((products / f"full_{case}/summary.json").read_text())
        receiver[case] = {"ate_m": prism["ape_rmse_m"],
                          "translation_rpe_m": rpe["translation_error_m"]["rmse"],
                          "rotation_rpe_deg": rpe["rotation_error_deg"]["rmse"],
                          "path_m": fgo["path_length_m"],
                          "wall_sec": fgo["wall_seconds"],
                          "accepted": fgo["beliefs_accepted"]}
    pose_only = {}
    for case in ("first", "revision", "last_revision", "finalized"):
        pose_only[case] = []
        for parity in (0, 1):
            prism = json.loads((evaluation /
                f"score_pose_{case}_{parity}_prism.json").read_text())
            rpe = json.loads((evaluation /
                f"score_pose_{case}_{parity}_rpe.json").read_text())
            pose_only[case].append({
                "parity": parity, "ate_m": prism["ape_rmse_m"],
                "translation_rpe_m": rpe["translation_error_m"]["rmse"],
                "rotation_rpe_deg": rpe["rotation_error_deg"]["rmse"]})
    events_path = products / "full_r1b/revision_events.csv"
    if not events_path.exists():
        events_path = products / "full_r1/revision_events.csv"
    events = read_rows(events_path)
    updates = [row for row in events if row["outcome"] in ("insert", "replace")]
    first = {(int(row["from_index"]), int(row["to_index"])): row
             for row in read_rows(products / "first.csv")}
    last_record = {}
    per_second = Counter()
    for row in updates:
        key = (int(row["from_index"]), int(row["to_index"]))
        record = int(row["record_timestamp_ns"])
        last_record[key] = record
        per_second[record // 1_000_000_000] += 1
    latencies = [(last_record[key] - int(row["to_timestamp_ns"])) / 1e9
                 for key, row in first.items()]
    source_versions = defaultdict(list)
    for row in read_rows(products / "revisions.csv"):
        source_versions[(int(row["from_index"]), int(row["to_index"]))].append(row)
    covariance_evolution = []
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        rotation = []
        translation = []
        for versions in source_versions.values():
            row = versions[round((len(versions) - 1) * fraction)]
            rotation.append(sum(float(row[f"covariance_{k}_{k}"])
                                for k in range(3)))
            translation.append(sum(float(row[f"covariance_{k}_{k}"])
                                   for k in range(3, 6)))
        covariance_evolution.append({"revision_fraction": fraction,
                                     "median_rotation_trace_rad2": float(np.median(rotation)),
                                     "median_translation_trace_m2": float(np.median(translation))})
    source = {}
    for case in ("online", "matured"):
        prism = json.loads((evaluation / f"score_source_{case}_prism.json").read_text())
        rpe = json.loads((evaluation / f"score_source_{case}_rpe.json").read_text())
        source[case] = {"ate_m": prism["ape_rmse_m"],
                        "translation_rpe_m": rpe["translation_error_m"]["rmse"],
                        "rotation_rpe_deg": rpe["rotation_error_deg"]["rmse"]}
    differences = {}
    for a, b in (("r1", "r0"), ("r1", "r2"), ("r1", "r3"), ("r2", "r3")):
        differences[f"{a}_minus_{b}"] = trajectory_difference(
            products / f"full_{a}/trajectory.tum",
            products / f"full_{b}/trajectory.tum")
    for a, b in (("revision", "last_revision"),
                 ("revision", "finalized"),
                 ("last_revision", "finalized")):
        differences[f"pose_{a}_minus_{b}"] = trajectory_difference(
            products / f"pose_only_{a}.tum",
            products / f"pose_only_{b}.tum")
    summary = {
        "source": source, "receiver": receiver, "pose_only": pose_only,
        "factor_events": {
            "unique_ids": len(first), "total_messages": len(events),
            "insertions": sum(row["outcome"] == "insert" for row in events),
            "replacements": sum(row["outcome"] == "replace" for row in events),
            "duplicates": sum(row["outcome"] == "duplicate" for row in events),
            "stale": sum(row["outcome"] == "stale" for row in events),
            "last_revision_latency_sec": stats(latencies),
            "updates_per_record_second": stats(list(per_second.values())),
            "per_revision_update_ms": stats([
                float(row["update_seconds"]) * 1000 for row in updates]),
            "current_position_correction_m": stats([
                float(row["current_position_correction_m"]) for row in updates]),
            "endpoint_position_correction_m": stats([
                float(row.get("endpoint_position_correction_m", 0)) for row in updates]),
            "peak_active_revised_beliefs": len(first)},
        "trajectory_differences": differences,
        "covariance_evolution": covariance_evolution,
        "online_ate_vs_playback": online_ate_progress(products, run),
    }
    (products / "result_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
