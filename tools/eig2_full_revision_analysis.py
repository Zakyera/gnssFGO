#!/usr/bin/env python3
"""Validate and summarize the unchanged full EIG-2 revision receiver outputs.

This script only reads frozen source/receiver products and writes diagnostic
JSON and plots. It never constructs or updates a factor graph.
"""

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def rows(path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def stats(values):
    values = np.asarray(values, dtype=float)
    if len(values) == 0:
        raise ValueError("statistics require at least one value")
    return {
        "median": float(np.median(values)),
        "mean": float(np.mean(values)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(np.max(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
    }


def load_tum(path):
    data = np.loadtxt(path)
    if data.ndim != 2 or data.shape[1] < 8:
        raise ValueError(f"invalid TUM file {path}")
    return data


def difference(left, right):
    if left.shape != right.shape or not np.allclose(
            left[:, 0], right[:, 0], rtol=0, atol=1e-6):
        raise ValueError("receiver grids do not match")
    norms = np.linalg.norm(left[:, 1:4] - right[:, 1:4], axis=1)
    return norms


def score(product, kind, name):
    evaluation = product / "evaluation"
    prism = json.loads((evaluation / f"score_{kind}_{name}_prism.json").read_text())
    rpe = json.loads((evaluation / f"score_{kind}_{name}_rpe.json").read_text())
    return {
        "ate_m": prism["ape_rmse_m"],
        "translation_rpe_m": rpe["translation_error_m"]["rmse"],
        "rotation_rpe_deg": rpe["rotation_error_deg"]["rmse"],
        "matched_poses": prism["matched_poses"],
        "matched_duration_sec": prism["matched_elapsed_coverage_s"],
        "matched_path_m": prism["matched_estimate_path_m"],
        "full_prism_path_m": prism["estimate_full_path_m"],
        "rpe_pairs": rpe["valid_pairs"],
    }


def matched_mask(products, kind, name):
    path = products / "evaluation" / f"score_{kind}_{name}_prism_aligned.csv"
    return tuple(int(row["matched_reference"]) for row in rows(path))


def save_plot(path):
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run
    products = run / "revision_products"
    source_audit = json.loads((run / "analysis/summary.json").read_text())
    export_audit = json.loads((products / "export_summary.json").read_text())
    online = json.loads((products / "online_errors_summary.json").read_text())

    receiver = {}
    trajectories = {}
    for case in ("r0", "r1", "r2", "r3"):
        receiver[case] = score(products, "full", case)
        receiver[case]["replay"] = json.loads(
            (products / f"full_{case}/summary.json").read_text())
        trajectories[case] = load_tum(products / f"full_{case}/trajectory.tum")
    common_times = trajectories["r0"][:, 0]
    common_mask = matched_mask(products, "full", "r0")
    for case in ("r1", "r2", "r3"):
        difference(trajectories[case], trajectories["r0"])
        if matched_mask(products, "full", case) != common_mask:
            raise ValueError(f"different prism evaluation mask for {case}")
    if sum(common_mask) != receiver["r0"]["matched_poses"]:
        raise ValueError("prism mask/count mismatch")
    if len(common_mask) != len(common_times):
        raise ValueError("prism mask/trajectory length mismatch")

    source = {name: score(products, "source", name)
              for name in ("online", "matured")}
    if matched_mask(products, "source", "online") != common_mask or \
            matched_mask(products, "source", "matured") != common_mask:
        raise ValueError("source and receiver prism masks differ")
    receiver["r0_online"] = score(products, "online", "r0")
    receiver["r1_online"] = score(products, "online", "r1")
    if matched_mask(products, "online", "r0") != common_mask or \
            matched_mask(products, "online", "r1") != common_mask:
        raise ValueError("online and final prism masks differ")
    for case in ("r0", "r1"):
        if abs(online[f"{case}_full_online_ate_m"] -
               receiver[f"{case}_online"]["ate_m"]) > 1e-8:
            raise ValueError("online checkpoint and retained prism scorers disagree")

    pose_only = {}
    for product, case in (("first", "r0"), ("revision", "r1"),
                          ("last_revision", "r2"), ("finalized", "r3")):
        pose_only[case] = [score(products, "pose", f"{product}_{parity}")
                           for parity in (0, 1)]

    separation = {}
    elapsed = common_times - common_times[0]
    for a, b in (("r1", "r0"), ("r1", "r2"), ("r2", "r3")):
        values = difference(trajectories[a], trajectories[b])
        first_centimeter = np.flatnonzero(values > 0.01)
        separation[f"{a}_vs_{b}"] = {
            **stats(values),
            "first_over_1cm_elapsed_sec":
                float(elapsed[first_centimeter[0]]) if len(first_centimeter) else None,
            "first_over_1cm_index":
                int(first_centimeter[0]) if len(first_centimeter) else None,
        }
    plt.figure(figsize=(11, 4))
    plt.plot(elapsed, difference(trajectories["r1"], trajectories["r0"]),
             label="R1–R0", lw=1.1)
    plt.plot(elapsed, difference(trajectories["r1"], trajectories["r2"]),
             label="R1–R2", lw=1.1)
    plt.xlabel("Physical time after first support pose (s)")
    plt.ylabel("Raw same-gauge position difference (m)")
    plt.legend()
    save_plot(products / "trajectory_separation.png")

    checkpoints = online["checkpoints"]
    plt.figure(figsize=(9, 4))
    plt.plot([r["playback_elapsed_sec"] for r in checkpoints],
             [r["r0_cumulative_ate_m"] for r in checkpoints], "o-", label="R0 online")
    plt.plot([r["playback_elapsed_sec"] for r in checkpoints],
             [r["r1_cumulative_ate_m"] for r in checkpoints], "o-", label="R1 online")
    plt.xlabel("Playback time after first support pose (s)")
    plt.ylabel("Cumulative prism ATE RMSE (m)")
    plt.legend()
    save_plot(products / "online_cumulative_ate.png")

    first = {}
    per_id_count = Counter()
    previous = {}
    per_second = Counter()
    last = {}
    revision_fraction = []
    mean_translation_steps = []
    mean_rotation_steps = []
    covariance_rotation_change = []
    covariance_translation_change = []
    covariance_first = {}
    for row in rows(products / "revisions.csv"):
        key = (row["source_agent"], int(row["from_index"]), int(row["to_index"]))
        revision = int(row["revision"])
        if revision != per_id_count[key]:
            raise ValueError(f"non-consecutive revision for {key}")
        per_id_count[key] += 1
        timestamp_ns = int(row["record_timestamp_ns"])
        per_second[timestamp_ns // 1_000_000_000] += 1
        mu = np.array([float(row[f"relative_mu_{k}"]) for k in range(6)])
        rotation_trace = sum(float(row[f"covariance_{k}_{k}"]) for k in range(3))
        translation_trace = sum(float(row[f"covariance_{k}_{k}"]) for k in range(3, 6))
        if revision == 0:
            first[key] = row
            covariance_first[key] = (rotation_trace, translation_trace)
        else:
            old_mu, old_rot, old_trans = previous[key]
            # These are coordinate changes in the already-defined [phi,rho]
            # tangent, not a new SE(3) factor or a re-evaluated belief.
            mean_rotation_steps.append(float(np.linalg.norm(mu[:3] - old_mu[:3]))
                                       * 180.0 / math.pi)
            mean_translation_steps.append(float(np.linalg.norm(mu[3:] - old_mu[3:])))
            covariance_rotation_change.append(abs(rotation_trace - old_rot))
            covariance_translation_change.append(abs(translation_trace - old_trans))
            revision_fraction.append(revision)
        previous[key] = (mu, rotation_trace, translation_trace)
        last[key] = timestamp_ns
    if len(first) != export_audit["belief_ids"]:
        raise ValueError("BeliefID count differs from source export")
    if sum(per_id_count.values()) != export_audit["revision_messages"]:
        raise ValueError("revision message count differs from source export")
    if len(common_times) != source_audit["matured_pose_count"] or \
            len(first) != len(common_times) - 2:
        raise ValueError("receiver support grid differs from retired source population")
    last_latency = [(last[key] - int(row["to_timestamp_ns"])) * 1e-9
                    for key, row in first.items()]
    first_latency = [(int(row["record_timestamp_ns"]) -
                      int(row["to_timestamp_ns"])) * 1e-9
                     for row in first.values()]
    second_bins = list(range(min(per_second), max(per_second) + 1))
    plt.figure(figsize=(11, 4))
    plt.plot(np.array(second_bins) - second_bins[0],
             [per_second[second] for second in second_bins], lw=1)
    plt.xlabel("Record time after first revision message (s)")
    plt.ylabel("Revision messages per record second")
    save_plot(products / "revision_rate.png")

    plt.figure(figsize=(11, 4))
    edge_seconds = np.array([int(row["to_timestamp_ns"]) * 1e-9
                             for row in first.values()])
    plt.plot(edge_seconds - edge_seconds[0], first_latency, ".", ms=2,
             label="Revision 0")
    plt.plot(edge_seconds - edge_seconds[0], last_latency, ".", ms=2,
             label="Last jointly valid")
    plt.xlabel("Physical edge end after first retained end (s)")
    plt.ylabel("Availability latency (s)")
    plt.legend()
    save_plot(products / "revision_latency.png")

    # Show the size of each actual change, not only departure from finalized.
    step_index = np.asarray(revision_fraction)
    translation_steps = np.asarray(mean_translation_steps)
    rotation_steps = np.asarray(mean_rotation_steps)
    covariance_rot_steps = np.asarray(covariance_rotation_change)
    covariance_trans_steps = np.asarray(covariance_translation_change)
    max_revision = max(per_id_count.values()) - 1
    bins = np.linspace(0, max_revision + 1, 21)
    bin_centers = (bins[1:] + bins[:-1]) / 2
    grouped = np.digitize(step_index, bins[1:-1], right=False)
    def bin_medians(data):
        return [float(np.median(data[grouped == index]))
                if np.any(grouped == index) else math.nan
                for index in range(len(bin_centers))]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    series = ((translation_steps * 1000, "Translation coordinate step (mm)"),
              (rotation_steps, "Rotation coordinate step (deg)"),
              (covariance_trans_steps, "Translation trace step (m²)"),
              (covariance_rot_steps, "Rotation trace step (rad²)"))
    for axis, (data, label) in zip(axes.flat, series):
        axis.plot(bin_centers, bin_medians(data), marker="o", ms=3)
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("Revision number")
    axes[-1, 1].set_xlabel("Revision number")
    fig.tight_layout()
    fig.savefig(products / "revision_magnitude.png", dpi=160)
    plt.close(fig)

    event_path = products / "full_r1/revision_events.csv"
    outcomes = Counter()
    update_ms = []
    replacement_ms = []
    active_factors = []
    for event in rows(event_path):
        outcome = event["outcome"]
        outcomes[outcome] += 1
        active_factors.append(int(event["active_factors"]))
        if outcome in ("insert", "replace"):
            value = float(event["update_seconds"]) * 1000
            update_ms.append(value)
            if outcome == "replace":
                replacement_ms.append(value)
    r1 = receiver["r1"]["replay"]
    if outcomes["insert"] != len(first) or \
            outcomes["replace"] != export_audit["revision_messages"] - len(first):
        raise ValueError("factor replacement accounting mismatch")
    if r1["revision_insertions"] != export_audit["revision_messages"] or \
            r1["revision_removals"] != outcomes["replace"]:
        raise ValueError("R1 replay revision summary mismatch")
    expected_factors = 4 + 2 * (len(common_times) - 1) + len(first)
    if active_factors[-1] != expected_factors:
        raise ValueError("extra active factor after final revision")
    summary = {
        "source": source,
        "receiver": receiver,
        "pose_only": pose_only,
        "common_gt_mask": {
            "matched_poses": sum(common_mask),
            "grid_poses": len(common_mask),
            "matched_duration_sec": receiver["r0"]["matched_duration_sec"],
        },
        "source_population": {
            "published_beliefs": sum(1 for _ in rows(run / "beliefs.csv")),
            "retained_ids": len(first),
            "right_censored_beliefs":
                sum(1 for _ in rows(run / "beliefs.csv")) - len(first),
            "right_censored_poses": source_audit["run_pose_count"] -
                source_audit["matured_pose_count"],
            "run_pose_count": source_audit["run_pose_count"],
            "matured_pose_count": source_audit["matured_pose_count"],
        },
        "separation": separation,
        "revision_dynamics": {
            "messages": sum(per_id_count.values()),
            "revisions_per_id": stats(list(per_id_count.values())),
            "revision_zero_latency_sec": stats(first_latency),
            "last_revision_latency_sec": stats(last_latency),
            "revision_rate_per_second": stats(
                [per_second[second] for second in second_bins]),
            "revision_rate_peak": max(per_second.values()),
            "factor_outcomes": dict(outcomes),
            "peak_active_factors": max(active_factors),
            "final_active_factors": active_factors[-1],
            "expected_final_active_factors": expected_factors,
            "maximum_active_values": r1["maximum_active_values"],
            "revision_update_ms": stats(update_ms),
            "replacement_update_ms": stats(replacement_ms),
            "total_revision_update_sec": r1["revision_update_seconds"],
            "r1_replay_wall_sec": r1["wall_seconds"],
            "r1_processing_realtime_factor":
                r1["duration_seconds"] / r1["wall_seconds"],
            "source_mean_translation_coordinate_step_m": stats(translation_steps),
            "source_mean_rotation_coordinate_step_deg": stats(rotation_steps),
            "source_translation_covariance_trace_step_m2": stats(covariance_trans_steps),
            "source_rotation_covariance_trace_step_rad2": stats(covariance_rot_steps),
        },
        "online_accuracy": online,
        "source_export": export_audit,
    }
    output = products / "full_result_summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({
        "ate": {name: data["ate_m"] for name, data in receiver.items()},
        "source_ate": {name: data["ate_m"] for name, data in source.items()},
        "revision_dynamics": summary["revision_dynamics"],
        "separation": separation,
    }, indent=2))


if __name__ == "__main__":
    main()
