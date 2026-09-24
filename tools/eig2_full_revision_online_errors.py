#!/usr/bin/env python3
"""Score saved as-published R0/R1 states using the existing prism evaluator math.

Run with the ROS Noetic Python environment; this is evaluation-only code.
One fixed Kabsch transform is fitted per complete online trajectory, and that
same transform is used for every cumulative playback checkpoint.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/workspace/cbs_gtsam4.3/src/cbsms/tools")
from score_grandtour_prism import (  # noqa: E402
    convert_positions_to_prism,
    interpolate_reference,
    load_prism_reference,
    load_tum,
    rigid_kabsch,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--products", type=Path, required=True)
    parser.add_argument("--prism-bag", type=Path, required=True)
    args = parser.parse_args()
    gt_times, gt_points = load_prism_reference(
        args.prism_bag, "/boxi/ap20/prism_position")
    trajectories = {}
    for name in ("r0", "r1"):
        times, positions, rotations = load_tum(
            args.products / f"full_{name}/online_trajectory.tum")
        prism = convert_positions_to_prism(positions, rotations, "cpt7_imu")
        indices, reference = interpolate_reference(times, gt_times, gt_points, 0.2)
        if len(indices) < 100:
            raise ValueError(f"insufficient prism coverage for {name}")
        rotation, translation = rigid_kabsch(prism[indices], reference)
        errors = np.linalg.norm(
            prism[indices] @ rotation.T + translation - reference, axis=1)
        trajectories[name] = (times, indices, errors)
    t0, indices0, errors0 = trajectories["r0"]
    t1, indices1, errors1 = trajectories["r1"]
    if t0.shape != t1.shape or not np.allclose(t0, t1, rtol=0, atol=1e-6):
        raise ValueError("R0/R1 online physical timestamp grids differ")
    if not np.array_equal(indices0, indices1):
        raise ValueError("R0/R1 prism masks differ")
    elapsed = t0[indices0] - t0[0]
    output_csv = args.products / "online_errors.csv"
    with output_csv.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["state_index", "timestamp_sec", "playback_elapsed_sec",
                         "r0_error_m", "r1_error_m"])
        for index, time, seconds, a, b in zip(
                indices0, t0[indices0], elapsed, errors0, errors1):
            writer.writerow([int(index), format(time, ".17g"),
                             format(seconds, ".17g"), format(a, ".17g"),
                             format(b, ".17g")])
    checkpoints = [float(value) for value in range(30, 301, 30)]
    if float(elapsed[-1]) > checkpoints[-1] + 1.0:
        checkpoints.append(float(elapsed[-1]))
    result = []
    for checkpoint in checkpoints:
        mask = elapsed <= checkpoint + 1e-9
        if not np.any(mask):
            continue
        result.append({
            "playback_elapsed_sec": checkpoint,
            "matched_poses": int(np.count_nonzero(mask)),
            "r0_cumulative_ate_m": float(np.sqrt(np.mean(errors0[mask] ** 2))),
            "r1_cumulative_ate_m": float(np.sqrt(np.mean(errors1[mask] ** 2))),
        })
    summary = {
        "matched_poses": len(indices0),
        "first_to_last_matched_span_sec": float(elapsed[-1] - elapsed[0]),
        "r0_full_online_ate_m": float(np.sqrt(np.mean(errors0 ** 2))),
        "r1_full_online_ate_m": float(np.sqrt(np.mean(errors1 ** 2))),
        "checkpoints": result,
        "alignment": "separate whole-online-trajectory SE3 Kabsch, no scale",
    }
    (args.products / "online_errors_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
