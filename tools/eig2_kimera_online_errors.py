#!/usr/bin/env python3
"""Cumulative ATE for the causally saved Kimera-only receiver trajectories.

Uses the unchanged AP20 prism conversion, GT matching and no-scale Kabsch
functions. Each whole online trajectory gets one fixed transform; no prefix
is allowed to refit its own alignment.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/workspace/cbs_gtsam4.3/src/cbsms/tools")
from score_grandtour_prism import (  # noqa: E402
    convert_positions_to_prism, interpolate_reference, load_prism_reference,
    load_tum, rigid_kabsch,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--products", required=True, type=Path)
    parser.add_argument("--prism-bag", required=True, type=Path)
    args = parser.parse_args()
    gt_times, gt_points = load_prism_reference(
        args.prism_bag, "/boxi/ap20/prism_position")
    trajectories = {}
    for name in ("k0_online", "k1_online"):
        times, positions, rotations = load_tum(
            args.products / "common_eval_tum" / f"{name}.tum")
        prism = convert_positions_to_prism(positions, rotations, "cpt7_imu")
        indices, reference = interpolate_reference(times, gt_times, gt_points, 0.2)
        if len(indices) < 100:
            raise ValueError(f"insufficient prism matches for {name}")
        rotation, translation = rigid_kabsch(prism[indices], reference)
        error = np.linalg.norm(prism[indices] @ rotation.T + translation - reference,
                               axis=1)
        trajectories[name] = times, indices, error
    t0, i0, e0 = trajectories["k0_online"]
    t1, i1, e1 = trajectories["k1_online"]
    if not np.allclose(t0, t1, atol=1e-6, rtol=0) or not np.array_equal(i0, i1):
        raise ValueError("online trajectory timestamps/masks differ")
    elapsed = t0[i0] - t0[0]
    checkpoints = list(range(30, 301, 30))
    checkpoints.append(float(elapsed[-1]))
    rows = []
    for checkpoint in checkpoints:
        mask = elapsed <= checkpoint + 1e-9
        if not np.any(mask):
            continue
        rows.append({"playback_elapsed_sec": float(checkpoint),
                     "matched_poses": int(np.count_nonzero(mask)),
                     "k0_online_ate_m": float(np.sqrt(np.mean(e0[mask] ** 2))),
                     "k1_online_ate_m": float(np.sqrt(np.mean(e1[mask] ** 2)))})
    with (args.products / "online_ate_vs_playback.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    result = {"matched_poses": len(i0), "checkpoints": rows,
              "alignment": "one whole-online-trajectory Kabsch transform per case"}
    (args.products / "online_ate_summary.json").write_text(
        json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
