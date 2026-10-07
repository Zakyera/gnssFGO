#!/usr/bin/env python3
"""Collect unchanged-evaluator scores and receiver accounting for SPX-3 A–D."""

import argparse
import json
from pathlib import Path


def read_json(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--products", required=True, type=Path)
    args = parser.parse_args()
    products = args.products
    table = {}
    for name in [*(f"{case}_{kind}" for case in "ABCD"
                  for kind in ("trajectory", "online_trajectory")),
                 *(f"native_{mode}_{source}" for mode in ("off", "on")
                   for source in ("glim", "kimera"))]:
        prism = read_json(products / "evaluation" / f"{name}_prism.json")
        rpe = read_json(products / "evaluation" / f"{name}_rpe.json")
        table[name] = {
            "ate_rmse_m": prism["ape_rmse_m"],
            "translation_rpe_1s_rmse_m": rpe["translation_error_m"]["rmse"],
            "rotation_rpe_1s_rmse_deg": rpe["rotation_error_deg"]["rmse"],
            "full_path_m": prism["estimate_full_path_m"],
            "gt_overlap_path_m": prism["matched_estimate_path_m"],
            "gt_matched_poses": prism["matched_poses"],
            "rpe_valid_pairs": rpe["valid_pairs"],
            "gt_coverage_s": prism["matched_elapsed_coverage_s"],
        }
    grid = {(table[f"{case}_{kind}"]["gt_matched_poses"],
             table[f"{case}_{kind}"]["rpe_valid_pairs"])
            for case in "ABCD" for kind in ("trajectory", "online_trajectory")}
    if len(grid) != 1:
        raise ValueError(f"A–D graph evaluation masks differ: {grid}")
    receiver = {}
    for case in "ABCD":
        summary = read_json(products / case / "summary.json")
        receiver[case] = {key: summary[key] for key in (
            "support_states", "startup_imu_samples", "glim_beliefs_accepted",
            "kimera_beliefs_accepted", "beliefs_skipped_no_support",
            "revision_insertions", "revision_removals", "revision_duplicates",
            "revision_stale", "maximum_factor_covariance_abs_difference",
            "wall_seconds")}
    outcome = {"scores": table, "receiver": receiver,
               "common_graph_evaluation_mask": sorted(grid)}
    (products / "matched_result_summary.json").write_text(
        json.dumps(outcome, indent=2) + "\n")
    print(json.dumps(outcome, indent=2))


if __name__ == "__main__":
    main()
