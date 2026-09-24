#!/usr/bin/env python3
"""Check common EIG-2 Kimera score masks and summarize source/receiver cases."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


CASES = ("native_online", "native_matured",
         "k0_online", "k0_final", "k1_online", "k1_final", "k2_final",
         "k3_final", "pose_k0", "pose_k1", "pose_k2", "pose_k3")


def rows(path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"median": float(np.median(a)), "rms": float(np.sqrt(np.mean(a * a))),
            "p95": float(np.percentile(a, 95)), "maximum": float(np.max(a))}


def score(directory, name):
    prism = json.loads((directory / f"{name}_prism.json").read_text())
    rpe = json.loads((directory / f"{name}_rpe.json").read_text())
    return {"ate_m": prism["ape_rmse_m"],
            "translation_rpe_m": rpe["translation_error_m"]["rmse"],
            "rotation_rpe_deg": rpe["rotation_error_deg"]["rmse"],
            "matched_poses": prism["matched_poses"],
            "rpe_pairs": rpe["valid_pairs"],
            "full_path_m": prism["estimate_full_path_m"],
            "matched_path_m": prism["matched_estimate_path_m"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    products = args.run / "revision_products"
    evaluation = products / "evaluation"
    exported = json.loads((products / "export_summary.json").read_text())
    result = {"source": exported, "score": {}, "receiver": {}}
    baseline_mask = None
    cases = list(CASES)
    if (products / "common_eval_tum/native_smoother_first.tum").exists():
        cases.append("native_smoother_first")
    for name in cases:
        result["score"][name] = score(evaluation, name)
        mask = tuple(int(row["matched_reference"]) for row in
                     rows(evaluation / f"{name}_prism_aligned.csv"))
        if baseline_mask is None:
            baseline_mask = mask
        elif mask != baseline_mask:
            raise ValueError(f"different AP20 prism mask: {name}")
    result["common_gt_mask_rows"] = sum(baseline_mask)
    result["common_eval_rows"] = len(baseline_mask)
    for case in ("k0", "k1", "k2", "k3"):
        info = json.loads((products / f"full_{case}" / "summary.json").read_text())
        if info["glim_beliefs_accepted"] != 0:
            raise ValueError(f"GLIM measurement leaked into {case}")
        if info["kimera_beliefs_accepted"] < 100:
            raise ValueError(f"unexpectedly few Kimera beliefs in {case}")
        result["receiver"][case] = info
    selected = result["receiver"]["k0"]["kimera_beliefs_accepted"]
    if any(result["receiver"][c]["kimera_beliefs_accepted"] != selected
           for c in ("k1", "k2", "k3")):
        raise ValueError("full receiver cases accepted different ID counts")
    for a, b in (("k1_final", "k0_final"), ("k1_final", "k2_final"),
                 ("k2_final", "k3_final")):
        one = np.loadtxt(products / "common_eval_tum" / f"{a}.tum")
        two = np.loadtxt(products / "common_eval_tum" / f"{b}.tum")
        if one.shape != two.shape or np.max(np.abs(one[:, 0] - two[:, 0])) > 1e-6:
            raise ValueError("receiver comparison grids differ")
        result.setdefault("same_gauge_separation", {})[f"{a}_vs_{b}"] = stats(
            np.linalg.norm(one[:, 1:4] - two[:, 1:4], axis=1))
    online = list(rows(products / "online_ate_vs_playback.csv"))
    result["online_ate_checkpoints"] = online
    if abs(float(online[-1]["k1_online_ate_m"]) -
           result["score"]["k1_online"]["ate_m"]) > 1e-8:
        raise ValueError("online ATE checkpoint/scorer mismatch")
    plt.figure(figsize=(9, 4))
    plt.plot([float(r["playback_elapsed_sec"]) for r in online],
             [float(r["k0_online_ate_m"]) for r in online], "o-", label="K0 first")
    plt.plot([float(r["playback_elapsed_sec"]) for r in online],
             [float(r["k1_online_ate_m"]) for r in online], "o-", label="K1 revision")
    plt.xlabel("Playback elapsed after common Kimera start (s)")
    plt.ylabel("Cumulative online AP20 prism ATE RMSE (m)")
    plt.legend()
    plt.grid(alpha=0.25)
    plt.tight_layout()
    plt.savefig(products / "online_cumulative_ate.png", dpi=160)
    plt.close()
    (products / "full_result_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"score": result["score"],
                      "same_gauge_separation": result.get("same_gauge_separation"),
                      "common_gt_mask_rows": result["common_gt_mask_rows"]}, indent=2))


if __name__ == "__main__":
    main()
