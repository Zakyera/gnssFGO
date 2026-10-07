#!/usr/bin/env python3
"""Read-only summaries/plots for the bounded EIG-2 CBS information audit."""

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import eigh


ANSI = re.compile(r"\x1b\[[0-9;]*m")


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"count": int(len(a)), "median": float(np.median(a)),
            "p05": float(np.percentile(a, 5)), "p95": float(np.percentile(a, 95)),
            "min": float(a.min()), "max": float(a.max())} if len(a) else None


def parse_events(path):
    events = {"K2G": [], "G2K": []}
    merges = {"K2G": [], "G2K": []}
    with path.open(encoding="utf-8", errors="replace") as stream:
        for number, raw in enumerate(stream, 1):
            line = ANSI.sub("", raw)
            if "CBS_ODOM_PREINJECTION_RESIDUAL_ROW," in line:
                f = line.split("CBS_ODOM_PREINJECTION_RESIDUAL_ROW,", 1)[1].strip().split(",")
                if len(f) >= 15 and f[0] == "K2G":
                    events["K2G"].append({"line": number, "key": f[3] + "->" + f[4],
                        "rotation": float(f[8]), "translation": float(f[9]),
                        "nis": float(f[14]), "trace": float(f[12])})
            if "CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW," in line:
                f = line.split("CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW,", 1)[1].strip().split(",")
                if len(f) >= 30 and f[0] == "G2K":
                    v = np.fromstring(f[25], sep=";")
                    events["G2K"].append({"line": number, "key": f[5] + "->" + f[6],
                        "rotation": float(np.linalg.norm(v[:3])),
                        "translation": float(np.linalg.norm(v[3:])),
                        "nis": float(f[28]), "trace": float(f[17])})
            for direction, marker in (("K2G", "GLIM_CBS_POSE_MERGE_ROW,"),
                                      ("G2K", "KIMERA_CBS_POSE_MERGE_ROW,")):
                if marker in line:
                    f = line.split(marker, 1)[1].strip().split(",")
                    if len(f) >= 6:
                        merges[direction].append({"line": number, "key": f[1],
                            "before_rotation": float(f[2]), "before_translation": float(f[3]),
                            "after_rotation": float(f[4]), "after_translation": float(f[5])})
    return events, merges


def attach_physical_time(items, stamps):
    for item in items:
        match = re.search(r"x(\d+)$", item["key"])
        if match and int(match.group(1)) in stamps:
            item["elapsed_s"] = stamps[int(match.group(1))]


def odometry_stamps(path):
    measurements = rows(path)
    origin = int(measurements[0]["field.header.stamp"])
    return {int(row["field.header.seq"]):
            (int(row["field.header.stamp"]) - origin) * 1e-9
            for row in measurements}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--on", type=Path, required=True)
    args = p.parse_args()
    out = args.on / "competition_finalize"
    out.mkdir(parents=True, exist_ok=True)
    samples = rows(args.on / "receiver_competition/competition_samples.csv")
    matrices = np.load(args.on / "receiver_competition/competition_matrices.npz",
                       allow_pickle=True)
    events, merges = parse_events(args.on / "estimators.log")
    stamps = {"K2G": odometry_stamps(args.on / "glim_odometry.csv"),
              "G2K": odometry_stamps(args.on / "kimera_odometry.csv")}
    for direction in stamps:
        attach_physical_time(events[direction], stamps[direction])
        attach_physical_time(merges[direction], stamps[direction])
    report = {"directions": {}, "selected": []}
    for direction in ("G2K", "K2G"):
        ev, mg = events[direction], merges[direction]
        report["directions"][direction] = {
            "preupdate_rotation_rad": stats([x["rotation"] for x in ev]),
            "preupdate_translation_m": stats([x["translation"] for x in ev]),
            "preupdate_mahalanobis2": stats([x["nis"] for x in ev]),
            "source_covariance_trace_mixed": stats([x["trace"] for x in ev]),
            "merge_before_rotation_rad": stats([x["before_rotation"] for x in mg]),
            "merge_after_rotation_rad": stats([x["after_rotation"] for x in mg]),
            "merge_before_translation_m": stats([x["before_translation"] for x in mg]),
            "merge_after_translation_m": stats([x["after_translation"] for x in mg]),
            "counts": {"preupdate": len(ev), "merge": len(mg)},
        }
        group = [r for r in samples if r["direction"] == direction and r["status"] == "ok"]
        if group:
            chosen = {0, len(group) - 1, len(group) // 2,
                      int(np.argmax([float(r["trace_ratio"]) for r in group])),
                      int(np.argmin([float(r["trace_ratio"]) for r in group]))}
            for index in sorted(chosen):
                row = group[index]
                matrix_index = next(j for j, meta in enumerate(matrices["metadata"])
                                    if meta[0] == direction and
                                    int(meta[1]) == int(row["sample_index"]))
                covariance = matrices["inserted_covariance"][matrix_index]
                incoming = matrices["inserted_information"][matrix_index]
                local = matrices["local_information"][matrix_index]
                key = row["receiver_from"] + "->" + row["receiver_to"]
                lineno = int(row["log_line"])
                prior = [e for e in ev if e["key"] == key and e["line"] <= lineno]
                following = [m for m in mg if m["key"] == key and m["line"] > lineno]
                match = prior[-1] if prior else None
                merge = following[0] if following else None
                report["selected"].append({
                    "direction": direction, "receiver_edge": key,
                    "sample_index": int(row["sample_index"]),
                    "local_factor_count": int(row["local_factor_count"]),
                    "trace_information_ratio": float(row["trace_ratio"]),
                    "generalized_ratio_eigenvalues": [float(row[f"generalized_eigenvalue_{i}"]) for i in range(6)],
                    "sender_rotation_covariance_eigenvalues_rad2": np.linalg.eigvalsh(
                        covariance[:3, :3]).tolist(),
                    "sender_translation_covariance_eigenvalues_m2": np.linalg.eigvalsh(
                        covariance[3:, 3:]).tolist(),
                    "sender_cross_covariance_frobenius_rad_m": float(np.linalg.norm(
                        covariance[:3, 3:])),
                    "sender_full_covariance_eigenvalues_mixed": np.linalg.eigvalsh(
                        covariance).tolist(),
                    "sender_information_eigenvalues_mixed": np.linalg.eigvalsh(
                        incoming).tolist(),
                    "native_local_information_eigenvalues_mixed": np.linalg.eigvalsh(
                        local).tolist(),
                    "rotation_block_generalized_ratios": eigh(
                        incoming[:3, :3], local[:3, :3], eigvals_only=True).tolist(),
                    "translation_block_generalized_ratios": eigh(
                        incoming[3:, 3:], local[3:, 3:], eigvals_only=True).tolist(),
                    "external_mahalanobis2": match["nis"] if match else None,
                    "preupdate_translation_m": match["translation"] if match else None,
                    "preupdate_rotation_rad": match["rotation"] if match else None,
                    "postsolve_gap_translation_m": merge["after_translation"] if merge else None,
                    "postsolve_gap_rotation_rad": merge["after_rotation"] if merge else None,
                    "merge_log_distance": merge["line"] - lineno if merge else None,
                })
        covariance_samples = []
        for j, meta in enumerate(matrices["metadata"]):
            if meta[0] != direction:
                continue
            receiver_to = str(meta[6])
            match = re.search(r"x(\d+)$", receiver_to)
            if match and int(match.group(1)) in stamps[direction]:
                covariance = matrices["inserted_covariance"][j]
                covariance_samples.append({
                    "elapsed_s": stamps[direction][int(match.group(1))],
                    "rot_cov": float(np.trace(covariance[:3, :3])),
                    "trans_cov": float(np.trace(covariance[3:, 3:])),
                })
        fig, axes = plt.subplots(6, 1, figsize=(10, 11), sharex=True)
        for ax, label, source, value_key, unit in (
            (axes[0], "source rotational trace", covariance_samples, "rot_cov", "rad²"),
            (axes[1], "source translational trace", covariance_samples, "trans_cov", "m²"),
            (axes[2], "translation residual", ev, "translation", "m"),
            (axes[3], "rotation residual", ev, "rotation", "rad"),
            (axes[4], "Mahalanobis squared", ev, "nis", ""),
            (axes[5], "postsolve external gap", mg, "after_translation", "m")):
            valid = [item for item in source if "elapsed_s" in item]
            xs = [item["elapsed_s"] for item in valid]
            ys = [item[value_key] for item in valid]
            if source is mg:
                ax.plot(xs, ys, lw=.8)
            else:
                ax.scatter(xs, ys, s=5 if len(valid) > 100 else 18, alpha=.45)
            ax.set_ylabel(label + (f" ({unit})" if unit else ""))
            ax.grid(alpha=.25)
        axes[0].set_yscale("log")
        axes[1].set_yscale("log")
        axes[4].set_yscale("log")
        axes[5].set_xlabel("seconds from receiver trajectory start")
        fig.suptitle(f"{direction}: recorded CBS belief/receiver diagnostics")
        fig.tight_layout()
        fig.savefig(out / f"{direction.lower()}_competition.png", dpi=150)
        plt.close(fig)
    with (out / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({d: report["directions"][d]["counts"] for d in report["directions"]}))


if __name__ == "__main__":
    main()
