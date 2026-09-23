#!/usr/bin/env python3
"""Isolated EIG-2 edge/covariance and graph-residual audit; no estimator changes."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from eig2_single_source_preservation import inverse, log_se3, pose, read_tum, stats, write_tum


def load_beliefs(path):
    from eig2_single_source_preservation import exp_se3

    edges = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            covariance = np.array([[float(row[f"covariance_{i}_{j}"]) for j in range(6)] for i in range(6)])
            tangent = np.array([float(row[f"relative_mu_{i}"]) for i in range(6)])
            edges.append({"ordinal": int(row["ordinal"]), "i": int(row["from_index"]),
                          "j": int(row["to_index"]), "ti": int(row["from_timestamp_ns"]),
                          "tj": int(row["to_timestamp_ns"]), "record": int(row["record_timestamp_ns"]),
                          "z": exp_se3(tangent), "cov": covariance})
    return edges


def residuals(poses, edges):
    rows = []
    for edge in edges:
        delta = log_se3(inverse(edge["z"]) @ inverse(poses[edge["i"]]) @ poses[edge["j"]])
        whitened = np.linalg.solve(np.linalg.cholesky(edge["cov"]), delta)
        rows.append((np.linalg.norm(delta[:3]) * 180 / np.pi,
                     np.linalg.norm(delta[3:]), np.linalg.norm(whitened),
                     float(whitened @ whitened)))
    return np.asarray(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--beliefs", type=Path, required=True)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--pose-only-native", type=Path, required=True)
    parser.add_argument("--pose-only-exported", type=Path, required=True)
    parser.add_argument("--compare-beliefs", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    timestamps, native = read_tum(args.native)
    edges = load_beliefs(args.beliefs)
    results = {}
    trajectories = {"native": native}
    for label, filename in (("graph", args.graph), ("pose_only_native", args.pose_only_native),
                            ("pose_only_exported", args.pose_only_exported)):
        other_times, trajectory = read_tum(filename)
        if len(other_times) != len(timestamps) or max(
            abs(a - b) for a, b in zip(other_times, timestamps)
        ) > 500:
            raise ValueError(f"{label} timestamps differ from native by more than TUM rounding")
        trajectories[label] = trajectory
    for label, trajectory in trajectories.items():
        values = residuals(trajectory, edges)
        results[label] = {
            "rotation_deg": stats(values[:, 0]), "translation_m": stats(values[:, 1]),
            "whitened_norm": stats(values[:, 2]), "nis": stats(values[:, 3]),
        }
    covariances = np.array([edge["cov"] for edge in edges])
    eigenvalues = np.linalg.eigvalsh(covariances)
    results["covariance"] = {
        "count": len(edges),
        "max_symmetry_error": float(np.max(np.abs(covariances - np.swapaxes(covariances, 1, 2)))),
        "minimum_eigenvalue": float(np.min(eigenvalues)),
        "maximum_eigenvalue": float(np.max(eigenvalues)),
        "rotation_trace": stats(np.trace(covariances[:, :3, :3], axis1=1, axis2=2)),
        "translation_trace": stats(np.trace(covariances[:, 3:, 3:], axis1=1, axis2=2)),
        "max_abs_offdiagonal": float(np.max(np.abs(covariances - covariances * np.eye(6)))),
    }
    results["belief_arrival_delay_s"] = stats(
        [(edge["record"] - edge["tj"]) * 1e-9 for edge in edges])
    if args.compare_beliefs is not None:
        other_edges = load_beliefs(args.compare_beliefs)
        if len(other_edges) != len(edges) or any(
            (a["ordinal"], a["i"], a["j"], a["ti"], a["tj"]) !=
            (b["ordinal"], b["i"], b["j"], b["ti"], b["tj"])
            for a, b in zip(edges, other_edges)
        ):
            raise ValueError("comparison belief endpoint map is different")
        delta = np.array([log_se3(inverse(b["z"]) @ a["z"]) for a, b in zip(edges, other_edges)])
        other_cov = np.array([edge["cov"] for edge in other_edges])
        results["difference_from_comparison"] = {
            "relative_mean_rotation_deg": stats(np.linalg.norm(delta[:, :3], axis=1) * 180 / np.pi),
            "relative_mean_translation_m": stats(np.linalg.norm(delta[:, 3:], axis=1)),
            "covariance_frobenius": stats(np.linalg.norm(covariances - other_cov, axis=(1, 2))),
            "rotation_trace_ratio": stats(np.trace(covariances[:, :3, :3], axis1=1, axis2=2) /
                                          np.trace(other_cov[:, :3, :3], axis1=1, axis2=2)),
            "translation_trace_ratio": stats(np.trace(covariances[:, 3:, 3:], axis1=1, axis2=2) /
                                             np.trace(other_cov[:, 3:, 3:], axis1=1, axis2=2)),
        }
    for label in ("pose_only_native", "pose_only_exported", "graph"):
        translation, rotation = [], []
        for reference, estimate in zip(native, trajectories[label]):
            d = log_se3(inverse(reference) @ estimate)
            translation.append(np.linalg.norm(d[3:]))
            rotation.append(np.linalg.norm(d[:3]) * 180 / np.pi)
        results[label]["difference_to_native"] = {
            "translation_m": stats(translation), "rotation_deg": stats(rotation)}
        for component in (0, 1):
            write_tum(args.out / f"{label}_component_{component}.tum",
                      timestamps[component::2], trajectories[label][component::2])
    # This is a source-edge time series, not a GT-aligned trajectory comparison.
    native_residual = residuals(native, edges)
    graph_residual = residuals(trajectories["graph"], edges)
    with (args.out / "factor_residuals.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("ordinal", "from_index", "to_index", "to_timestamp_ns", "record_timestamp_ns",
                         "native_translation_m", "native_rotation_deg", "native_whitened_norm",
                         "graph_translation_m", "graph_rotation_deg", "graph_whitened_norm"))
        for edge, source, graph in zip(edges, native_residual, graph_residual):
            writer.writerow((edge["ordinal"], edge["i"], edge["j"], edge["tj"], edge["record"],
                             source[1], source[0], source[2], graph[1], graph[0], graph[2]))
    (args.out / "factor_summary.json").write_text(json.dumps(results, indent=2) + "\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    seconds = np.array([(edge["tj"] - edges[0]["ti"]) * 1e-9 for edge in edges])
    fig, axes = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
    axes[0].plot(seconds, native_residual[:, 1], lw=0.7)
    axes[0].set_ylabel("translation (m)")
    axes[1].plot(seconds, native_residual[:, 0], lw=0.7)
    axes[1].set_ylabel("rotation (deg)")
    axes[1].set_xlabel("time since first GLIM endpoint (s)")
    fig.suptitle("Exported GLIM belief vs scored native relative pose")
    fig.tight_layout()
    fig.savefig(args.out / "export_vs_native_edges.png", dpi=150)
    plt.close(fig)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
