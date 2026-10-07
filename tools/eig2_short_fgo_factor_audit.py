#!/usr/bin/env python3
"""Read-only native factor covariance and residual audit for short gnssFGO replay."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def distribution(values):
    a = np.asarray(values, dtype=float)
    if len(a) == 0:
        return None
    return {"count": int(len(a)), "min": float(a.min()),
            "p05": float(np.percentile(a, 5)), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--on", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    source_rows = {}
    for source in ("glim", "kimera"):
        for row in read_csv(args.on / "belief_audit" / f"{source}_revisions.csv"):
            source_rows[(source, int(row["ordinal"]))] = row
    result = {}
    for graph in ("glim", "kimera", "joint"):
        metrics = {source: {key: [] for key in (
            "rotation_trace_rad2", "translation_trace_m2", "full_min_eigenvalue_mixed",
            "full_max_eigenvalue_mixed", "full_condition_mixed", "full_logdet_mixed",
            "cross_covariance_fro_rad_m", "information_rotation_trace_rad_inv2",
            "information_translation_trace_m_inv2", "rotation_residual_rad",
            "translation_residual_m", "mahalanobis_squared")}
                   for source in ("glim", "kimera")}
        for residual in read_csv(args.on / f"gnssfgo_{graph}_residual" /
                                 "belief_residuals.csv"):
            source = residual["source"]
            if source not in metrics:
                raise ValueError(f"unexpected factor source {source}")
            row = source_rows[(source, int(residual["ordinal"]))]
            covariance = np.asarray([[float(row[f"covariance_{i}_{j}"])
                                      for j in range(6)] for i in range(6)])
            info = np.linalg.inv(covariance)
            eig = np.linalg.eigvalsh(covariance)
            values = {
                "rotation_trace_rad2": np.trace(covariance[:3, :3]),
                "translation_trace_m2": np.trace(covariance[3:, 3:]),
                "full_min_eigenvalue_mixed": eig[0],
                "full_max_eigenvalue_mixed": eig[-1],
                "full_condition_mixed": eig[-1] / eig[0],
                "full_logdet_mixed": np.linalg.slogdet(covariance)[1],
                "cross_covariance_fro_rad_m": np.linalg.norm(covariance[:3, 3:]),
                "information_rotation_trace_rad_inv2": np.trace(info[:3, :3]),
                "information_translation_trace_m_inv2": np.trace(info[3:, 3:]),
                "rotation_residual_rad": float(residual["rotation_residual"]),
                "translation_residual_m": float(residual["translation_residual"]),
                "mahalanobis_squared": float(residual["mahalanobis_squared"]),
            }
            for key, value in values.items():
                metrics[source][key].append(float(value))
            vector = np.asarray([float(residual[f"r{i}"]) for i in range(6)])
            check = float(vector @ np.linalg.solve(covariance, vector))
            if abs(check - values["mahalanobis_squared"]) > 1e-6 * max(1, check):
                raise ValueError("factor residual Mahalanobis mismatch")
        result[graph] = {source: {key: distribution(values)
                                  for key, values in source_metrics.items()}
                         for source, source_metrics in metrics.items()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print({g: {s: v["mahalanobis_squared"]["count"]
                if v["mahalanobis_squared"] else 0 for s, v in by_source.items()}
           for g, by_source in result.items()})


if __name__ == "__main__":
    main()
