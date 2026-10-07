#!/usr/bin/env python3
"""Read-only comparison/aggregation for the bounded EIG-2 factor audit."""
import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np

from eig2_single_source_preservation import inverse, pose
from kimera_eig2_revised_source_audit import reference_pose


TARGETS = ((1084, 1079, 1080), (1093, 1084, 1085),
           (1093, 1085, 1086), (1093, 1086, 1087))
PATTERN = re.compile(r"KIMERA_CBS_REVISION_EDGE_ROW,(\d+),k,(\d+),(\d+),")


def revisions(path):
    selected = {}
    with path.open(errors="replace") as stream:
        for line in stream:
            match = PATTERN.search(line)
            if not match:
                continue
            key = tuple(map(int, match.groups()))
            if key not in TARGETS:
                continue
            fields = line[match.start():].split("KIMERA_CBS_REVISION_EDGE_ROW,", 1)[1].strip().split(",")
            selected[key] = {
                "from_time": float(fields[4]), "to_time": float(fields[5]),
                "revision": int(fields[6]), "status": fields[7],
                "from_pose": np.array(fields[8:15], dtype=float),
                "to_pose": np.array(fields[15:22], dtype=float),
                "mu": np.array(fields[22:28], dtype=float),
                "covariance": np.array(fields[28:64], dtype=float),
            }
    return selected


def compare_revisions(old, new):
    output = []
    for key in TARGETS:
        a, b = old.get(key), new.get(key)
        if a is None or b is None:
            output.append({"update_edge": key, "present_old": a is not None,
                           "present_new": b is not None})
            continue
        output.append({
            "update_edge": key,
            "indices_revision_status_equal": a["revision"] == b["revision"] and a["status"] == b["status"],
            "timestamp_max_abs_s": max(abs(a["from_time"]-b["from_time"]),
                                       abs(a["to_time"]-b["to_time"])),
            **{f"{name}_max_abs": float(np.max(np.abs(a[name]-b[name])))
               for name in ("from_pose", "to_pose", "mu", "covariance")},
            "old_mu": a["mu"].tolist(), "new_mu": b["mu"].tolist(),
        })
    return output


def factors(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def family_costs(rows):
    groups = defaultdict(lambda: {"count": 0, "cost_before": 0.0,
                                  "cost_after": 0.0, "raw_before": 0.0,
                                  "raw_after": 0.0, "invalid_before": 0,
                                  "invalid_after": 0})
    for row in rows:
        out = groups[row["family"]]
        out["count"] += 1
        for name in ("cost_before", "cost_after", "raw_before", "raw_after"):
            value = number(row[name])
            if math.isfinite(value):
                out[name] += value
            else:
                out["invalid_before" if name.endswith("before") else "invalid_after"] += 1
    return {name: dict(value, cost_delta=value["cost_after"]-value["cost_before"])
            for name, value in groups.items()}


def local_subgraph(rows, keys):
    seeds = set(keys)
    adjacent = set(seeds)
    for row in rows:
        endpoints = set(row["keys"].split("|"))
        if endpoints & seeds:
            adjacent.update(endpoints)
    out = []
    for row in rows:
        endpoints = set(row["keys"].split("|"))
        if endpoints & adjacent:
            out.append({"slot": row["slot"], "family": row["family"],
                        "keys": row["keys"], "observations": row["observations"],
                        "cost_before": number(row["cost_before"]),
                        "cost_after": number(row["cost_after"]),
                        "imu_delta_t": number(row["imu_delta_t"]),
                        "prior_anchor_keys": row["prior_anchor_keys"]})
    return out


def pose_values(path):
    output = {}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["type"] != "pose":
                continue
            output[row["key"]] = pose(
                [float(row[f"p{i}"]) for i in range(3)],
                [float(row[name]) for name in ("qx", "qy", "qz", "qw")])
    return output


def relative_errors(directory, old_rows, reference):
    states = {label: pose_values(directory / name)
              for label, name in (("production", "values_after.csv"),
                                  ("reference", "values_reference.csv"))
              if (directory / name).exists()}
    output = []
    for update, i, j in TARGETS:
        if directory.name != f"update_{update}":
            continue
        row = old_rows[(update, i, j)]
        truth = inverse(reference_pose(reference, row["from_time"])) @ \
                reference_pose(reference, row["to_time"])
        for label, values in states.items():
            if f"x{i}" not in values or f"x{j}" not in values:
                continue
            relative = inverse(values[f"x{i}"]) @ values[f"x{j}"]
            difference = inverse(truth) @ relative
            angle = math.acos(max(-1.0, min(1.0,
                (np.trace(difference[:3, :3])-1.0)/2.0)))
            output.append({"state": label, "edge": f"{i}->{j}",
                           "relative_translation_xyz": relative[:3, 3].tolist(),
                           "translation_error_m": float(np.linalg.norm(
                               relative[:3, 3]-truth[:3, 3])),
                           "rotation_error_deg": float(np.degrees(angle))})
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--original-log", type=Path, required=True)
    parser.add_argument("--reproduction-log", type=Path, required=True)
    parser.add_argument("--snapshot-dir", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    original_rows = revisions(args.original_log)
    compared = compare_revisions(original_rows,
                                 revisions(args.reproduction_log))
    result = {"reproduction_comparison": compared, "updates": {}}
    matching = all(row.get("indices_revision_status_equal", False)
                   and row.get("timestamp_max_abs_s", math.inf) < 1e-6
                   and row.get("from_pose_max_abs", math.inf) < 1e-3
                   and row.get("to_pose_max_abs", math.inf) < 1e-3
                   and row.get("mu_max_abs", math.inf) < 1e-3
                   and row.get("covariance_max_abs", math.inf) < 1e-3
                   for row in compared)
    result["reproduction_close"] = matching
    if matching:
        reference = np.load(args.reference)
        for update, keys in ((1084, ("x1079", "x1080", "v1079", "v1080", "b1079", "b1080")),
                             (1093, tuple(f"{symbol}{index}" for symbol in "xvb"
                                          for index in range(1084, 1088)))):
            directory = args.snapshot_dir / f"update_{update}"
            rows = factors(directory / "factors_after.csv")
            result["updates"][str(update)] = {
                "family_costs": family_costs(rows),
                "local_subgraph": local_subgraph(rows, keys),
                "factor_count": len(rows),
                "objective_lines": (directory / "objective.csv").read_text().splitlines(),
                "relative_errors": relative_errors(directory, original_rows, reference),
            }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=True) + "\n")
    print(json.dumps({"reproduction_close": matching,
                      "target_rows_found": sum("mu_max_abs" in x for x in compared),
                      "factor_analysis_performed": matching}, indent=2))


if __name__ == "__main__":
    main()
