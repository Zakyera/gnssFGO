#!/usr/bin/env python3
"""Read-only first-divergence scan of two Kimera revision logs."""
import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np


PATTERN = re.compile(r"KIMERA_CBS_REVISION_EDGE_ROW,(\d+),k,(\d+),(\d+),")


def load(path, maximum_update):
    rows = {}
    with path.open(errors="replace") as stream:
        for line in stream:
            found = PATTERN.search(line)
            if not found:
                continue
            key = tuple(int(value) for value in found.groups())
            if key[0] > maximum_update:
                continue
            fields = line[found.start():].split("KIMERA_CBS_REVISION_EDGE_ROW,", 1)[1].split(",")
            rows[key] = (float(fields[4]), float(fields[5]),
                         int(fields[6]), fields[7],
                         np.array(fields[8:28], dtype=float),
                         np.array(fields[28:64], dtype=float))
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--original", required=True, type=Path)
    parser.add_argument("--reproduced", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--maximum-update", type=int, default=1100)
    args = parser.parse_args()
    original = load(args.original, args.maximum_update)
    reproduced = load(args.reproduced, args.maximum_update)
    updates = []
    for update in sorted({key[0] for key in original} &
                         {key[0] for key in reproduced}):
        keys_a = {key for key in original if key[0] == update}
        keys_b = {key for key in reproduced if key[0] == update}
        common = sorted(keys_a & keys_b)
        differences = []
        for key in common:
            a, b = original[key], reproduced[key]
            pose = float(np.max(np.abs(a[4][:14]-b[4][:14])))
            mu = float(np.max(np.abs(a[4][14:]-b[4][14:])))
            covariance = float(np.max(np.abs(a[5]-b[5])))
            differences.append((mu, pose, covariance, key))
        largest = max(differences, default=(float("nan"),)*3 + (None,))
        updates.append({
            "update": update,
            "old_rows": len(keys_a), "new_rows": len(keys_b),
            "common_rows": len(common),
            "source_identity_match": keys_a == keys_b,
            "timestamps_revisions_match": all(original[key][:4] == reproduced[key][:4]
                                              for key in common),
            "largest_mu_difference": largest[0],
            "largest_pose_difference": max((x[1] for x in differences), default=float("nan")),
            "largest_covariance_difference": max((x[2] for x in differences), default=float("nan")),
            "largest_mu_edge": largest[3],
        })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(updates[0]))
        writer.writeheader()
        writer.writerows(updates)
    print(json.dumps({
        "common_update_count": len(updates),
        "first_source_identity_mismatch": next((x for x in updates if not x["source_identity_match"]), None),
        "first_timestamp_or_revision_mismatch": next((x for x in updates if not x["timestamps_revisions_match"]), None),
        "first_mu_above": {str(threshold): next((x for x in updates
            if x["largest_mu_difference"] > threshold), None)
            for threshold in (1e-6, 1e-4, 1e-3, 1e-2, 5e-2)},
        "target_updates": [x for x in updates if x["update"] in (1083, 1084, 1092, 1093)],
    }, indent=2))


if __name__ == "__main__":
    main()
