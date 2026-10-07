#!/usr/bin/env python3
"""Audit full 6x6 covariance transport in the bounded live CBS run."""

import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

import numpy as np


ANSI = re.compile(r"\x1b\[[0-9;]*m")
G2K = "KIMERA_CBS_SOURCE_TRANSPORT_ROW,"
K2G = "GLIM_CBS_SOURCE_TRANSPORT_ROW,"
BPSAM = "CBS_BPSAM_SOURCE_TRANSPORT_ROW,"


def parse_matrix(token):
    return np.array([[float(x) for x in row.split(":")]
                     for row in token.split(";")], dtype=float)


def load_source(path):
    values = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["from_index"]), int(row["to_index"]),
                   int(row["revision"]))
            matrix = np.array(
                [[float(row[f"covariance_{i}_{j}"]) for j in range(6)]
                 for i in range(6)], dtype=float)
            values[key].append(matrix)
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--glim-revisions", type=Path, required=True)
    parser.add_argument("--kimera-revisions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sources = {
        G2K: load_source(args.glim_revisions),
        K2G: load_source(args.kimera_revisions),
    }
    checks = defaultdict(lambda: defaultdict(list))
    unmatched = defaultdict(int)
    with args.log.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            for marker in (G2K, K2G):
                if marker not in line:
                    continue
                fields = ANSI.sub("", line.split(marker, 1)[1]).strip().split(",")
                if len(fields) < 6:
                    unmatched[marker] += 1
                    break
                key = tuple(map(int, fields[1:4]))
                raw, used = parse_matrix(fields[4]), parse_matrix(fields[5])
                if raw.shape != (6, 6) or used.shape != (6, 6):
                    raise ValueError(f"bad covariance shape for {marker}")
                candidates = sources[marker].get(key, [])
                if not candidates:
                    unmatched[marker] += 1
                else:
                    checks[marker]["source_to_receiver_max_abs"].append(
                        min(np.max(np.abs(raw - candidate))
                            for candidate in candidates))
                checks[marker]["receiver_to_factor_max_abs"].append(
                    float(np.max(np.abs(raw - used))))
                checks[marker]["used_min_eigenvalue"].append(
                    float(np.linalg.eigvalsh(used)[0]))
                break
            if BPSAM in line:
                fields = ANSI.sub("", line.split(BPSAM, 1)[1]).strip().split(",")
                if len(fields) < 7:
                    unmatched[BPSAM] += 1
                    continue
                raw, used = parse_matrix(fields[5]), parse_matrix(fields[6])
                checks[BPSAM]["source_to_bpsam_used_max_abs"].append(
                    float(np.max(np.abs(raw - used))))
                checks[BPSAM]["used_min_eigenvalue"].append(
                    float(np.linalg.eigvalsh(used)[0]))

    result = {}
    for marker, values in checks.items():
        result[marker.rstrip(",")] = {
            name: {"count": len(sequence),
                   "max": float(max(sequence)) if sequence else None,
                   "min": float(min(sequence)) if sequence else None}
            for name, sequence in values.items()
        }
        result[marker.rstrip(",")]["unmatched"] = unmatched[marker]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
