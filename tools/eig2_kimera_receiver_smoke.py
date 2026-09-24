#!/usr/bin/env python3
"""Focused Kimera-only interpolated-factor replacement smoke test."""

import argparse
import csv
import json
import subprocess
import tempfile
from pathlib import Path


def read(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write(path, data):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(data[0]))
        writer.writeheader()
        writer.writerows(data)


def run(binary, imu, support, first, revisions, out):
    command = [str(binary), "--imu", str(imu), "--glim-beliefs", str(support),
               "--kimera-beliefs", str(first), "--mode", "kimera",
               "--timeline", "record", "--solver", "isam2", "--out", str(out)]
    if revisions is not None:
        command.extend(["--revision-beliefs", str(revisions)])
    subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads((out / "summary.json").read_text())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--binary", required=True, type=Path)
    p.add_argument("--imu", required=True, type=Path)
    p.add_argument("--support", required=True, type=Path)
    p.add_argument("--first", required=True, type=Path)
    p.add_argument("--revisions", required=True, type=Path)
    args = p.parse_args()
    with tempfile.TemporaryDirectory(prefix="eig2_kimera_smoke_") as directory:
        root = Path(directory)
        support = read(args.support)[:17]
        first = [r for r in read(args.first) if 4 <= int(r["from_index"]) <= 8]
        keys = {(r["from_index"], r["to_index"]) for r in first}
        revisions = [r for r in read(args.revisions)
                     if (r["from_index"], r["to_index"]) in keys]
        if len(first) != 5 or len(revisions) <= 10:
            raise AssertionError("small Kimera chain not present")
        write(root / "support.csv", support)
        write(root / "first.csv", first)
        write(root / "revisions.csv", revisions)
        k0 = run(args.binary, args.imu, root / "support.csv", root / "first.csv",
                 None, root / "k0")
        k1 = run(args.binary, args.imu, root / "support.csv", root / "first.csv",
                 root / "revisions.csv", root / "k1")
        if k0["beliefs_accepted"] != 5 or k1["beliefs_accepted"] != 5 or \
                k1["kimera_beliefs_accepted"] != 5 or \
                k1["glim_beliefs_accepted"] != 0 or \
                k1["revision_removals"] != len(revisions) - 5:
            raise AssertionError({"k0": k0, "k1": k1})
        if k1["revision_duplicates"] or k1["revision_stale"]:
            raise AssertionError("unexpected duplicate or stale revision")
        if k1["factor_kinds"]["exact"] == 5:
            raise AssertionError("Kimera endpoint interpolation was not exercised")
        print(json.dumps({"ids": 5, "revisions": len(revisions),
                          "replacements": k1["revision_removals"],
                          "skipped_no_support": k1["beliefs_skipped_no_support"],
                          "factor_kinds": k1["factor_kinds"],
                          "glim_measurements": k1["glim_beliefs_accepted"]}))


if __name__ == "__main__":
    main()
