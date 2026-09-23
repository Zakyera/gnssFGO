#!/usr/bin/env python3
"""Small end-to-end smoke test of the opt-in GrandTour revision ledger."""

import argparse
import csv
import json
import subprocess
import tempfile
from pathlib import Path


def write(path, rows, fields):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def run(binary, imu, first, revisions, out):
    subprocess.run([str(binary), "--imu", str(imu), "--glim-beliefs", str(first),
                    "--revision-beliefs", str(revisions), "--mode", "glim",
                    "--timeline", "record", "--solver", "isam2", "--out", str(out)],
                   check=True, capture_output=True, text=True)
    return json.loads((out / "summary.json").read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--beliefs", type=Path, required=True)
    args = parser.parse_args()
    with args.beliefs.open(newline="") as stream:
        original = list(csv.DictReader(stream))[:4]
    fields = list(original[0]) + ["revision"]
    with tempfile.TemporaryDirectory(prefix="eig2_revision_smoke_") as directory:
        root = Path(directory)
        first = root / "first.csv"
        write(first, original, list(original[0]))
        core = []
        for row in original:
            first_version = dict(row, revision="0")
            second = dict(row, revision="1")
            second["relative_mu_3"] = str(float(row["relative_mu_3"]) + 0.001)
            second["record_timestamp_ns"] = str(int(row["record_timestamp_ns"]) + 300_000_000)
            core.extend([first_version, second])
        core.sort(key=lambda row: (int(row["record_timestamp_ns"]),
                                   int(row["ordinal"]), int(row["revision"])))
        core_path = root / "core.csv"
        write(core_path, core, fields)
        core_result = run(args.binary, args.imu, first, core_path, root / "core_out")
        extra = list(core)
        duplicate = dict(next(row for row in core
                              if row["ordinal"] == original[0]["ordinal"]
                              and row["revision"] == "1"))
        duplicate["record_timestamp_ns"] = str(
            max(int(row["record_timestamp_ns"]) for row in core) + 100_000_000)
        stale = dict(next(row for row in core
                          if row["ordinal"] == original[0]["ordinal"]
                          and row["revision"] == "0"))
        stale["record_timestamp_ns"] = str(
            int(duplicate["record_timestamp_ns"]) + 100_000_000)
        extra.extend([duplicate, stale])
        extra_path = root / "extra.csv"
        write(extra_path, extra, fields)
        extra_result = run(args.binary, args.imu, first, extra_path,
                           root / "extra_out")
        if extra_result["revision_duplicates"] != 1 or \
                extra_result["revision_stale"] != 1 or \
                extra_result["revision_removals"] != 4:
            raise AssertionError(extra_result)
        if (root / "core_out/trajectory.tum").read_bytes() != \
                (root / "extra_out/trajectory.tum").read_bytes():
            raise AssertionError("duplicate/stale messages changed trajectory")
        if core_result["beliefs_accepted"] != 4:
            raise AssertionError(core_result)
        invalid = [dict(row) for row in core]
        invalid[0]["covariance_0_0"] = "-1"
        invalid_path = root / "invalid.csv"
        write(invalid_path, invalid, fields)
        invalid_out = root / "invalid_out"
        attempt = subprocess.run(
            [str(args.binary), "--imu", str(args.imu),
             "--glim-beliefs", str(first), "--revision-beliefs", str(invalid_path),
             "--mode", "glim", "--timeline", "record", "--solver", "isam2",
             "--out", str(invalid_out)], capture_output=True, text=True)
        if attempt.returncode == 0 or invalid_out.exists():
            raise AssertionError("invalid covariance was not rejected transactionally")
        print(json.dumps({"core": core_result["revision_removals"],
                          "duplicate": extra_result["revision_duplicates"],
                          "stale": extra_result["revision_stale"],
                          "trajectory_identical": True,
                          "invalid_covariance_rejected": True}))


if __name__ == "__main__":
    main()
