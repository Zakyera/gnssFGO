#!/usr/bin/env python3
"""Prepare bounded, unchanged source rows for the existing GrandTour CT replay."""

import argparse
import csv
from pathlib import Path


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def first_versions(rows):
    first = {}
    for row in rows:
        key = (row["source_agent"], row["from_index"], row["to_index"])
        if key not in first:
            if int(row["revision"]) != 0:
                raise ValueError(f"first version is not revision 0: {key}")
            first[key] = row
    return sorted(first.values(), key=lambda row: (int(row["to_timestamp_ns"]),
                                                   int(row["record_timestamp_ns"]),
                                                   int(row["ordinal"])))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--glim-revisions", type=Path, required=True)
    parser.add_argument("--kimera-revisions", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    glim_fields, glim_rows = read_csv(args.glim_revisions)
    kimera_fields, kimera_rows = read_csv(args.kimera_revisions)
    if glim_fields != kimera_fields:
        raise ValueError("source CSV schemas differ")
    glim_first = first_versions(glim_rows)
    kimera_first = first_versions(kimera_rows)
    all_rows = glim_rows + kimera_rows
    last_ns = max(max(int(row["to_timestamp_ns"]),
                      int(row["record_timestamp_ns"])) for row in all_rows)
    imu_fields, imu_rows = read_csv(args.imu)
    imu_bounded = [row for row in imu_rows
                   if int(row["timestamp_ns"]) <= last_ns + 500_000_000]
    if not imu_bounded or int(imu_bounded[-1]["timestamp_ns"]) <= last_ns:
        raise ValueError("bounded IMU does not bracket final belief arrival")
    write_csv(args.out / "imu_short.csv", imu_fields, imu_bounded)
    write_csv(args.out / "glim_first.csv", glim_fields, glim_first)
    write_csv(args.out / "kimera_first.csv", kimera_fields, kimera_first)
    joint = sorted(all_rows,
                   key=lambda row: (int(row["record_timestamp_ns"]),
                                    row["source_agent"], int(row["ordinal"])))
    write_csv(args.out / "joint_revisions.csv", glim_fields, joint)
    print(f"imu={len(imu_bounded)} glim_ids={len(glim_first)} "
          f"kimera_ids={len(kimera_first)} revisions={len(joint)}")


if __name__ == "__main__":
    main()
