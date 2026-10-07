#!/usr/bin/env python3
"""Extract Kimera optimizer and increment-composed paths from existing logs."""

import argparse
import csv
import re
from pathlib import Path


MARKER = "KIMERA_BACKEND_POSE_PATH_ROW,"
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--odometry-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    optimizer = []
    increments = []
    seen = set()
    stamps_by_frame = {}
    with args.odometry_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamps_by_frame[int(row["field.header.seq"])] = (
                int(row["field.header.stamp"]) * 1e-9
            )
    with args.log.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if MARKER not in line:
                continue
            fields = ANSI.sub("", line.split(MARKER, 1)[1]).strip().split(",")
            if len(fields) < 16:
                continue
            frame = int(fields[1])
            if frame in seen:
                continue
            seen.add(frame)
            if frame not in stamps_by_frame:
                raise ValueError(f"no exact ROS timestamp for backend frame {frame}")
            timestamp = stamps_by_frame[frame]
            optimizer.append((timestamp, *map(float, fields[2:5]),
                              *map(float, fields[6:9]), float(fields[5])))
            increments.append((timestamp, *map(float, fields[9:12]),
                               *map(float, fields[13:16]), float(fields[12])))
    for name, rows in (("kimera_optimizer.tum", optimizer),
                       ("kimera_increments.tum", increments)):
        with (args.out / name).open("w", encoding="utf-8") as stream:
            for row in rows:
                stream.write(" ".join(f"{value:.17g}" for value in row) + "\n")
    print(f"optimizer_rows={len(optimizer)} increments_rows={len(increments)}")


if __name__ == "__main__":
    main()
