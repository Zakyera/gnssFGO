#!/usr/bin/env python3
"""Validate and combine two diagnostic source-revision streams for joint iSAM2."""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def load(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames), list(reader)


def validate(first_path, revision_path, source):
    first_fields, first = load(first_path)
    revision_fields, revisions = load(revision_path)
    if revision_fields != first_fields + ["revision"]:
        raise ValueError(f"{source} revision CSV schema differs from first-publication schema")
    first_by_id = {}
    for row in first:
        if row["source_agent"] != source:
            raise ValueError(f"wrong source in first CSV: {source}")
        identity = (int(row["from_index"]), int(row["to_index"]))
        if identity in first_by_id:
            raise ValueError(f"duplicate first-publication ID: {source} {identity}")
        first_by_id[identity] = row
    versions = defaultdict(list)
    for row in revisions:
        if row["source_agent"] != source:
            raise ValueError(f"wrong source in revision CSV: {source}")
        identity = (int(row["from_index"]), int(row["to_index"]))
        if identity not in first_by_id:
            raise ValueError(f"revision without first publication: {source} {identity}")
        versions[identity].append(row)
    if set(versions) != set(first_by_id):
        raise ValueError(f"missing revised IDs from {source}")
    for identity, history in versions.items():
        template = first_by_id[identity]
        if {k: v for k, v in history[0].items() if k != "revision"} != template:
            raise ValueError(f"revision zero payload mismatch: {source} {identity}")
        times = []
        for n, row in enumerate(history):
            if int(row["revision"]) != n:
                raise ValueError(f"nonconsecutive revision: {source} {identity}")
            if any(row[key] != template[key] for key in (
                    "from_index", "to_index", "from_timestamp_ns", "to_timestamp_ns")):
                raise ValueError(f"physical edge changed: {source} {identity}")
            times.append(int(row["record_timestamp_ns"]))
        if times != sorted(times):
            raise ValueError(f"nonmonotone availability: {source} {identity}")
    return revision_fields, first, revisions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--glim-first", required=True)
    parser.add_argument("--glim-revisions", required=True)
    parser.add_argument("--kimera-first", required=True)
    parser.add_argument("--kimera-revisions", required=True)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    g_fields, g_first, g_revisions = validate(
        args.glim_first, args.glim_revisions, "g")
    k_fields, k_first, k_revisions = validate(
        args.kimera_first, args.kimera_revisions, "k")
    if g_fields != k_fields:
        raise ValueError("GLIM and Kimera revision CSV schemas differ")
    combined = sorted(g_revisions + k_revisions,
                      key=lambda row: (int(row["record_timestamp_ns"]),
                                       row["source_agent"], int(row["ordinal"]),
                                       int(row["revision"])))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=g_fields)
        writer.writeheader()
        writer.writerows(combined)
    print(json.dumps({"glim_ids": len(g_first), "kimera_ids": len(k_first),
                      "glim_versions": len(g_revisions),
                      "kimera_versions": len(k_revisions),
                      "combined_versions": len(combined)}, indent=2))


if __name__ == "__main__":
    main()
