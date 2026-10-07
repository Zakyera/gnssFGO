#!/usr/bin/env python3
"""Require identical OFF/ON physical edge populations before A–D scoring."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identities(rows_):
    return [(r["source_agent"], r["from_index"], r["to_index"],
             r["from_timestamp_ns"], r["to_timestamp_ns"])
            for r in rows_]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--products", required=True, type=Path)
    args = parser.parse_args()
    result = {}
    for source in ("glim", "kimera"):
        off_path = args.products / "off" / source / "first.csv"
        on_path = args.products / "on" / source / "first.csv"
        off, on = rows(off_path), rows(on_path)
        if identities(off) != identities(on):
            mismatch = next(((index, a, b) for index, (a, b) in enumerate(
                zip(identities(off), identities(on))) if a != b), None)
            raise ValueError(f"{source} OFF/ON edge population differs: "
                             f"counts {len(off)}/{len(on)}, first={mismatch}")
        result[source] = {
            "matched_edges": len(off),
            "off_first_sha256": digest(off_path),
            "on_first_sha256": digest(on_path),
            "off_revisions_sha256": digest(
                args.products / "off" / source / "revisions.csv"),
            "on_revisions_sha256": digest(
                args.products / "on" / source / "revisions.csv"),
            "first_physical_ns": int(off[0]["from_timestamp_ns"]),
            "last_physical_ns": int(off[-1]["to_timestamp_ns"]),
        }
    output = args.products / "source_match.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
