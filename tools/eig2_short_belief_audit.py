#!/usr/bin/env python3
"""Read-only audit/export of a bounded live CBS belief bag.

The CSVs preserve the ROS message's full 6x6 right-local [phi,rho]
covariance and the sender's revision/physical-endpoint metadata. They are
diagnostic source products; this script does not alter estimator inputs.
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag


TOPICS = {
    "/kimera/cbs/odom_belief_in": "glim",
    "/kimera/cbs/odom_belief_out": "kimera",
}


def summary(values):
    data = np.asarray(values, dtype=float)
    if not len(data):
        return None
    return {
        "count": int(len(data)),
        "min": float(np.min(data)),
        "p05": float(np.percentile(data, 5)),
        "median": float(np.median(data)),
        "p95": float(np.percentile(data, 95)),
        "max": float(np.max(data)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    columns = [
        "ordinal", "source_agent", "from_index", "to_index", "revision",
        "from_timestamp_ns", "to_timestamp_ns", "record_timestamp_ns",
    ]
    columns += [f"relative_mu_{i}" for i in range(6)]
    columns += [f"covariance_{i}_{j}" for i in range(6) for j in range(6)]
    outputs = {}
    rows = defaultdict(list)
    metrics = defaultdict(lambda: defaultdict(list))
    versions = defaultdict(lambda: defaultdict(list))
    seen = defaultdict(dict)
    duplicate_counts = defaultdict(int)
    stale_counts = defaultdict(int)
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=list(TOPICS)):
            source = TOPICS[topic]
            for belief in message.beliefs:
                covariance = np.asarray(belief.covariance, dtype=float).reshape(6, 6)
                eigenvalues = np.linalg.eigvalsh(covariance)
                rot = covariance[:3, :3]
                trans = covariance[3:, 3:]
                rot_eig = np.linalg.eigvalsh(rot)
                trans_eig = np.linalg.eigvalsh(trans)
                if not np.isfinite(covariance).all():
                    raise ValueError(f"{source} nonfinite covariance")
                if np.max(np.abs(covariance - covariance.T)) > 1e-9:
                    raise ValueError(f"{source} nonsymmetric covariance")
                if eigenvalues[0] <= 0:
                    raise ValueError(f"{source} non-PD covariance")
                key = (int(belief.source_agent), int(belief.from_pose_index),
                       int(belief.to_pose_index))
                revision = int(belief.revision)
                first = key not in seen[source]
                if first and revision != 0:
                    raise ValueError(f"{source} first version not revision 0: {key}")
                if not first and revision <= seen[source][key]:
                    if revision == seen[source][key]:
                        duplicate_counts[source] += 1
                    else:
                        stale_counts[source] += 1
                else:
                    seen[source][key] = revision
                record_ns = int(belief.header.stamp.to_nsec())
                if record_ns == 0:
                    record_ns = int(bag_stamp.to_nsec())
                row = {
                    "ordinal": len(rows[source]),
                    "source_agent": chr(int(belief.source_agent)),
                    "from_index": key[1], "to_index": key[2],
                    "revision": revision,
                    "from_timestamp_ns": round(float(belief.from_stamp_sec) * 1e9),
                    "to_timestamp_ns": round(float(belief.to_stamp_sec) * 1e9),
                    "record_timestamp_ns": record_ns,
                }
                row.update({f"relative_mu_{i}": float(belief.relative_mu[i])
                            for i in range(6)})
                row.update({f"covariance_{i}_{j}": float(covariance[i, j])
                            for i in range(6) for j in range(6)})
                rows[source].append(row)
                values = {
                    "rotation_trace_rad2": np.trace(rot),
                    "translation_trace_m2": np.trace(trans),
                    "rotation_eigen_min_rad2": rot_eig[0],
                    "rotation_eigen_max_rad2": rot_eig[-1],
                    "translation_eigen_min_m2": trans_eig[0],
                    "translation_eigen_max_m2": trans_eig[-1],
                    "full_eigen_min_mixed": eigenvalues[0],
                    "full_eigen_max_mixed": eigenvalues[-1],
                    "full_condition_mixed": eigenvalues[-1] / eigenvalues[0],
                    "full_logdet_mixed": np.linalg.slogdet(covariance)[1],
                    "cross_fro_rad_m": np.linalg.norm(covariance[:3, 3:]),
                    "latency_s": (record_ns - row["to_timestamp_ns"]) * 1e-9,
                }
                for name, value in values.items():
                    metrics[source][name].append(float(value))
                if first or revision == seen[source][key]:
                    if not versions[source][key] or revision > versions[source][key][-1][0]:
                        versions[source][key].append((revision, values))

    report = {}
    representative = {}
    combined_rows = sorted(
        (row for source_rows in rows.values() for row in source_rows),
        key=lambda row: (row["record_timestamp_ns"], row["source_agent"],
                         row["ordinal"]),
    )
    with (args.out / "joint_revisions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(combined_rows)
    for source, source_rows in rows.items():
        path = args.out / f"{source}_revisions.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(source_rows)
        evolution = defaultdict(list)
        for edge_versions in versions[source].values():
            n = len(edge_versions)
            for label, fraction in (("revision_0", 0), ("age_25", .25),
                                    ("age_50", .50), ("age_75", .75),
                                    ("last", 1)):
                _, value = edge_versions[round((n - 1) * fraction)]
                for metric_name, metric_value in value.items():
                    evolution[(label, metric_name)].append(metric_value)
        report[source] = {
            "csv": str(path),
            "belief_ids": len(versions[source]),
            "messages": len(source_rows),
            "revisions": len(source_rows) - len(versions[source]),
            "duplicates": duplicate_counts[source],
            "stale": stale_counts[source],
            "metrics": {name: summary(values)
                        for name, values in metrics[source].items()},
            "revision_evolution": {
                label: {name: summary(evolution[(label, name)])
                        for name in metrics[source]}
                for label in ("revision_0", "age_25", "age_50", "age_75", "last")
            },
        }
        rot_trace = np.array([
            sum(row[f"covariance_{i}_{i}"] for i in range(3))
            for row in source_rows], dtype=float)
        trans_trace = np.array([
            sum(row[f"covariance_{i}_{i}"] for i in range(3, 6))
            for row in source_rows], dtype=float)
        selected = {
            "first": 0,
            "min_rotation_trace": int(np.argmin(rot_trace)),
            "median_rotation_trace": int(np.argmin(np.abs(
                rot_trace - np.median(rot_trace)))),
            "max_rotation_trace": int(np.argmax(rot_trace)),
            "min_translation_trace": int(np.argmin(trans_trace)),
            "median_translation_trace": int(np.argmin(np.abs(
                trans_trace - np.median(trans_trace)))),
            "max_translation_trace": int(np.argmax(trans_trace)),
        }
        representative[source] = {}
        for label, index in selected.items():
            row = source_rows[index]
            covariance = np.array(
                [[row[f"covariance_{i}_{j}"] for j in range(6)]
                 for i in range(6)], dtype=float)
            rot_eig, rot_vec = np.linalg.eigh(covariance[:3, :3])
            trans_eig, trans_vec = np.linalg.eigh(covariance[3:, 3:])
            representative[source][label] = {
                "ordinal": index,
                "belief_id": [row["source_agent"], row["from_index"],
                              row["to_index"]],
                "revision": row["revision"],
                "rotation_eigenvalues_rad2": rot_eig.tolist(),
                "rotation_eigenvectors_columns_body": rot_vec.tolist(),
                "translation_eigenvalues_m2": trans_eig.tolist(),
                "translation_eigenvectors_columns_body": trans_vec.tolist(),
                "cross_covariance_rad_m": covariance[:3, 3:].tolist(),
            }
    with (args.out / "belief_audit.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (args.out / "principal_directions.json").open("w", encoding="utf-8") as stream:
        json.dump(representative, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({s: {"ids": v["belief_ids"], "messages": v["messages"]}
                      for s, v in report.items()}, sort_keys=True))


if __name__ == "__main__":
    main()
