#!/usr/bin/env python3
"""Convert GLIM's diagnostic publication/revision rows into receiver CSV inputs.

The first revision comes from the recorded ROS belief CSV. Later full mean and
covariance revisions come from the same smoother's flag-gated audit log.
This is an offline diagnostic transport, not the production CBS wire format.
"""

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np

from eig2_glim_revision_topology_audit import (
    covariance_token, decode_pose, parse_log, relative, tangent_token,
)
from eig2_single_source_preservation import exp_se3, inverse, log_se3


REVISION = re.compile(r"GLIM_CBS_REVISION_BELIEF_ROW,([^\x1b\r\n]+)")


def write_csv(path, rows, fields):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--beliefs", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--retired-pose-count", type=int, default=592)
    parser.add_argument("--omit-finalized", action="store_true",
                        help="keep the full source window including right-censored active endpoints; do not claim finalized means")
    parser.add_argument("--repair-submicrosecond-timestamp-roundoff", action="store_true",
                        help="clamp a later revision up to revision-0 bag receipt only when the inversion is at most 1000 ns")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    with args.beliefs.open(newline="") as stream:
        original = list(csv.DictReader(stream))
    selected = [row for row in original
                if int(row["to_index"]) < args.retired_pose_count]
    if len(selected) != args.retired_pose_count - 2:
        raise ValueError("unexpected 0.20 s paired edge population")
    by_id = {(int(row["from_index"]), int(row["to_index"])): row
             for row in selected}
    if len(by_id) != len(selected):
        raise ValueError("duplicate first-publication BeliefID")
    fields = list(original[0])
    write_csv(args.out / "first.csv", selected, fields)

    active, _, published = parse_log(args.log)
    versions = defaultdict(list)
    for key, row in by_id.items():
        pub = published[key]
        if len(pub) != 1:
            raise ValueError(f"publication count for {key}: {len(pub)}")
        source = pub[0]
        mu = np.array([float(row[f"relative_mu_{i}"]) for i in range(6)])
        cov = np.array([[float(row[f"covariance_{r}_{c}"])
                         for c in range(6)] for r in range(6)])
        if np.max(np.abs(mu - source["mu"])) > 1e-12 or np.max(
                np.abs(cov - covariance_token(source["covariance"]))) > 1e-12:
            raise ValueError(f"bag/log first-publication mismatch {key}")
        first = dict(row)
        first["revision"] = "0"
        versions[key].append(first)

    errors = []
    repaired_arrivals = 0
    largest_arrival_repair_ns = 0
    for line in args.log.open(errors="replace"):
        if "GLIM_CBS_REVISION_ERROR_ROW," in line:
            errors.append(line.strip())
        match = REVISION.search(line)
        if match is None:
            continue
        parts = match.group(1).split(",")
        if len(parts) != 10:
            raise ValueError(f"revision row width {len(parts)}")
        source, i, j, revision, record, update, ti, tj, mu_token, cov_token = parts
        key = (int(i), int(j))
        if key not in by_id:
            continue
        template = by_id[key]
        if source != template["source_agent"]:
            raise ValueError(f"source changed for {key}")
        if abs(float(ti) - float(template["from_timestamp_sec"])) > 1e-6 or \
                abs(float(tj) - float(template["to_timestamp_sec"])) > 1e-6:
            raise ValueError(f"endpoint timestamps changed for {key}")
        if int(revision) != len(versions[key]):
            raise ValueError(f"non-consecutive revision for {key}")
        mu = tangent_token(mu_token)
        cov = covariance_token(cov_token)
        if mu.shape != (6,) or not np.all(np.isfinite(mu)):
            raise ValueError(f"invalid revision mean for {key}")
        if np.linalg.eigvalsh(cov)[0] <= 0.0:
            raise ValueError(f"nonpositive revision covariance for {key}")
        revised = dict(template)
        revised["revision"] = revision
        record_ns = round(float(record) * 1e9)
        first_record_ns = int(versions[key][0]["record_timestamp_ns"])
        if record_ns < first_record_ns:
            discrepancy_ns = first_record_ns - record_ns
            if not args.repair_submicrosecond_timestamp_roundoff or discrepancy_ns > 1000:
                raise ValueError(f"revision precedes first bag receipt for {key} by {discrepancy_ns} ns")
            record_ns = first_record_ns
            repaired_arrivals += 1
            largest_arrival_repair_ns = max(largest_arrival_repair_ns, discrepancy_ns)
        revised["record_timestamp_ns"] = str(record_ns)
        for coordinate in range(6):
            revised[f"relative_mu_{coordinate}"] = format(mu[coordinate], ".17g")
        for r in range(6):
            for c in range(6):
                revised[f"covariance_{r}_{c}"] = format(cov[r, c], ".17g")
        versions[key].append(revised)

    if errors:
        raise ValueError(f"source revision covariance errors: {errors[:3]}")
    revision_rows = sorted(
        [row for group in versions.values() for row in group],
        key=lambda row: (int(row["record_timestamp_ns"]),
                         int(row["ordinal"]), int(row["revision"])))
    revision_fields = fields + ["revision"]
    write_csv(args.out / "revisions.csv", revision_rows, revision_fields)
    final_revision = [versions[(int(row["from_index"]),
                                int(row["to_index"]))][-1]
                      for row in selected]
    write_csv(args.out / "last_revision.csv", final_revision, revision_fields)

    if args.omit_finalized:
        counts = [len(versions[key]) for key in by_id]
        rotation_ratios = []
        translation_ratios = []
        for first, last in zip(selected, final_revision):
            rotation_ratios.append(sum(float(last[f"covariance_{k}_{k}"])
                                       for k in range(3)) /
                                   sum(float(first[f"covariance_{k}_{k}"])
                                       for k in range(3)))
            translation_ratios.append(sum(float(last[f"covariance_{k}_{k}"])
                                          for k in range(3, 6)) /
                                      sum(float(first[f"covariance_{k}_{k}"])
                                          for k in range(3, 6)))
        summary = {
            "belief_ids": len(by_id),
            "revision_messages": len(revision_rows),
            "revisions_per_belief": {
                "mean": float(np.mean(counts)),
                "median": float(np.median(counts)),
                "maximum": max(counts), "minimum": min(counts)},
            "last_over_first_rotation_covariance_trace_ratio": {
                "median": float(np.median(rotation_ratios)),
                "minimum": min(rotation_ratios), "maximum": max(rotation_ratios)},
            "last_over_first_translation_covariance_trace_ratio": {
                "median": float(np.median(translation_ratios)),
                "minimum": min(translation_ratios), "maximum": max(translation_ratios)},
            "right_censored_finalized_means": True,
            "submicrosecond_arrivals_repaired": repaired_arrivals,
            "largest_arrival_repair_ns": largest_arrival_repair_ns,
            "source_errors": errors,
        }
        (args.out / "export_summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        return

    final_poses = {index: observations[-1]["pose"]
                   for index, observations in active.items()
                   if observations and index < args.retired_pose_count}
    if len(final_poses) != args.retired_pose_count:
        raise ValueError("missing fully retired source pose")
    finalized = []
    mean_errors = []
    translation_errors = []
    rotation_errors_deg = []
    covariance_rotation_ratios = []
    covariance_translation_ratios = []
    target_mu_by_id = {}
    for original_row, latest in zip(selected, final_revision):
        i, j = int(original_row["from_index"]), int(original_row["to_index"])
        final_mu = log_se3(relative(final_poses[i], final_poses[j]))
        target_mu_by_id[(i, j)] = final_mu
        latest_mu = np.array([float(latest[f"relative_mu_{c}"])
                              for c in range(6)])
        local_error = log_se3(inverse(exp_se3(latest_mu)) @ exp_se3(final_mu))
        mean_errors.append(float(np.linalg.norm(local_error)))
        translation_errors.append(float(np.linalg.norm(local_error[3:])))
        rotation_errors_deg.append(float(np.linalg.norm(local_error[:3]) * 180 / math.pi))
        final_row = dict(latest)
        for coordinate in range(6):
            final_row[f"relative_mu_{coordinate}"] = format(
                final_mu[coordinate], ".17g")
        finalized.append(final_row)
        old_cov = np.array([[float(original_row[f"covariance_{r}_{c}"])
                             for c in range(6)] for r in range(6)])
        new_cov = np.array([[float(latest[f"covariance_{r}_{c}"])
                             for c in range(6)] for r in range(6)])
        covariance_rotation_ratios.append(
            float(np.trace(new_cov[:3, :3]) / np.trace(old_cov[:3, :3])))
        covariance_translation_ratios.append(
            float(np.trace(new_cov[3:, 3:]) / np.trace(old_cov[3:, 3:])))
    write_csv(args.out / "finalized.csv", finalized, revision_fields)
    counts = [len(versions[key]) for key in by_id]
    mean_evolution = []
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        translation = []
        rotation = []
        for key, group in versions.items():
            row = group[round((len(group) - 1) * fraction)]
            mu = np.array([float(row[f"relative_mu_{k}"]) for k in range(6)])
            delta = log_se3(inverse(exp_se3(mu)) @ exp_se3(target_mu_by_id[key]))
            translation.append(float(np.linalg.norm(delta[3:])))
            rotation.append(float(np.linalg.norm(delta[:3]) * 180 / math.pi))
        mean_evolution.append({
            "revision_fraction": fraction,
            "translation_error_median_m": float(np.median(translation)),
            "translation_error_rms_m": float(np.sqrt(np.mean(np.square(translation)))),
            "rotation_error_median_deg": float(np.median(rotation)),
            "rotation_error_rms_deg": float(np.sqrt(np.mean(np.square(rotation))))})
    summary = {
        "belief_ids": len(by_id),
        "revision_messages": len(revision_rows),
        "revisions_per_belief": {
            "mean": float(np.mean(counts)), "median": float(np.median(counts)),
            "maximum": max(counts), "minimum": min(counts)},
        "revision_mean_error_to_finalized_by_fraction": mean_evolution,
        "last_joint_revision_to_finalized_mean_tangent_norm": {
            "median": float(np.median(mean_errors)),
            "rms": float(np.sqrt(np.mean(np.square(mean_errors)))),
            "maximum": max(mean_errors)},
        "last_joint_revision_to_finalized_translation_m": {
            "median": float(np.median(translation_errors)),
            "rms": float(np.sqrt(np.mean(np.square(translation_errors)))),
            "maximum": max(translation_errors)},
        "last_joint_revision_to_finalized_rotation_deg": {
            "median": float(np.median(rotation_errors_deg)),
            "rms": float(np.sqrt(np.mean(np.square(rotation_errors_deg)))),
            "maximum": max(rotation_errors_deg)},
        "last_over_first_rotation_covariance_trace_ratio": {
            "median": float(np.median(covariance_rotation_ratios)),
            "minimum": min(covariance_rotation_ratios),
            "maximum": max(covariance_rotation_ratios)},
        "last_over_first_translation_covariance_trace_ratio": {
            "median": float(np.median(covariance_translation_ratios)),
            "minimum": min(covariance_translation_ratios),
            "maximum": max(covariance_translation_ratios)},
        "source_errors": errors,
    }
    (args.out / "export_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
