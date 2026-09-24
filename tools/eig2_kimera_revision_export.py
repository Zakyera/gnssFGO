#!/usr/bin/env python3
"""Audit Kimera's same-snapshot fixed-lag revisions; export single-source cases.

The first version is always the actual recorded ROS belief. Later versions are
diagnostic snapshots, causally available no earlier than the matching recorded
ROS odometry output. No covariance is synthesized or carried forward as a
newly computed covariance. This is an offline audit transport, not CBS wire.
"""

import argparse
import csv
import json
import math
import re
from collections import defaultdict, deque
from decimal import Decimal
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from eig2_single_source_preservation import exp_se3, inverse, log_se3, write_tum


EDGE = re.compile(r"KIMERA_CBS_REVISION_EDGE_ROW,([^\x1b\r\n]+)")
RETIRE = re.compile(r"KIMERA_CBS_REVISION_POSE_RETIRE_ROW,([^\x1b\r\n]+)")
ERROR = re.compile(r"KIMERA_CBS_REVISION_EDGE_ERROR_ROW,([^\x1b\r\n]+)")


def rows(path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def numeric_summary(values):
    a = np.asarray(values, dtype=float)
    return {"count": len(a), "median": float(np.median(a)),
            "rms": float(np.sqrt(np.mean(a * a))),
            "p95": float(np.percentile(a, 95)), "maximum": float(np.max(a))}


def parse_pose(values):
    x, y, z, qw, qx, qy, qz = map(float, values)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    result[:3, 3] = [x, y, z]
    return result


def mu_cov(row):
    mu = np.array([float(row[f"relative_mu_{i}"]) for i in range(6)])
    cov = np.array([[float(row[f"covariance_{i}_{j}"]) for j in range(6)]
                    for i in range(6)])
    return mu, cov


def set_mu_cov(template, mu, cov):
    row = dict(template)
    for i in range(6):
        row[f"relative_mu_{i}"] = format(mu[i], ".17g")
        for j in range(6):
            row[f"covariance_{i}_{j}"] = format(cov[i, j], ".17g")
    return row


def validate(mu, cov, key, update):
    if not np.all(np.isfinite(mu)) or not np.all(np.isfinite(cov)):
        raise ValueError(f"nonfinite belief {key} update {update}")
    if np.max(np.abs(cov - cov.T)) > 1e-9:
        raise ValueError(f"nonsymmetric covariance {key} update {update}")
    if np.linalg.eigvalsh((cov + cov.T) / 2)[0] <= 0:
        raise ValueError(f"nonpositive covariance {key} update {update}")


def parse_log(path):
    versions = defaultdict(list)
    retired = {}
    errors = []
    for line in path.open(errors="replace"):
        if (match := EDGE.search(line)) is not None:
            v = match.group(1).split(",")
            if len(v) != 64:
                raise ValueError(f"edge row has {len(v)} fields")
            update, source, i, j, ti, tj, version, status = v[:8]
            if source != "k":
                raise ValueError("wrong source")
            key = (int(i), int(j))
            mean = np.array(list(map(float, v[22:28])))
            covariance = np.array(list(map(float, v[28:64]))).reshape((6, 6))
            validate(mean, covariance, key, update)
            from_pose, to_pose = parse_pose(v[8:15]), parse_pose(v[15:22])
            same_snapshot = log_se3(inverse(from_pose) @ to_pose)
            if np.max(np.abs(mean - same_snapshot)) > 2e-7:
                raise ValueError(f"relative mean and endpoint snapshot differ {key}")
            # Kimera can publish the same stable pair twice at startup before
            # advancing its audit counter. Preserve both snapshots in source
            # order; the receiver-facing revisions are reindexed after the
            # first ROS-recorded payload is located by exact mean/covariance.
            prior_source_revision = (versions[key][-1]["source_revision"]
                                     if versions[key] else -1)
            if int(version) < prior_source_revision or \
                    int(version) > prior_source_revision + 1:
                raise ValueError(f"nonmonotone source revision {key}")
            versions[key].append({"update": int(update), "source_revision": int(version),
                                  "status": status, "ti": float(ti), "tj": float(tj),
                                  "from_pose": from_pose, "to_pose": to_pose,
                                  "mu": mean, "cov": covariance})
        elif (match := RETIRE.search(line)) is not None:
            v = match.group(1).split(",")
            if len(v) != 10:
                raise ValueError(f"retired pose row has {len(v)} fields")
            update, index, stamp = int(v[0]), int(v[1]), float(v[2])
            if index in retired:
                raise ValueError(f"pose retired twice: {index}")
            retired[index] = {"update": update, "time": stamp,
                              "pose": parse_pose(v[3:10])}
        elif (match := ERROR.search(line)) is not None:
            errors.append(match.group(1))
    if errors:
        raise ValueError(f"source same-snapshot covariance errors: {errors[:3]}")
    return versions, retired


def write_csv(path, data, fields):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(data)


def topology(first, represented):
    adjacent = defaultdict(set)
    for row in first:
        i, j = int(row["from_index"]), int(row["to_index"])
        adjacent[i].add(j)
        adjacent[j].add(i)
    components = []
    unvisited = set(represented)
    while unvisited:
        root = min(unvisited)
        unvisited.remove(root)
        queue = deque([root])
        component = []
        while queue:
            node = queue.popleft()
            component.append(node)
            for other in adjacent[node]:
                if other in unvisited:
                    unvisited.remove(other)
                    queue.append(other)
        components.append(sorted(component))
    return sorted(components, key=lambda c: c[0])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--beliefs", required=True, type=Path)
    p.add_argument("--log", required=True, type=Path)
    p.add_argument("--odom-records", required=True, type=Path,
                   help="CSV: source update ID, ROS bag record timestamp ns")
    p.add_argument("--odom-csv", required=True, type=Path)
    p.add_argument("--out", required=True, type=Path)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    published = list(rows(args.beliefs))
    original_fields = list(published[0])
    recorded_updates = {}
    for record in rows(args.odom_records):
        update = int(record["update_frame_id"])
        stamp = int(record["record_timestamp_ns"])
        if update in recorded_updates:
            raise ValueError(f"duplicate recorded update {update}")
        recorded_updates[update] = stamp
    logged, retired = parse_log(args.log)
    smoother_first = {}
    for key, history in logged.items():
        for version in history:
            for index, endpoint in zip(key, ("from_pose", "to_pose")):
                prior = smoother_first.get(index)
                if prior is None or version["update"] < prior[0]:
                    smoother_first[index] = (version["update"], version[endpoint])
    selected, revisions, last, finalized = [], [], [], []
    match_offsets = []
    covariance_ratios_rotation, covariance_ratios_translation = [], []
    first_latency_sec, last_latency_sec = [], []
    relative_first_to_last_translation = []
    relative_first_to_last_rotation = []
    relative_first_to_final_translation = []
    relative_first_to_final_rotation = []
    latest_to_final_translation = []
    latest_to_final_rotation = []
    first_endpoint = {}
    last_endpoint = {}
    excluded_tail, missing_source, unmatched_publication = [], [], []
    for original in published:
        key = (int(original["from_index"]), int(original["to_index"]))
        if key[0] not in retired or key[1] not in retired:
            excluded_tail.append(key)
            continue
        if key not in logged:
            missing_source.append(key)
            continue
        first_mu, first_cov = mu_cov(original)
        matches = [n for n, version in enumerate(logged[key])
                   if np.max(np.abs(first_mu - version["mu"])) <= 1e-11
                   and np.max(np.abs(first_cov - version["cov"])) <= 1e-11]
        if not matches:
            unmatched_publication.append(key)
            continue
        bag_arrival = int(original["record_timestamp_ns"])
        matches = [n for n in matches if logged[key][n]["update"] in recorded_updates]
        if not matches:
            raise ValueError(f"published source snapshot has no ROS update time {key}")
        # Repeated identical snapshots are possible. Match publication to the
        # same callback's recorded odometry, not merely to the earliest equal
        # floating-point payload from some prior callback.
        offset = min(matches, key=lambda n: (
            abs(recorded_updates[logged[key][n]["update"]] - bag_arrival), -n))
        if abs(recorded_updates[logged[key][offset]["update"]] - bag_arrival) > 100_000_000:
            raise ValueError(f"published payload/update record gap exceeds 100 ms {key}")
        match_offsets.append(offset)
        version_zero = logged[key][offset]
        if abs(float(original["from_timestamp_sec"]) - version_zero["ti"]) > 2e-6 or \
                abs(float(original["to_timestamp_sec"]) - version_zero["tj"]) > 2e-6:
            raise ValueError(f"physical timestamps changed: {key}")
        first = dict(original)
        first["revision"] = "0"
        selected.append(dict(original))
        revisions.append(first)
        first_endpoint[key] = version_zero["from_pose"], version_zero["to_pose"]
        prev_availability = int(original["record_timestamp_ns"])
        physical_end = int(original["to_timestamp_ns"])
        if prev_availability < physical_end:
            raise ValueError(f"first belief recorded before physical endpoint {key}")
        first_latency_sec.append((prev_availability - physical_end) / 1e9)
        for revision_number, version in enumerate(logged[key][offset + 1:], start=1):
            availability = recorded_updates.get(version["update"])
            if availability is None:
                raise ValueError(f"no recorded ROS availability for update {version['update']}")
            availability = max(availability, prev_availability)
            if availability < physical_end:
                raise ValueError(f"revision available before physical endpoint {key}")
            prev_availability = availability
            new_row = set_mu_cov(original, version["mu"], version["cov"])
            new_row["revision"] = str(revision_number)
            new_row["record_timestamp_ns"] = str(availability)
            revisions.append(new_row)
        last_version = logged[key][-1]
        last_endpoint[key] = last_version["from_pose"], last_version["to_pose"]
        latest_row = set_mu_cov(original, last_version["mu"], last_version["cov"])
        latest_row["revision"] = str(len(logged[key]) - offset - 1)
        latest_row["record_timestamp_ns"] = str(prev_availability)
        last.append(latest_row)
        last_latency_sec.append((prev_availability - physical_end) / 1e9)
        final_mean = log_se3(inverse(retired[key[0]]["pose"]) @
                             retired[key[1]]["pose"])
        final = set_mu_cov(latest_row, final_mean, last_version["cov"])
        finalized.append(final)
        for target, translations, rotations in (
                (last_version["mu"], relative_first_to_last_translation,
                 relative_first_to_last_rotation),
                (final_mean, relative_first_to_final_translation,
                 relative_first_to_final_rotation)):
            delta = log_se3(inverse(exp_se3(first_mu)) @ exp_se3(target))
            translations.append(float(np.linalg.norm(delta[3:])))
            rotations.append(float(np.linalg.norm(delta[:3]) * 180 / math.pi))
        delta = log_se3(inverse(exp_se3(last_version["mu"])) @ exp_se3(final_mean))
        latest_to_final_translation.append(float(np.linalg.norm(delta[3:])))
        latest_to_final_rotation.append(float(np.linalg.norm(delta[:3]) * 180 / math.pi))
        covariance_ratios_rotation.append(float(np.trace(last_version["cov"][:3, :3]) /
                                                 np.trace(first_cov[:3, :3])))
        covariance_ratios_translation.append(float(np.trace(last_version["cov"][3:, 3:]) /
                                                    np.trace(first_cov[3:, 3:])))
    if missing_source or unmatched_publication:
        raise ValueError(f"unmatched source belief: absent={missing_source[:5]} "
                         f"payload={unmatched_publication[:5]}")
    if not selected:
        raise ValueError("no fully retired published beliefs")
    if len({(r["from_index"], r["to_index"]) for r in selected}) != len(selected):
        raise ValueError("duplicate published BeliefID")
    first_ids = {(int(r["from_index"]), int(r["to_index"])) for r in selected}
    represented = set(index for pair in first_ids for index in pair)
    components = topology(selected, represented)
    # A source pose may have been retired without belonging to any published edge.
    unrepresented = sorted(set(retired) - represented)
    all_revisions = sorted(revisions, key=lambda r: (
        int(r["record_timestamp_ns"]), int(r["ordinal"]), int(r["revision"])))
    write_csv(args.out / "first.csv", selected, original_fields)
    write_csv(args.out / "revisions.csv", all_revisions, original_fields + ["revision"])
    write_csv(args.out / "last_revision.csv", last, original_fields + ["revision"])
    write_csv(args.out / "finalized.csv", finalized, original_fields + ["revision"])
    # Source trajectory products retain their own 0.2 s sampling; the online
    # trajectory is the ROS odometry actually used for the prior native ATE.
    source_online = {int(row["field.header.seq"]): row for row in rows(args.odom_csv)}
    common_poses = sorted(retired)
    if common_poses != list(range(len(common_poses))):
        raise ValueError("retired source indices are not contiguous from zero")
    if any(index not in source_online for index in common_poses):
        raise ValueError("missing native trajectory pose")
    times = [int(source_online[index]["field.header.stamp"]) for index in common_poses]
    online_poses = [parse_pose([
        source_online[index][f"field.pose.pose.position.{axis}"] for axis in "xyz"] +
        [source_online[index][f"field.pose.pose.orientation.w"]] +
        [source_online[index][f"field.pose.pose.orientation.{axis}"]
         for axis in "xyz"]) for index in common_poses]
    matured_poses = [retired[index]["pose"] for index in common_poses]
    write_tum(args.out / "native_online.tum", times, online_poses)
    write_tum(args.out / "native_matured.tum", times, matured_poses)
    smoother_first_missing = sorted(set(common_poses) - set(smoother_first))
    if not smoother_first_missing:
        first_smoother_poses = [smoother_first[index][1] for index in common_poses]
        write_tum(args.out / "native_smoother_first.tum", times, first_smoother_poses)
    shared_start, shared_end = [], []
    by_start = defaultdict(list)
    for row in selected:
        by_start[int(row["from_index"])].append(row)
    for row in selected:
        key = (int(row["from_index"]), int(row["to_index"]))
        for following in by_start[key[1]]:
            next_key = (int(following["from_index"]), int(following["to_index"]))
            a = first_endpoint[key][1]
            b = first_endpoint[next_key][0]
            delta = log_se3(inverse(a) @ b)
            shared_start.append(float(np.linalg.norm(delta[3:])))
            shared_end.append(float(np.linalg.norm(delta[:3]) * 180 / math.pi))
    summary = {
        "published_total": len(published), "fully_retired_beliefs": len(selected),
        "right_censored_beliefs": len(excluded_tail),
        "retired_source_poses": len(retired), "represented_retired_poses": len(represented),
        "unrepresented_retired_pose_indices": unrepresented,
        "components": [{"root": c[0], "end": c[-1], "poses": len(c),
                        "edges": sum(int(r["from_index"]) in c for r in selected)}
                       for c in components],
        "first_record_matches_logged_revision": numeric_summary(match_offsets),
        "first_publication_latency_from_end_sec": numeric_summary(first_latency_sec),
        "last_valid_revision_latency_from_end_sec": numeric_summary(last_latency_sec),
        "revision_messages": len(revisions),
        "revisions_per_id": numeric_summary([
            len(logged[(int(r["from_index"]), int(r["to_index"]))]) -
            match_offsets[n] for n, r in enumerate(selected)]),
        "first_vs_last_translation_m": numeric_summary(relative_first_to_last_translation),
        "first_vs_last_rotation_deg": numeric_summary(relative_first_to_last_rotation),
        "first_vs_final_translation_m": numeric_summary(relative_first_to_final_translation),
        "first_vs_final_rotation_deg": numeric_summary(relative_first_to_final_rotation),
        "last_vs_final_translation_m": numeric_summary(latest_to_final_translation),
        "last_vs_final_rotation_deg": numeric_summary(latest_to_final_rotation),
        "shared_endpoint_first_publication_translation_m": numeric_summary(shared_start),
        "shared_endpoint_first_publication_rotation_deg": numeric_summary(shared_end),
        "last_over_first_rotation_covariance_trace_ratio":
            numeric_summary(covariance_ratios_rotation),
        "last_over_first_translation_covariance_trace_ratio":
            numeric_summary(covariance_ratios_translation),
        "source_log_covariance_errors": 0,
        "native_online_product": "recorded /kimera_vio_ros/odometry, normally incremental-pose output",
        "native_matured_product": "last active smoother pose immediately before retirement",
        "native_smoother_first_product": "first logged smoother pose per retired index, if all indices observed",
        "native_smoother_first_missing_indices": smoother_first_missing,
        "availability_basis": "first: recorded belief; revisions: recorded ROS odometry of same smoother update (conservative max with previous arrival)",
    }
    (args.out / "export_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
