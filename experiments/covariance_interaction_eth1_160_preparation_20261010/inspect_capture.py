#!/usr/bin/env python3
"""Read saved telemetry only. No ROS, estimator, solver, network or launch calls.

Inventory the existing logs before choosing more logging. The JSON is a data
readiness check, NOT a covariance calibration or factor-dominance result.
"""
import argparse
import csv
import hashlib
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ANSI = re.compile(r"\x1b\[[0-9;]*m")
MARKERS = (
    "GLIM_CBS_MEAN_ALIGNMENT_ROW", "KIMERA_CBS_MEAN_ALIGNMENT_ROW",
    "CBS_ODOM_FACTOR_MATRIX_ROW", "CBS_RECEIVER_LOCAL_COMPETITION_ROW",
    "CBS_RECEIVER_LOCAL_COMPETITION_STATUS_ROW", "CBS_RECEIVER_LOCAL_FACTOR_GROUP_ROW",
    "CBS_COVARIANCE_EXCHANGE_MATRIX_ROW", "CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW",
    "CBS_COVARIANCE_EXCHANGE_STATUS_ROW", "CBS_COVARIANCE_EXCHANGE_FACTOR_GROUP_ROW",
    "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW", "KIMERA_CBS_POSE_MERGE_ROW",
    "GLIM_CBS_POSE_MERGE_ROW", "GLIM_CBS_EXPORT_BELIEF_ROW",
    "GLIM_CBS_REVISION_BELIEF_ROW", "KIMERA_CBS_REVISION_EDGE_ROW",
)


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def matrix_token(token):
    vals = np.array([float(x) for x in re.split("[;:]", token)], dtype=float)
    if vals.size != 36:
        raise ValueError("expected every one of 36 covariance entries")
    return vals.reshape(6, 6)


def covariance_stats(cov):
    if cov.shape != (6, 6) or not np.isfinite(cov).all():
        raise ValueError("invalid covariance")
    symmetry = float(np.max(np.abs(cov - cov.T)))
    if symmetry > 1e-12:
        raise ValueError("asymmetric covariance")
    # Symmetrization only for numerical inspection, NEVER written to a factor.
    sym = (cov + cov.T) / 2
    eig = np.linalg.eigvalsh(sym)
    if eig[0] <= 0:
        raise ValueError("covariance not positive definite")
    return dict(rotation_trace_rad2=float(np.trace(sym[:3, :3])),
                rho_trace_m2=float(np.trace(sym[3:, 3:])),
                rotation_sqrt_trace_rad=float(np.sqrt(np.trace(sym[:3, :3]))),
                rho_sqrt_trace_m=float(np.sqrt(np.trace(sym[3:, 3:]))),
                symmetry_max=symmetry, minimum_eigenvalue_numeric=float(eig[0]))


def distribution(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return {"count": 0}
    if not np.isfinite(a).all():
        raise ValueError("nonfinite summary")
    return dict(count=int(a.size), minimum=float(a.min()), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), maximum=float(a.max()))


def aggregate(covs):
    rows = [covariance_stats(c) for c in covs]
    return {k: distribution([r[k] for r in rows]) for k in rows[0]} if rows else {}


def journal_inventory(path):
    """Keep revisions distinct; summarize first/last once per stable identity.

    This includes transported-but-unsupported payloads. Accepted factor slots
    must be joined separately from revision_events, never inferred here.
    """
    ids, by_source, counts = {}, defaultdict(list), Counter()
    with path.open() as f:
        for row in csv.reader(f):
            counts[row[0]] += 1
            if row[0] != "B":
                continue
            if len(row) != 52:
                raise ValueError("invalid belief journal width")
            arrival, ordinal, revision, i, j, ti, tj = map(
                int, (row[1], row[4], row[5], row[6], row[7], row[8], row[9]))
            if not ti < tj <= arrival:
                raise ValueError("invalid physical/availability time")
            mean = np.asarray(row[10:16], dtype=float)
            if not np.isfinite(mean).all():
                raise ValueError("nonfinite mean")
            cov = np.asarray(row[16:52], dtype=float).reshape(6, 6)
            covariance_stats(cov)
            key = (row[2], row[3], i, j)
            if key in ids:
                old = ids[key]
                if (ordinal, ti, tj) != old["identity"] or revision != old["revision"] + 1:
                    raise ValueError("revision identity/order changed")
                if arrival < old["last_arrival"]:
                    raise ValueError("per-ID availability regressed")
                old.update(revision=revision, last=cov, last_arrival=arrival)
                old["count"] += 1
            else:
                if revision != 0:
                    raise ValueError("missing first revision")
                ids[key] = dict(identity=(ordinal, ti, tj), revision=revision,
                                first=cov, last=cov, count=1,
                                first_arrival=arrival, last_arrival=arrival)
    for key, value in ids.items():
        by_source[key[0]].append(value)
    summary = {}
    for source, values in sorted(by_source.items()):
        summary[source] = dict(
            transported_ids=len(values), versions=sum(x["count"] for x in values),
            versions_per_id=distribution([x["count"] for x in values]),
            first_publication=aggregate([x["first"] for x in values]),
            last_observed=aggregate([x["last"] for x in values]),
            limitation="last observed may be right-censored; includes unsupported receiver payloads")
    return dict(event_counts=dict(counts), sources=summary)


def inspect(source, receiver):
    if not (source / "complete.marker").is_file() or not (receiver / "complete.marker").is_file():
        raise ValueError("completed source and receiver captures required")
    logs = sorted(p for p in (source / "ros_logs").glob("*/*stdout.log")
                  if p.parent.name != "latest")
    glogs = [p for p in logs if "glim_rosnode" in p.name]
    klogs = [p for p in logs if "kimera_vio" in p.name]
    if len(glogs) != 1 or len(klogs) != 1:
        raise ValueError("ambiguous native node logs")
    roles = {"glim_stdout": glogs[0], "kimera_stdout": klogs[0],
             "kimera_main": source / "estimators.log"}
    inputs = list(roles.values()) + [receiver / "events.csv", receiver / "resolved_config.json"]
    before = {str(p): sha(p) for p in inputs}
    markers, aligned = {}, {}
    for role, path in roles.items():
        counts, covs = Counter(), []
        with path.open() as f:
            for line in f:
                line = ANSI.sub("", line).strip()
                for marker in MARKERS:
                    if marker + "," not in line:
                        continue
                    counts[marker] += 1
                    if marker in ("GLIM_CBS_MEAN_ALIGNMENT_ROW", "KIMERA_CBS_MEAN_ALIGNMENT_ROW"):
                        row = line.split(marker + ",", 1)[1].split(",")
                        if marker.startswith("KIMERA"):
                            row = row[1:]
                        if len(row) != 12:
                            raise ValueError("alignment row schema changed")
                        times = list(map(float, row[4:9]))
                        if not np.isfinite(times).all() or max(times[:4]) > times[4]:
                            raise ValueError("noncausal alignment")
                        covs.append(matrix_token(row[11]))
        markers[role] = dict(counts)
        if covs:
            aligned[role] = dict(rows=len(covs), covariance=aggregate(covs))
    config = json.loads((receiver / "resolved_config.json").read_text())
    journal = journal_inventory(receiver / "events.csv")
    after = {str(p): sha(p) for p in inputs}
    if before != after:
        raise ValueError("input changed while inspecting")
    return dict(
        status="INVENTORY_ONLY_NOT_INTERACTION_VALIDATION", input_sha256=before,
        input_hashes_unchanged=True, source=str(source), receiver=str(receiver),
        marker_counts_by_file=markers, mean_alignment=aligned, transport=journal,
        receiver_model=config["receiver_model"],
        limitations=[
            "No graph Jacobians reconstructed or optimizer run by this tool",
            "No NIS, NEES, physical uncertainty calibration or dominance claim",
            "First/last covariance distributions are transported information, not independently weighted observations",
            "Full 6D eigenvalues are numeric health checks with mixed units, not a source confidence ranking",
            "Missing detailed native diagnostics are NOT interpreted as zero external influence",
        ])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--receiver", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    if a.out.exists():
        raise ValueError("refuse overwrite")
    result = inspect(a.source.resolve(), a.receiver.resolve())
    with a.out.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps(dict(status=result["status"], output=str(a.out),
                          event_counts=result["transport"]["event_counts"])))


if __name__ == "__main__":
    main()
