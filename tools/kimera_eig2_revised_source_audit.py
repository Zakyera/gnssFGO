#!/usr/bin/env python3
"""Read-only audit of a few retained EIG-2 Kimera belief revisions."""
import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp

from eig2_single_source_preservation import exp_se3, inverse, log_se3, pose, read_online_csv


def rows(path):
    with path.open(newline="") as stream:
        yield from csv.DictReader(stream)


def write(path, values):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(values[0]))
        writer.writeheader()
        writer.writerows(values)


def decode_pose(values):
    v = np.array(values, dtype=float)
    return pose(v[:3], [v[4], v[5], v[6], v[3]])


def reference_pose(reference, stamp):
    times = reference["times"]
    index = int(np.searchsorted(times, stamp))
    if index == 0 or index == len(times) or times[index]-times[index-1] > 0.05:
        raise ValueError(f"missing IE-TC coverage at {stamp}")
    left, right = times[index-1:index+1]
    alpha = (stamp-left)/(right-left)
    position = (1-alpha)*reference["positions"][index-1] + alpha*reference["positions"][index]
    orientation = Slerp([left, right], Rotation.from_quat(reference["quaternions"][index-1:index+1]))([stamp]).as_quat()[0]
    return pose(position, orientation)


def choose_edges(groups, reference, t0):
    scored = []
    for edge, versions in groups.items():
        last = versions[-1]
        ti, tj = float(last["from_timestamp_sec"]), float(last["to_timestamp_sec"])
        if not (0 <= tj-t0 <= 235):
            continue
        reference_edge = inverse(reference_pose(reference, ti)) @ reference_pose(reference, tj)
        native = exp_se3([float(versions[0][f"relative_mu_{n}"]) for n in range(6)])
        revised = exp_se3([float(last[f"relative_mu_{n}"]) for n in range(6)])
        error0 = np.linalg.norm(native[:3, 3]-reference_edge[:3, 3])
        error1 = np.linalg.norm(revised[:3, 3]-reference_edge[:3, 3])
        scored.append(dict(edge=edge, elapsed=tj-t0, error0=error0,
                           error1=error1, deterioration=error1-error0))
    late = [r for r in scored if 215 <= r["elapsed"] <= 235]
    startup = [r for r in scored if 0 <= r["elapsed"] <= 20]
    worst = sorted(late, key=lambda r: r["deterioration"], reverse=True)[:4]
    # Prefer a control near the bad cluster, not merely anywhere in 25 s.
    nearby = [r for r in late if r not in worst and abs(r["elapsed"]-worst[0]["elapsed"]) <= 3.]
    stable = min(nearby, key=lambda r: abs(r["deterioration"]))
    improving = min((r for r in late if r not in worst and r is not stable),
                    key=lambda r: r["deterioration"])
    improving_startup = min(startup, key=lambda r: r["deterioration"])
    selected = [dict(**r, category=label) for label, items in
                (("worst", worst), ("stable", [stable]), ("improving", [improving]),
                 ("startup_improving", [improving_startup])) for r in items]
    if len({r["edge"] for r in selected}) != len(selected):
        raise ValueError("selection contains duplicate edges")
    return selected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kimera-run", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    reference = np.load(args.reference)
    online = read_online_csv(args.kimera_run/"kimera_odometry.csv")
    t0 = online[0][0]*1e-9
    groups = defaultdict(list)
    for row in rows(args.kimera_run/"revision_products/revisions.csv"):
        to = float(row["to_timestamp_sec"])
        if to-t0 <= 235+1e-5:
            groups[(int(row["from_index"]), int(row["to_index"]))].append(row)
    for group in groups.values():
        group.sort(key=lambda r: int(r["record_timestamp_ns"]))
    selected = choose_edges(groups, reference, t0)
    identifiers = {tuple(r["edge"]) for r in selected}
    selected_metadata = {tuple(r["edge"]): r for r in selected}

    log_snapshots = defaultdict(list)
    timing = {}
    revision_pattern = re.compile(r"KIMERA_CBS_REVISION_EDGE_ROW,([^\x1b\r\n]+)")
    optimize_pattern = re.compile(r"KIMERA_OPTIMIZE_TIMING_ROW,([^\x1b\r\n]+)")
    for line in (args.kimera_run/"estimators.log").open(errors="replace"):
        match = revision_pattern.search(line)
        if match:
            fields = match[1].split(",")
            key = (int(fields[2]), int(fields[3]))
            if key in identifiers:
                log_snapshots[key].append(fields)
        else:
            match = optimize_pattern.search(line)
            if match:
                fields = match[1].split(",")
                update = int(fields[0])
                if 1065 <= update <= 1220 or update <= 100:
                    timing[update] = fields

    results = []
    covariance_max_element_error = 0.
    mean_max_translation_error = 0.
    mean_max_rotation_error = 0.
    covariance_max_symmetry_error = 0.
    covariance_min_eigenvalue = np.inf
    compared = 0
    selected_summary = []
    for edge in identifiers:
        versions, snapshots = groups[edge], sorted(log_snapshots[edge], key=lambda f: int(f[6]))
        if len(versions) != len(snapshots):
            raise ValueError(f"incomplete logged source revisions for {edge}")
        if edge[1] != edge[0]+1:
            raise ValueError(f"nonconsecutive selected edge {edge}")
        if [int(f[6]) for f in snapshots] != list(range(len(versions))):
            raise ValueError(f"nonmonotonic or reused source revision for {edge}")
        reference_edge = inverse(reference_pose(reference, float(versions[0]["from_timestamp_sec"]))) @ reference_pose(
            reference, float(versions[0]["to_timestamp_sec"]))
        per_edge = []
        for revision, (csv_row, fields) in enumerate(zip(versions, snapshots)):
            update = int(fields[0])
            if (fields[1] != "k" or int(fields[2]) != edge[0] or
                    int(fields[3]) != edge[1] or int(fields[6]) != revision or
                    update != edge[1]+revision):
                raise ValueError(f"key/index/update mismatch for {edge} revision {revision}")
            if fields[7] != ("first" if revision == 0 else "active"):
                raise ValueError(f"first/active status mismatch for {edge} revision {revision}")
            if (abs(float(fields[4])-float(csv_row["from_timestamp_sec"])) > 1e-6 or
                    abs(float(fields[5])-float(csv_row["to_timestamp_sec"])) > 1e-6):
                raise ValueError(f"physical timestamp mismatch for {edge}")
            first_pose, second_pose = decode_pose(fields[8:15]), decode_pose(fields[15:22])
            logged_mu = np.array(fields[22:28], dtype=float)
            exported_mu = np.array([float(csv_row[f"relative_mu_{n}"]) for n in range(6)])
            if np.max(np.abs(logged_mu-exported_mu)) > 1e-12:
                raise ValueError(f"CSV/log mean mismatch for {edge} revision {revision}")
            logged_cov = np.array(fields[28:64], dtype=float).reshape(6, 6)
            exported_cov = np.array([[float(csv_row[f"covariance_{i}_{j}"]) for j in range(6)] for i in range(6)])
            covariance_max_element_error = max(covariance_max_element_error,
                                               float(np.max(np.abs(logged_cov-exported_cov))))
            if covariance_max_element_error > 1e-12:
                raise ValueError(f"CSV/log covariance mismatch for {edge} revision {revision}")
            covariance_max_symmetry_error = max(covariance_max_symmetry_error,
                                                float(np.max(np.abs(logged_cov-logged_cov.T))))
            eigenvalue = float(np.linalg.eigvalsh((logged_cov+logged_cov.T)/2)[0])
            covariance_min_eigenvalue = min(covariance_min_eigenvalue, eigenvalue)
            if not np.all(np.isfinite(logged_cov)) or covariance_max_symmetry_error > 1e-9 or eigenvalue < -1e-9:
                raise ValueError(f"invalid logged covariance for {edge} revision {revision}")
            z = exp_se3(logged_mu)
            recomputed = inverse(first_pose) @ second_pose
            difference = log_se3(inverse(z) @ recomputed)
            mean_max_rotation_error = max(mean_max_rotation_error, float(np.linalg.norm(difference[:3])))
            mean_max_translation_error = max(mean_max_translation_error, float(np.linalg.norm(difference[3:])))
            if mean_max_translation_error > 1e-8 or mean_max_rotation_error > 1e-8:
                raise ValueError(f"BELIEF EXTRACTION FAILED at {edge} revision {revision}")
            error_vector = z[:3, 3]-reference_edge[:3, 3]
            rotation_error = Rotation.from_matrix(reference_edge[:3, :3].T@z[:3, :3]).magnitude()
            row = dict(category=selected_metadata[edge]["category"], from_index=edge[0], to_index=edge[1],
                       revision=revision, backend_update=update,
                       availability_ns=int(csv_row["record_timestamp_ns"]),
                       from_timestamp_ns=int(csv_row["from_timestamp_ns"]),
                       to_timestamp_ns=int(csv_row["to_timestamp_ns"]),
                       from_pose_p_qwxyz=json.dumps(list(map(float,fields[8:15]))),
                       to_pose_p_qwxyz=json.dumps(list(map(float,fields[15:22]))),
                       logged_se3_mu_phi_rho=json.dumps(logged_mu.tolist()),
                       logged_covariance_6x6_phi_rho=json.dumps(logged_cov.tolist()),
                       ie_tc_relative_p_qwxyz=json.dumps(list(reference_edge[:3,3]) +
                           list(Rotation.from_matrix(reference_edge[:3,:3]).as_quat()[[3,0,1,2]])),
                       translation_error_x_m=float(error_vector[0]),
                       translation_error_y_m=float(error_vector[1]),
                       translation_error_z_m=float(error_vector[2]),
                       translation_error_norm_m=float(np.linalg.norm(error_vector)),
                       rotation_error_deg=float(np.degrees(rotation_error)),
                       recompute_log_phi_norm_rad=float(np.linalg.norm(difference[:3])),
                       recompute_log_rho_norm_m=float(np.linalg.norm(difference[3:])),
                       covariance_trace_rotation=float(np.trace(logged_cov[:3,:3])),
                       covariance_trace_translation=float(np.trace(logged_cov[3:,3:])))
            results.append(row)
            per_edge.append(row)
            compared += 1
        first, last = per_edge[0], per_edge[-1]
        changes = [r["translation_error_norm_m"]-per_edge[k-1]["translation_error_norm_m"]
                   for k, r in enumerate(per_edge) if k]
        bad_index = 1+int(np.argmax(changes))
        pre, post = per_edge[bad_index-1:bad_index+1]
        selected_summary.append(dict(category=selected_metadata[edge]["category"],
            edge=f"{edge[0]}->{edge[1]}", elapsed_s=selected_metadata[edge]["elapsed"],
            error_first_m=first["translation_error_norm_m"],
            error_last_m=last["translation_error_norm_m"],
            error_delta_m=last["translation_error_norm_m"]-first["translation_error_norm_m"],
            max_one_update_error_increase_m=float(changes[bad_index-1]),
            worst_update=int(post["backend_update"]),
            error_before_worst_m=pre["translation_error_norm_m"],
            error_after_worst_m=post["translation_error_norm_m"],
            positive_step_count=sum(v > 0 for v in changes),
            negative_step_count=sum(v < 0 for v in changes),
            smoother_update_ms=float(timing[int(post["backend_update"])][5]) if int(post["backend_update"]) in timing else None,
            extra_iteration_ms=float(timing[int(post["backend_update"])][7]) if int(post["backend_update"]) in timing else None,
            local_cbs_covariance_graph_factor_count=int(timing[int(post["backend_update"])][15]) if int(post["backend_update"]) in timing else None,
            backend_healthy=int(timing[int(post["backend_update"])][16]) if int(post["backend_update"]) in timing else None))
    results.sort(key=lambda r: (r["from_index"], r["revision"]))
    selected_summary.sort(key=lambda r: (r["category"], -r["error_delta_m"]))
    write(args.out/"selected_revisions_full.csv", results)
    write(args.out/"selected_edge_summary.csv", selected_summary)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for record in selected_summary:
        edge = tuple(map(int, record["edge"].split("->")))
        versions = [r for r in results if (r["from_index"], r["to_index"]) == edge]
        if record["category"] == "startup_improving":
            continue
        label = record["edge"] + " (" + record["category"] + ")"
        axes[0].plot([r["backend_update"] for r in versions],
                     [r["translation_error_norm_m"] for r in versions], label=label)
        axes[1].plot([r["backend_update"] for r in versions],
                     [r["covariance_trace_translation"] for r in versions], label=label)
    for axis in axes:
        axis.axvline(1084, linestyle="--", color="black", alpha=.45)
        axis.axvline(1093, linestyle=":", color="black", alpha=.55)
        axis.grid(alpha=.3)
    axes[0].set_ylabel("IE-TC relative translation error (m)")
    axes[1].set_ylabel("Native translation covariance trace (m²)")
    axes[1].set_xlabel("Kimera backend update ID")
    axes[0].legend(ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out/"selected_revision_evolution.png", dpi=160)
    plt.close(fig)
    summary = dict(t0=t0, selected_edges=len(selected_summary), selected_revisions=compared,
                   max_log_se3_rotation_rad=mean_max_rotation_error,
                   max_log_se3_translation_m=mean_max_translation_error,
                   max_covariance_log_export_element_error=covariance_max_element_error,
                   max_covariance_symmetry_error=covariance_max_symmetry_error,
                   min_covariance_eigenvalue=covariance_min_eigenvalue,
                   selected=selected_summary)
    (args.out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
