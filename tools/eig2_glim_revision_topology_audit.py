#!/usr/bin/env python3
"""Offline, source-only EIG-2 endpoint revision/topology audit.

Consumes the flag-gated GLIM diagnostic log. It does not load or change an
estimator. T1 and T2 are forests and are reconstructed by exact SE(3) products;
T3 edge lists are exported for the separate isolated pose-only graph tool.
"""

import argparse
import csv
import json
import math
import re
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from eig2_single_source_preservation import exp_se3, inverse, log_se3, pose, read_tum, stats, write_tum


PATTERN = re.compile(r"GLIM_CBS_(ACTIVE_POSE|EXPORT_ENDPOINT|EXPORT_BELIEF)_ROW,([^\x1b\r\n]+)")


def decode_pose(values):
    return pose(np.array([float(x) for x in values[:3]]), [float(x) for x in values[3:7]])


def tangent_token(text):
    return np.array([float(x) for x in text.split(";")])


def covariance_token(text):
    matrix = np.array([[float(x) for x in row.split(":")]
                       for row in text.split(";")])
    if matrix.shape != (6, 6) or not np.all(np.isfinite(matrix)):
        raise ValueError("invalid full 6x6 belief covariance token")
    if np.max(np.abs(matrix-matrix.T)) > 1e-10:
        raise ValueError("nonsymmetric source covariance token")
    return matrix


def parse_log(path):
    active = defaultdict(list)
    endpoint = defaultdict(list)
    published = defaultdict(list)
    for line in path.open(errors="replace"):
        match = PATTERN.search(line)
        if not match:
            continue
        kind, payload = match.groups()
        parts = payload.split(",")
        if kind == "ACTIVE_POSE":
            if len(parts) != 12:
                raise ValueError((kind, len(parts), payload))
            source, update, record, index, stamp = parts[:5]
            active[int(index)].append({"source": source, "update": int(update),
                                       "record": float(record), "stamp": float(stamp),
                                       "pose": decode_pose(parts[5:12])})
        elif kind == "EXPORT_ENDPOINT":
            if len(parts) != 22:
                raise ValueError((kind, len(parts), payload))
            source, update, current_stamp, i, j, ti, tj = parts[:7]
            row = {"source": source, "update": int(update),
                   "current_stamp": float(current_stamp), "i": int(i), "j": int(j),
                   "ti": float(ti), "tj": float(tj),
                   "from": decode_pose(parts[7:14]), "to": decode_pose(parts[14:21]),
                   "mu": tangent_token(parts[21])}
            endpoint[(int(i), int(j))].append(row)
        else:
            if len(parts) != 10:
                raise ValueError((kind, len(parts), payload))
            source, i, j, zero_revision, record, current, ti, tj, mu, covariance = parts
            published[(int(i), int(j))].append({"source": source, "i": int(i),
                "j": int(j), "record": float(record), "current": float(current),
                "ti": float(ti), "tj": float(tj), "mu": tangent_token(mu),
                "covariance": covariance})
    return active, endpoint, published


def relative(a, b):
    return inverse(a) @ b


def error_norms(a, b):
    delta = log_se3(relative(a, b))
    return float(np.linalg.norm(delta[3:])), float(np.linalg.norm(delta[:3]) * 180 / math.pi)


def write_rows(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def components(n, edges):
    adjacent = defaultdict(list)
    for i, j, _ in edges:
        adjacent[i].append(j)
        adjacent[j].append(i)
    result = []
    unseen = set(range(n))
    while unseen:
        root = min(unseen)
        queue = deque([root])
        unseen.remove(root)
        component = []
        while queue:
            current = queue.popleft()
            component.append(current)
            for neighbor in adjacent[current]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    queue.append(neighbor)
        result.append(sorted(component))
    return result


def bridge_count(n, edges):
    adjacent = [[] for _ in range(n)]
    for edge_id, (i, j, _) in enumerate(edges):
        adjacent[i].append((j, edge_id))
        adjacent[j].append((i, edge_id))
    entered = [-1] * n
    low = [0] * n
    counter = 0
    bridges = []

    def visit(u, parent_edge):
        nonlocal counter
        entered[u] = low[u] = counter
        counter += 1
        for v, edge_id in adjacent[u]:
            if edge_id == parent_edge:
                continue
            if entered[v] < 0:
                visit(v, edge_id)
                low[u] = min(low[u], low[v])
                if low[v] > entered[u]:
                    bridges.append(edges[edge_id][:2])
            else:
                low[u] = min(low[u], entered[v])
    for u in range(n):
        if entered[u] < 0:
            visit(u, -1)
    return bridges


def reconstruct_forest(n, edges, roots):
    result = [None] * n
    for index, transform in roots.items():
        result[index] = transform
    outgoing = defaultdict(list)
    for i, j, transform in edges:
        outgoing[i].append((j, transform))
    for index in range(n):
        if result[index] is None:
            raise ValueError(f"unreached pose {index}")
        for to, transform in outgoing[index]:
            if result[to] is not None:
                raise ValueError(f"cycle at {to}")
            result[to] = result[index] @ transform
    return result


def write_edge_csv(path, edges, time_sec):
    rows = []
    for i, j, transform in edges:
        tangent = log_se3(transform)
        rows.append({"from_index": i, "to_index": j,
                     "from_stamp_sec": f"{time_sec[i]:.9f}",
                     "to_stamp_sec": f"{time_sec[j]:.9f}",
                     **{f"mu_{k}": f"{x:.17g}" for k, x in enumerate(tangent)}})
    write_rows(path, rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--beliefs", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    active, endpoint, published = parse_log(args.log)
    native_times, native_poses = read_tum(args.native)
    n_full = len(native_poses)
    if sorted(active) != list(range(n_full)):
        raise ValueError("active-pose audit and native TUM index maps differ")
    latest_update = max(row["update"] for rows in active.values() for row in rows)
    first_unretired = next((i for i in range(n_full)
                            if active[i][-1]["update"] >= latest_update), n_full)
    n = first_unretired
    if n < 100:
        raise ValueError(f"too few matured poses: {n}")
    time_sec = [active[i][0]["stamp"] for i in range(n)]
    if any(abs(time_sec[i] - native_times[i] * 1e-9) > 1e-5 for i in range(n)):
        raise ValueError("active/native physical timestamp mismatch")
    first = [active[i][0]["pose"] for i in range(n)]
    final = [active[i][-1]["pose"] for i in range(n)]
    native = native_poses[:n]
    write_tum(args.out / "source_online_first.tum", native_times[:n], first)
    write_tum(args.out / "source_last_active.tum", native_times[:n], final)
    write_tum(args.out / "source_shutdown.tum", native_times[:n], native)

    pair_first = {}
    pair_final = {}
    pair_revision_last = {}
    revisions = []
    revision_counts = []
    published_rows = []
    for key, history in sorted(endpoint.items()):
        i, j = key
        if j >= n or j != i + 2:
            continue
        if key not in published:
            raise ValueError(f"missing publication metadata for {key}")
        history.sort(key=lambda row: row["update"])
        pub = history[0]
        meta = sorted(published[key], key=lambda row: row["record"])[0]
        covariance_token(meta["covariance"])
        active_i_at_pub = next((row["pose"] for row in active[i]
                                if row["update"] == pub["update"]), None)
        active_j_at_pub = next((row["pose"] for row in active[j]
                                if row["update"] == pub["update"]), None)
        if active_i_at_pub is None or active_j_at_pub is None:
            raise ValueError(f"publication endpoints absent from active snapshot: {key}")
        if max(np.max(np.abs(log_se3(relative(pub["from"], active_i_at_pub)))),
               np.max(np.abs(log_se3(relative(pub["to"], active_j_at_pub))))) > 1e-10:
            raise ValueError(f"publication/active endpoint mismatch: {key}")
        pair_first[key] = relative(pub["from"], pub["to"])
        pair_final[key] = relative(final[i], final[j])
        if np.max(np.abs(log_se3(relative(pair_first[key], exp_se3(pub["mu"]))))) > 1e-10:
            raise ValueError(f"publication endpoint/mu mismatch at {key}")
        if np.max(np.abs(log_se3(relative(pair_first[key], exp_se3(meta["mu"]))))) > 1e-10:
            raise ValueError(f"publication log/mu mismatch at {key}")
        t_err, r_err = error_norms(pair_first[key], pair_final[key])
        from_t_err, from_r_err = error_norms(pub["from"], final[i])
        to_t_err, to_r_err = error_norms(pub["to"], final[j])
        last_published = relative(history[-1]["from"], history[-1]["to"])
        c_t_err, c_r_err = error_norms(last_published, pair_final[key])
        published_rows.append({"i": i, "j": j, "publication_update": pub["update"],
            "publication_record_sec": meta["record"], "endpoint_age_i_sec": meta["record"]-pub["ti"],
            "endpoint_age_j_sec": meta["record"]-pub["tj"],
            "from_fixed_lag_margin_sec": 5.0-(pub["current_stamp"]-pub["ti"]),
            "to_fixed_lag_margin_sec": 5.0-(pub["current_stamp"]-pub["tj"]),
            "from_time_to_retirement_sec": active[i][-1]["record"]-meta["record"],
            "to_time_to_retirement_sec": active[j][-1]["record"]-meta["record"],
            "updates_after_publication_i": active[i][-1]["update"]-pub["update"],
            "updates_after_publication_j": active[j][-1]["update"]-pub["update"],
            "republication_count": len(published[key]),
            "translation_revision_m": t_err, "rotation_revision_deg": r_err,
            "from_endpoint_revision_translation_m": from_t_err,
            "from_endpoint_revision_rotation_deg": from_r_err,
            "to_endpoint_revision_translation_m": to_t_err,
            "to_endpoint_revision_rotation_deg": to_r_err,
            "last_republication_to_final_translation_m": c_t_err,
            "last_republication_to_final_rotation_deg": c_r_err,
            "covariance_token": meta["covariance"]})
        # Revision-aware diagnostic continues after the sender's outgoing
        # window stops republishing this pair. A retired endpoint keeps its
        # last active estimate while the other endpoint can still revise.
        by_i = {row["update"]: row for row in active[i]}
        by_j = {row["update"]: row for row in active[j]}
        previous = None
        latest_i = None
        latest_j = None
        revision = -1
        last_update = max(active[i][-1]["update"], active[j][-1]["update"])
        for update in range(pub["update"], last_update + 1):
            if update in by_i:
                latest_i = by_i[update]["pose"]
            if update in by_j:
                latest_j = by_j[update]["pose"]
            if latest_i is None or latest_j is None:
                continue
            current = relative(latest_i, latest_j)
            if previous is not None and np.max(np.abs(log_se3(relative(previous, current)))) <= 1e-12:
                continue
            revision += 1
            revisions.append({"source": pub["source"], "i": i, "j": j,
                "revision": revision, "update": update,
                "physical_time_sec": active[update][0]["stamp"],
                **{f"mu_{k}": f"{x:.17g}" for k, x in enumerate(log_se3(current))}})
            previous = current
        if np.max(np.abs(log_se3(relative(previous, pair_final[key])))) > 1e-10:
            raise ValueError(f"final revision did not mature for {key}")
        pair_revision_last[key] = previous
        revision_counts.append(revision + 1)
    if not pair_first:
        raise ValueError("no measured 0.2 s pairs")
    bag_belief_check = None
    if args.beliefs:
        checked = 0
        with args.beliefs.open(newline="") as stream:
            for row in csv.DictReader(stream):
                key = (int(row["from_index"]), int(row["to_index"]))
                if key not in published:
                    raise ValueError(f"bag belief absent from publication log: {key}")
                mu = np.array([float(row[f"relative_mu_{k}"]) for k in range(6)])
                covariance = np.array([[float(row[f"covariance_{r}_{c}"])
                                        for c in range(6)] for r in range(6)])
                logged = sorted(published[key], key=lambda item: item["record"])[0]
                if np.max(np.abs(mu-logged["mu"])) > 1e-12 or \
                   np.max(np.abs(covariance-covariance_token(logged["covariance"]))) > 1e-12:
                    raise ValueError(f"ROS bag/exported payload differs from logged publication: {key}")
                checked += 1
        if checked != len(published):
            raise ValueError(f"bag/exported belief count {checked} != logged {len(published)}")
        bag_belief_check = checked
    write_rows(args.out / "publication_edges.csv", published_rows)
    write_rows(args.out / "revision_aware_edges.csv", revisions)

    consecutive_pub = {}
    consecutive_final = {}
    for j in range(1, n):
        i = j - 1
        snapshots = {row["update"]: row["pose"] for row in active[i]}
        if j not in snapshots:
            raise ValueError(f"consecutive start {i} absent at update {j}")
        consecutive_pub[(i, j)] = relative(snapshots[j], first[j])
        consecutive_final[(i, j)] = relative(final[i], final[j])

    shared_t1 = []
    shared_t2 = []
    for j in range(2, n-2):
        arrival_pose = endpoint[(j-2,j)][0]["to"]
        next_edge_pose = endpoint[(j,j+2)][0]["from"]
        shared_t1.append(error_norms(arrival_pose, next_edge_pose))
    for j in range(1, n-1):
        later = next(row["pose"] for row in active[j] if row["update"] == j+1)
        shared_t2.append(error_norms(first[j], later))

    t1_edges = sorted(pair_first)
    if t1_edges != [(i, i+2) for i in range(n-2)]:
        raise ValueError("0.2 s source map is not complete i->i+2")
    bridge_indices = sorted(set(range(0, n-1, 50)) | {n-2})
    revision_last_edges = [(i, j, pair_revision_last[(i,j)]) for i,j in t1_edges]
    write_edge_csv(args.out / "t1_revision_last_edges.csv", revision_last_edges, time_sec)
    revision_reconstruction = reconstruct_forest(n, revision_last_edges,
                                                  {0: native[0], 1: native[1]})
    for parity in (0, 1):
        indices = list(range(parity, n, 2))
        write_tum(args.out / f"t1_revision_last_component_{parity}.tum",
                  [native_times[i] for i in indices],
                  [revision_reconstruction[i] for i in indices])
    topologies = {}
    for name, pub_map, fin_map in [
        ("t1", pair_first, pair_final),
        ("t2", consecutive_pub, consecutive_final),
        ("t3", {**pair_first, **{(i,i+1): consecutive_pub[(i,i+1)] for i in bridge_indices}},
         {**pair_final, **{(i,i+1): consecutive_final[(i,i+1)] for i in bridge_indices}}),
    ]:
        for version, mapping in [("publication", pub_map), ("final", fin_map)]:
            edges = [(i, j, transform) for (i,j), transform in sorted(mapping.items())]
            write_edge_csv(args.out / f"{name}_{version}_edges.csv", edges, time_sec)
            comps = components(n, edges)
            bridges = bridge_count(n, edges)
            topologies[f"{name}_{version}"] = {"components": len(comps),
                "component_sizes": [len(c) for c in comps], "edges": len(edges),
                "single_edge_removals_that_increase_components": len(bridges),
                "example_critical_edges": bridges[:5]}
            total_t = []
            total_r = []
            for i, j, transform in edges:
                dt, dr = error_norms(transform, relative(final[i], final[j]))
                total_t.append(dt)
                total_r.append(dr)
            topologies[f"{name}_{version}"]["accumulated_edge_translation_error_m"] = float(sum(total_t))
            topologies[f"{name}_{version}"]["accumulated_edge_rotation_error_deg"] = float(sum(total_r))
            if version == "publication":
                worst = max(range(len(edges)), key=lambda k: total_t[k])
                topologies[f"{name}_{version}"]["worst_translation_edge"] = [
                    edges[worst][0], edges[worst][1], total_t[worst]]
            if name == "t3":
                continue
            roots = {component[0]: native[component[0]] for component in comps}
            reconstruction = reconstruct_forest(n, edges, roots)
            if len(comps) == 1:
                write_tum(args.out / f"{name}_{version}.tum", native_times[:n], reconstruction)
            else:
                for component_index, component in enumerate(comps):
                    write_tum(args.out / f"{name}_{version}_component_{component_index}.tum",
                              [native_times[i] for i in component],
                              [reconstruction[i] for i in component])
            topologies[f"{name}_{version}"]["rms_to_shutdown_m"] = float(np.sqrt(np.mean([
                np.linalg.norm(reconstruction[i][:3,3]-native[i][:3,3])**2 for i in range(n)])))
            separation = [float(np.linalg.norm(reconstruction[i][:3,3]-native[i][:3,3]))
                          for i in range(n)]
            topologies[f"{name}_{version}"]["max_to_shutdown_m"] = max(separation)
            topologies[f"{name}_{version}"]["terminal_to_shutdown_m"] = separation[-1]
            topologies[f"{name}_{version}"]["rms_to_causal_first_m"] = float(np.sqrt(np.mean([
                np.linalg.norm(reconstruction[i][:3,3]-first[i][:3,3])**2 for i in range(n)])))
    revision_t = [row["translation_revision_m"] for row in published_rows]
    revision_r = [row["rotation_revision_deg"] for row in published_rows]
    c_revision_t = [row["last_republication_to_final_translation_m"] for row in published_rows]
    c_revision_r = [row["last_republication_to_final_rotation_deg"] for row in published_rows]
    pose_t = [error_norms(first[i], final[i])[0] for i in range(n)]
    pose_r = [error_norms(first[i], final[i])[1] for i in range(n)]
    final_to_shutdown_t = [error_norms(final[i], native[i])[0] for i in range(n)]
    summary = {"run_pose_count": n_full, "matured_pose_count": n,
        "latest_update": latest_update, "pair_count": len(pair_first),
        "bag_export_payload_matches": bag_belief_check,
        "bridge_indices": bridge_indices, "topologies": topologies,
        "pose_first_to_final_translation_m": stats(pose_t),
        "pose_first_to_final_rotation_deg": stats(pose_r),
        "edge_publication_to_final_translation_m": stats(revision_t),
        "edge_publication_to_final_rotation_deg": stats(revision_r),
        "from_endpoint_publication_to_final_translation_m": stats([row["from_endpoint_revision_translation_m"] for row in published_rows]),
        "from_endpoint_publication_to_final_rotation_deg": stats([row["from_endpoint_revision_rotation_deg"] for row in published_rows]),
        "to_endpoint_publication_to_final_translation_m": stats([row["to_endpoint_revision_translation_m"] for row in published_rows]),
        "to_endpoint_publication_to_final_rotation_deg": stats([row["to_endpoint_revision_rotation_deg"] for row in published_rows]),
        "edges_revised_over_1um": sum(value > 1e-6 for value in revision_t),
        "poses_revised_over_1um": sum(value > 1e-6 for value in pose_t),
        "edge_last_republication_to_final_translation_m": stats(c_revision_t),
        "edge_last_republication_to_final_rotation_deg": stats(c_revision_r),
        "last_active_to_shutdown_translation_m": stats(final_to_shutdown_t),
        "accumulated_edge_revision_translation_m": float(sum(revision_t)),
        "accumulated_edge_revision_rotation_deg": float(sum(revision_r)),
        "republication_count": stats([row["republication_count"] for row in published_rows]),
        "revision_aware_rows": len(revisions),
        "revision_count_per_edge": stats(revision_counts),
        "shared_endpoint_non_telescoping_t1_translation_m": stats([x[0] for x in shared_t1]),
        "shared_endpoint_non_telescoping_t1_rotation_deg": stats([x[1] for x in shared_t1]),
        "shared_endpoint_non_telescoping_t2_translation_m": stats([x[0] for x in shared_t2]),
        "shared_endpoint_non_telescoping_t2_rotation_deg": stats([x[1] for x in shared_t2]),
    }
    for predictor in ("endpoint_age_i_sec", "endpoint_age_j_sec",
                      "from_fixed_lag_margin_sec", "to_fixed_lag_margin_sec",
                      "from_time_to_retirement_sec", "to_time_to_retirement_sec",
                      "updates_after_publication_i", "updates_after_publication_j"):
        x = np.array([row[predictor] for row in published_rows], dtype=float)
        y = np.array(revision_t)
        y_rotation = np.array(revision_r)
        edges = np.unique(np.quantile(x, [0, .25, .5, .75, 1]))
        groups = []
        for a, b in zip(edges[:-1], edges[1:]):
            mask = (x >= a) & (x <= b if b == edges[-1] else x < b)
            groups.append({"min": float(a), "max": float(b),
                           "translation_m": stats(y[mask]),
                           "rotation_deg": stats(y_rotation[mask])})
        summary.setdefault("revision_translation_vs", {})[predictor] = {
            "pearson_r": float(np.corrcoef(x, y)[0,1]) if np.std(x) > 0 else None,
            "rotation_pearson_r": float(np.corrcoef(x, y_rotation)[0,1]) if np.std(x) > 0 else None,
            "quartile_bins": groups}
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
