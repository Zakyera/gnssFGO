#!/usr/bin/env python3
"""Visualize the frozen SPX-3 classic-CBS-ON run without rerunning estimators.

The source runs disabled their live factor-graph inspectors. The graph panels
therefore show the exact logged external-factor endpoint connectivity over the
native pose trajectories, not a recovered complete internal smoother graph.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import re
import subprocess
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from rosbags.highlevel import AnyReader

from grandtour_eth1_gnssfgo_rerun import read_kimera, read_tum, sampled_indices


COLORS = {
    "ground_truth": [45, 125, 255, 255],
    "glim": [20, 205, 90, 255],
    "kimera": [255, 165, 0, 255],
    "gnssfgo": [235, 55, 55, 255],
}
EXTERNAL_COLOR = [225, 65, 235, 255]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_aligned(path: Path, expected_times: np.ndarray) -> np.ndarray:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    times = np.asarray([float(row["timestamp"]) for row in rows])
    xyz = np.asarray(
        [[float(row[f"aligned_{axis}"]) for axis in "xyz"] for row in rows]
    )
    if len(times) != len(expected_times) or np.max(np.abs(times - expected_times)) > 2e-6:
        raise ValueError(f"aligned trajectory grid differs from native input: {path}")
    if not np.isfinite(xyz).all():
        raise ValueError(f"non-finite aligned trajectory: {path}")
    return xyz


def source_logs(run_dir: Path) -> tuple[Path, Path]:
    glim = list((run_dir / "ros_logs").glob("*/glim_rosnode-1-stdout.log"))
    kimera = list(
        (run_dir / "ros_logs").glob("*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log")
    )
    if len(glim) != 1 or len(kimera) != 1:
        raise ValueError("expected one exact-run GLIM and Kimera diagnostic log")
    return glim[0], kimera[0]


def bag_belief_availability(bag_path: Path) -> tuple[dict, dict]:
    glim_in: dict[tuple[int, int], list[float]] = {}
    kimera_out: dict[tuple[int, int], list[float]] = {}
    with AnyReader([bag_path]) as reader:
        for connection, timestamp_ns, raw in reader.messages():
            if connection.topic not in (
                "/kimera/cbs/odom_belief_in",
                "/kimera/cbs/odom_belief_out",
            ):
                continue
            message = reader.deserialize(raw, connection.msgtype)
            target = (
                glim_in
                if connection.topic == "/kimera/cbs/odom_belief_in"
                else kimera_out
            )
            for belief in message.beliefs:
                key = (int(belief.from_pose_index), int(belief.to_pose_index))
                target.setdefault(key, []).append(timestamp_ns * 1e-9)
    return glim_in, kimera_out


def external_events(run_dir: Path, glim_times: np.ndarray, kimera_times: np.ndarray) -> tuple[list[dict], dict]:
    glim_log, kimera_log = source_logs(run_dir)
    glim_bag, kimera_bag = bag_belief_availability(run_dir / "recordings" / "cbs.bag")
    clock: list[tuple[float, float]] = []
    k2g: list[dict] = []
    glim_pattern = re.compile(
        r"\[(\d{10}\.\d+), (\d{10}\.\d+)\]: "
        r"GLIM_CBS_ODOM_INJECT_ROW,k(\d+)->k(\d+),g(\d+)->g(\d+),"
        r".*injected_persistent"
    )
    for line in glim_log.open(encoding="utf-8", errors="replace"):
        match = glim_pattern.search(line)
        if match is None:
            continue
        wall, event_time = float(match[1]), float(match[2])
        k_from, k_to, g_from, g_to = map(int, match.groups()[2:])
        clock.append((wall, event_time))
        key = (k_from, k_to)
        if key not in kimera_bag:
            raise ValueError(f"K2G injection lacks source bag belief {key}")
        if g_to >= len(glim_times):
            raise ValueError(f"K2G target index outside GLIM odometry: {g_to}")
        eligible = [timestamp for timestamp in kimera_bag[key] if timestamp <= event_time + 1e-3]
        if not eligible:
            raise ValueError(f"K2G injection precedes every source publication: {key}")
        k2g.append(
            dict(
                direction="K2G", from_index=g_from, to_index=g_to,
                source_from_index=k_from, source_to_index=k_to,
                event_time=event_time, source_available_time=max(eligible),
                endpoint_time=float(glim_times[g_to]), timing="recorded ROS time",
            )
        )
    if len(k2g) != 662:
        raise ValueError(f"expected 662 applied K2G factors, found {len(k2g)}")
    clock_array = np.asarray(clock)
    if not (np.all(np.diff(clock_array[:, 0]) > 0) and np.all(np.diff(clock_array[:, 1]) > 0)):
        raise ValueError("GLIM wall/sim clock anchors are not monotone")

    transport: list[tuple[int, int, int, int]] = []
    applied: list[tuple[float, int, int]] = []
    transport_pattern = re.compile(
        r"CBS_BPSAM_SOURCE_TRANSPORT_ROW,G2K,gp(\d+),gp(\d+),x(\d+),x(\d+),"
    )
    applied_pattern = re.compile(
        r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\].*"
        r"CBS_ODOM_PREINJECTION_RESIDUAL_ROW,G2K,k,g,x(\d+),x(\d+),"
        r"persistent_odom_applied"
    )
    for line in kimera_log.open(encoding="utf-8", errors="replace"):
        match = transport_pattern.search(line)
        if match is not None:
            transport.append(tuple(map(int, match.groups())))
        match = applied_pattern.search(line)
        if match is not None:
            wall = dt.datetime.fromisoformat(match[1]).replace(tzinfo=dt.timezone.utc).timestamp()
            applied.append((wall, int(match[2]), int(match[3])))
    if len(transport) != 659 or len(applied) != 659:
        raise ValueError(f"expected 659 applied G2K factors; got {len(transport)}, {len(applied)}")
    if [(item[2], item[3]) for item in transport] != [(item[1], item[2]) for item in applied]:
        raise ValueError("G2K source-transport and applied-target order differs")

    # Kimera's diagnostic log records wall time, not ROS simulated time. The
    # contemporaneous GLIM injection log records both clocks. Interpolation
    # recovers a display timestamp; IDs and applied status come from exact logs.
    estimated_times = np.interp(
        [item[0] for item in applied], clock_array[:, 0], clock_array[:, 1]
    )
    g2k: list[dict] = []
    for (g_from, g_to, k_from, k_to), estimated in zip(transport, estimated_times):
        key = (g_from, g_to)
        if key not in glim_bag:
            raise ValueError(f"G2K application lacks source bag belief {key}")
        if k_to >= len(kimera_times):
            raise ValueError(f"G2K target index outside Kimera odometry: {k_to}")
        g2k.append(
            dict(
                direction="G2K", from_index=k_from, to_index=k_to,
                source_from_index=g_from, source_to_index=g_to,
                event_time=float(estimated), source_available_time=max(glim_bag[key]),
                endpoint_time=float(kimera_times[k_to]),
                timing="wall-to-ROS interpolation (<1 ms anchor validation)",
            )
        )
    events = k2g + g2k
    for event in events:
        if event["event_time"] + 1e-3 < max(event["source_available_time"], event["endpoint_time"]):
            raise ValueError(f"external factor precedes availability/endpoint: {event}")
    midpoint = clock_array[::2]
    withheld = clock_array[1::2]
    withheld = withheld[withheld[:, 0] <= midpoint[-1, 0]]
    calibration_error = np.abs(
        np.interp(withheld[:, 0], midpoint[:, 0], midpoint[:, 1]) - withheld[:, 1]
    )
    return events, {
        "K2G_applied": len(k2g), "G2K_applied": len(g2k),
        "Kimera_time_interpolation_heldout_median_s": float(np.median(calibration_error)),
        "Kimera_time_interpolation_heldout_max_s": float(np.max(calibration_error)),
        "source_bag_beliefs_G2K": len(glim_bag),
        "source_bag_beliefs_K2G": len(kimera_bag),
    }


def progressive_path(path: str, times: np.ndarray, xyz: np.ndarray, color: list[int], gap_s: float) -> None:
    current = set(sampled_indices(times, 0.2))
    trail = set(sampled_indices(times, 0.5))
    gaps = np.flatnonzero(np.diff(times) > gap_s) + 1
    for index in sorted(current | trail):
        rr.set_time("time", timestamp=float(times[index]))
        if index in current:
            rr.log(f"{path}/current", rr.Points3D([xyz[index]], colors=[color], radii=0.25))
        if index in trail and index > 0:
            pieces = [segment for segment in np.split(xyz[: index + 1], gaps[gaps <= index]) if len(segment) > 1]
            if pieces:
                rr.log(f"{path}/trail", rr.LineStrips3D(pieces, colors=[color] * len(pieces), radii=rr.Radius.ui_points(2.7)))


def graph_skeleton(path: str, times: np.ndarray, xyz: np.ndarray, color: list[int]) -> None:
    for index in sampled_indices(times, 0.5):
        rr.set_time("time", timestamp=float(times[index]))
        rr.log(f"{path}/native_pose_history", rr.Points3D(xyz[: index + 1], colors=[color], radii=0.045))
        if index > 0:
            rr.log(f"{path}/native_trajectory_skeleton", rr.LineStrips3D([xyz[: index + 1]], colors=[color], radii=rr.Radius.ui_points(1.4)))


def map_points(run_dir: Path, glim_metrics: dict) -> np.ndarray:
    submap = run_dir / "glim_dump" / "000000"
    lines = (submap / "data.txt").read_text(encoding="utf-8").splitlines()
    marker = lines.index("T_world_origin: ")
    transform = np.asarray([[float(v) for v in row.split()] for row in lines[marker + 1 : marker + 5]])
    values = np.fromfile(submap / "points_compact.bin", dtype="<f4")
    if values.size % 3 or transform.shape != (4, 4):
        raise ValueError("invalid saved GLIM submap")
    points = values.reshape(-1, 3).astype(float)
    world = points @ transform[:3, :3].T + transform[:3, 3]
    rotation = np.asarray(glim_metrics["alignment_rotation"], dtype=float)
    translation = np.asarray(glim_metrics["alignment_translation_m"], dtype=float)
    aligned = world @ rotation.T + translation
    if not np.isfinite(aligned).all():
        raise ValueError("non-finite aligned GLIM map")
    return aligned.astype(np.float32)


def blueprint(center: np.ndarray, extent: float) -> rrb.Blueprint:
    def spatial(origin: str, name: str, aligned: bool = False) -> rrb.Spatial3DView:
        return rrb.Spatial3DView(
            origin=origin, contents=[f"+ {origin}/**", "- /__properties/**"],
            name=name, background=[16, 18, 22],
            eye_controls=(
                rrb.EyeControls3D(
                    kind=rrb.Eye3DKind.Orbital,
                    position=(float(center[0]), float(center[1] - 1.2 * extent), float(center[2] + 0.9 * extent)),
                    look_target=tuple(float(value) for value in center),
                    eye_up=(0.0, 0.0, 1.0),
                ) if aligned else None
            ),
        )
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                spatial("/spx3/glim", "GLIM poses + logged Kimera→GLIM links"),
                spatial("/spx3/kimera", "Kimera poses + logged GLIM→Kimera links"),
                spatial("/spx3/aligned", "SPX-3 GT / native GLIM / native Kimera / fused", aligned=True),
                column_shares=[1, 1, 1.5],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(origin="/spx3/counts/K2G", name="Kimera→GLIM injections"),
                rrb.TimeSeriesView(origin="/spx3/counts/G2K", name="GLIM→Kimera applications"),
                rrb.TextDocumentView(origin="/spx3/notes", name="Run provenance and limits"),
                column_shares=[1, 1, 1.5],
            ),
            row_shares=[2, 1],
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded", play_state="paused", loop_mode="off"),
        auto_layout=False, auto_views=False,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--results-dir", required=True, type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    results = args.results_dir.resolve()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if (run_dir / "resolved_glim_params.yaml").read_text().find("glim_factor_graph_inspector_enable: false") < 0:
        raise ValueError("this helper is only for the frozen source run without inspector recording")
    if (run_dir / "resolved_kimera_params.yaml").read_text().find("rerun_factor_graph_inspector_enable: false") < 0:
        raise ValueError("this helper is only for the frozen source run without inspector recording")

    evaluation = results / "evaluation"
    glim_times, glim_native, _ = read_tum(run_dir / "odom_imu.txt")
    kimera_times, kimera_native, _ = read_kimera(run_dir / "kimera_odometry.csv")
    fused_times, _, _ = read_tum(results / "D" / "trajectory.tum")
    gt_times, gt_xyz, _ = read_tum(args.ground_truth.resolve())
    paths = {
        "ground_truth": (gt_times, gt_xyz),
        "glim": (glim_times, read_aligned(evaluation / "native_on_glim_prism_aligned.csv", glim_times)),
        "kimera": (kimera_times, read_aligned(evaluation / "native_on_kimera_prism_aligned.csv", kimera_times)),
        "gnssfgo": (fused_times, read_aligned(evaluation / "D_trajectory_prism_aligned.csv", fused_times)),
    }
    events, event_validation = external_events(run_dir, glim_times, kimera_times)
    metrics = {
        name: json.loads((evaluation / name).read_text(encoding="utf-8"))
        for name in ("native_on_glim_prism.json", "native_on_kimera_prism.json", "D_trajectory_prism.json")
    }
    cloud = map_points(run_dir, metrics["native_on_glim_prism.json"])

    recording_id = "spx3_classic_on_20260927_frozen_reconstruction"
    app_id = "spx3_classic_on_frozen_native_and_fusion"
    data_rrd = output.with_name(output.stem + "_data.rrd")
    rbl = output.with_name(output.stem + "_dashboard.rbl")
    rr.init(app_id, recording_id=recording_id)
    rr.save(data_rrd)
    for space in ("aligned", "glim", "kimera"):
        rr.log(f"spx3/{space}", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    # The only saved GLIM submap completes at the end of the run. Show its
    # final geometry statically, explicitly not as a time-progressive map.
    rr.log("spx3/aligned/final_glim_map", rr.Points3D(cloud, colors=[145, 155, 165, 75], radii=rr.Radius.ui_points(1.0)), static=True)
    for name, (times, xyz) in paths.items():
        progressive_path(f"spx3/aligned/{name}", times, xyz, COLORS[name], 0.5 if name == "ground_truth" else 1.0)
    graph_skeleton("spx3/glim", glim_times, glim_native, COLORS["glim"])
    graph_skeleton("spx3/kimera", kimera_times, kimera_native, COLORS["kimera"])
    for direction, local_xyz, space in (
        ("K2G", glim_native, "glim"), ("G2K", kimera_native, "kimera")
    ):
        selected = sorted((event for event in events if event["direction"] == direction), key=lambda e: e["event_time"])
        rr.set_time("time", timestamp=float(selected[0]["event_time"]))
        rr.log(f"spx3/counts/{direction}", rr.Scalars(0.0))
        for index, event in enumerate(selected, 1):
            a, b = event["from_index"], event["to_index"]
            rr.set_time("time", timestamp=float(event["event_time"]))
            rr.log(
                f"spx3/{space}/logged_external_CBS/{index:04d}_{a}_{b}",
                rr.LineStrips3D([local_xyz[[a, b]]], colors=[EXTERNAL_COLOR], radii=rr.Radius.ui_points(3.4)),
            )
            rr.log(f"spx3/counts/{direction}", rr.Scalars(float(index)))

    note = "\n".join([
        "# SPX-3 classic CBS-ON, frozen 2026-09-27 source capture",
        "",
        "Blue: AP20 prism GT; green: **native GLIM**; orange: **native Kimera ROS odometry**; red: **gnssFGO D final retrospectively revised joint graph**.",
        "All four trails reveal with physical timestamp. Each estimator is independently no-scale Kabsch aligned using its saved evaluator transform. The red path is an offline receiver replay, not a live co-running gnssFGO trajectory.",
        "",
        "**Graph-panel limitation:** source factor-graph inspectors were disabled in this exact run. The left panels reconstruct native pose history and the endpoint connectivity of logged, applied external CBS factors. Magenta links are the 662 K→G and 659 G→K factors from this run. They are cumulative insertion history, **not** a complete or currently active smoother-factor inventory. Kimera application times are reconstructed from wall/ROS clock anchors (held-out maximum error <1 ms).",
        "The gray point cloud is the run's **final single saved GLIM submap**, shown as static context; individual scan timing was not retained with those points. It is not represented as a live-growing cloud.",
        "",
        "| Frozen product | ATE RMSE | Full path |",
        "|---|---:|---:|",
        f"| Native GLIM ON | {metrics['native_on_glim_prism.json']['ape_rmse_m']:.6f} m | {metrics['native_on_glim_prism.json']['estimate_full_path_m']:.3f} m |",
        f"| Native Kimera ON | {metrics['native_on_kimera_prism.json']['ape_rmse_m']:.6f} m | {metrics['native_on_kimera_prism.json']['estimate_full_path_m']:.3f} m |",
        f"| gnssFGO D final | {metrics['D_trajectory_prism.json']['ape_rmse_m']:.6f} m | {metrics['D_trajectory_prism.json']['estimate_full_path_m']:.3f} m |",
        "",
        "Native Kimera's GT matches cover a different sampled set than GLIM/fusion. Metrics are copied from the frozen evaluator outputs, not recomputed in this viewer.",
    ])
    rr.log("spx3/notes", rr.TextDocument(note, media_type=rr.MediaType.MARKDOWN), static=True)
    rr.disconnect()
    all_xyz = np.concatenate([xyz for _, xyz in paths.values()])
    center = (np.min(all_xyz, axis=0) + np.max(all_xyz, axis=0)) / 2
    extent = max(float(np.max(np.max(all_xyz, axis=0) - np.min(all_xyz, axis=0))), 10.0)
    blueprint(center, extent).save(application_id=app_id, path=rbl)
    subprocess.run(["rerun", "rrd", "merge", "-o", str(output), str(data_rrd), str(rbl)], check=True)
    subprocess.run(["rerun", "rrd", "verify", str(output)], check=True)

    inputs = [
        run_dir / "odom_imu.txt", run_dir / "kimera_odometry.csv",
        run_dir / "recordings" / "cbs.bag", args.ground_truth.resolve(),
        results / "D" / "trajectory.tum",
        evaluation / "native_on_glim_prism_aligned.csv",
        evaluation / "native_on_kimera_prism_aligned.csv",
        evaluation / "D_trajectory_prism_aligned.csv",
        *source_logs(run_dir),
    ]
    validation = {
        "visualization_only": True,
        "source_inspectors_were_disabled": True,
        "receiver_product": "D final retrospectively revised trajectory.tum",
        "source_events": event_validation,
        "rows": {name: len(item[0]) for name, item in paths.items()},
        "pointcloud_final_submap_points": len(cloud),
        "source_sha256": {str(path): sha256(path) for path in inputs},
        "recording_sha256": sha256(output),
    }
    output.with_name(output.stem + "_validation.json").write_text(
        json.dumps(validation, indent=2) + "\n", encoding="utf-8"
    )
    with output.with_name(output.stem + "_external_factors.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(events[0].keys()))
        writer.writeheader()
        writer.writerows(sorted(events, key=lambda event: (event["event_time"], event["direction"])))
    print(output)


if __name__ == "__main__":
    main()
