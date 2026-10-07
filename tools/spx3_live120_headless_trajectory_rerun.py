#!/usr/bin/env python3
"""Render saved headless SPX-3 live trajectories and completed GLIM submaps.

The source graph inspectors were disabled during playback to avoid blocking the
estimators. This deliberately does not reconstruct or label source graph panels
as actual active-smoother snapshots.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from grandtour_eth1_gnssfgo_rerun import (
    aligned_full_path,
    line_strips,
    load_glim_map,
    read_tum,
    sampled_indices,
    sha256,
)


COLORS = {
    "ground_truth": [45, 125, 255, 255],
    "glim": [20, 205, 90, 255],
    "kimera": [255, 165, 0, 255],
    "fused_online": [235, 55, 55, 255],
    "fused_final": [195, 95, 235, 255],
}


def log_growing_path(name: str, times: np.ndarray, xyz: np.ndarray, anchor: np.ndarray) -> int:
    if len(times) < 2 or not np.all(np.diff(times) > 0):
        raise ValueError(f"invalid time grid: {name}")
    points = xyz - anchor
    rr.set_time("time", timestamp=float(times[0]))
    rr.log(f"scene/paths/{name}/start", rr.Points3D([points[0]], colors=[COLORS[name]], radii=0.18))
    trail = set(int(i) for i in sampled_indices(times, 0.5))
    current = sampled_indices(times, 0.2)
    for idx in np.union1d(current, np.fromiter(trail, dtype=int)):
        rr.set_time("time", timestamp=float(times[idx]))
        rr.log(f"scene/paths/{name}/current", rr.Points3D([points[idx]], colors=[COLORS[name]], radii=0.25))
        if int(idx) in trail and idx > 0:
            strips = line_strips(times[: idx + 1], points[: idx + 1], 0.5 if name == "ground_truth" else 1.0)
            if strips:
                rr.log(f"scene/paths/{name}/trail", rr.LineStrips3D(
                    strips, colors=[COLORS[name]] * len(strips), radii=rr.Radius.ui_points(3.0)
                ))
    return len(trail)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("off", "on"))
    parser.add_argument("--runs", required=True, type=Path)
    parser.add_argument("--products", required=True, type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    run = args.runs / f"gnssfgo_spx3_cbs_{args.mode}_live120_headless_20260929"
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    metrics_dir = args.products / "evaluation"
    sources = {
        "glim": run / "spx3_source/odom_imu.txt",
        "kimera": args.products / f"{args.mode}_native_kimera.tum",
        "fused_online": run / "receiver/receiver/online_trajectory.tum",
        "fused_final": run / "receiver/receiver/trajectory.tum",
    }
    start = 1730565903.588829994
    end = 1730566022.688653946
    gt_t, gt_xyz, _ = read_tum(args.ground_truth)
    gt_mask = (gt_t >= start) & (gt_t <= end)
    gt_t, gt_xyz = gt_t[gt_mask], gt_xyz[gt_mask]
    if len(gt_t) < 2:
        raise ValueError("ground truth has no coverage in the matched window")
    anchor = gt_xyz[0].copy()
    paths = {"ground_truth": (gt_t, gt_xyz)}
    metrics = {}
    for name, source in sources.items():
        times, xyz, quat = read_tum(source)
        metric_path = metrics_dir / f"{args.mode}_{'native_' if name in ('glim', 'kimera') else ''}{name if name in ('glim', 'kimera') else name}_prism.json"
        aligned, metric = aligned_full_path(xyz, quat, metric_path)
        mask = (times >= start) & (times <= end)
        paths[name] = times[mask], aligned[mask]
        metrics[name] = metric

    application = f"spx3_classic_cbs_{args.mode}_live120_headless"
    data_path = output.with_name(output.stem + "_data.rrd")
    blueprint_path = output.with_suffix(".rbl")
    rr.init(application, recording_id=application)
    rr.save(data_path)
    rr.log("scene", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    cloud_batches, cloud_info = load_glim_map(run / "spx3_source", metrics["glim"], anchor)
    for stamp, submap_id, points in cloud_batches:
        rr.set_time("time", timestamp=stamp)
        rr.log(f"scene/pointcloud/glim_hesai/{submap_id}", rr.Points3D(
            points, colors=[145, 155, 165, 100], radii=rr.Radius.ui_points(1.0)
        ))
    updates = {}
    for name, (times, xyz) in paths.items():
        updates[name] = log_growing_path(name, times, xyz, anchor)
    notes = [
        f"# SPX-3 classic CBS-{args.mode.upper()}, live 120 s",
        "",
        "Blue: AP20 prism GT; green: native GLIM; orange: native Kimera; red: **live/as-played** gnssFGO; purple: **retrospectively revised** gnssFGO.",
        "All trajectories are scored using the same saved prism evaluator and shown in its aligned world frame. The purple final trajectory uses future revisions and is not an online estimate. The timeline is physical pose time; it does not reproduce GUI wall-clock latency.",
        "GLIM map submaps appear when their saved last-frame timestamp is reached. They are completed-submap playback, not individual live scans.",
        "The native GLIM/Kimera active factor-graph inspectors were intentionally disabled during this headless run. No actual source graph snapshots are claimed here.",
        "",
        "| Trajectory | ATE RMSE | Full path |",
        "|---|---:|---:|",
    ]
    for name, metric in metrics.items():
        notes.append(f"| {name} | {metric['ape_rmse_m']:.4f} m | {metric['estimate_full_path_m']:.1f} m |")
    rr.log("notes", rr.TextDocument("\n".join(notes), media_type=rr.MediaType.MARKDOWN), static=True)
    rr.disconnect()

    mins = np.min(np.vstack([xyz.min(axis=0) for _, xyz in paths.values()]), axis=0) - anchor
    maxs = np.max(np.vstack([xyz.max(axis=0) for _, xyz in paths.values()]), axis=0) - anchor
    center = 0.5 * (mins + maxs)
    extent = max(float(np.max(maxs - mins)), 10.0)
    view = rrb.Spatial3DView(
        origin="/scene", contents=["+ /scene/**", "- /scene/paths/fused_final/**"],
        name="GT / native GLIM / native Kimera / live fused + map",
        background=[16, 18, 22],
        eye_controls=rrb.EyeControls3D(
            kind=rrb.Eye3DKind.Orbital,
            position=(float(center[0]), float(center[1] - 1.2 * extent), float(center[2] + 0.8 * extent)),
            look_target=tuple(float(v) for v in center), eye_up=(0.0, 0.0, 1.0),
        ),
    )
    final_view = rrb.Spatial3DView(
        origin="/scene", contents=["+ /scene/paths/ground_truth/**", "+ /scene/paths/fused_online/**", "+ /scene/paths/fused_final/**"],
        name="Live fused vs retrospective final vs GT", background=[16, 18, 22]
    )
    blueprint = rrb.Blueprint(
        rrb.Horizontal(view, final_view, rrb.TextDocumentView(origin="/notes", name="Run identity and metrics"), column_shares=[1.5, 1, 0.9]),
        rrb.BlueprintPanel(state="collapsed"), rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded", play_state="paused", loop_mode="off"),
        auto_layout=False, auto_views=False,
    )
    blueprint.save(application_id=application, path=blueprint_path)
    subprocess.run(["/home/yeranis/.local/bin/rerun", "rrd", "merge", "-o", str(output), str(data_path), str(blueprint_path)], check=True)
    subprocess.run(["/home/yeranis/.local/bin/rerun", "rrd", "verify", str(output)], check=True)
    validation = {
        "mode": args.mode,
        "recording": str(output),
        "sha256": sha256(output),
        "sources": {name: {"path": str(path), "sha256": sha256(path)} for name, path in sources.items()},
        "ground_truth": {"path": str(args.ground_truth), "sha256": sha256(args.ground_truth)},
        "path_updates": updates,
        "cloud": cloud_info,
        "actual_source_graph_snapshots": False,
    }
    output.with_suffix(".json").write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "sha256": validation["sha256"], "cloud": cloud_info}, indent=2))


if __name__ == "__main__":
    main()
