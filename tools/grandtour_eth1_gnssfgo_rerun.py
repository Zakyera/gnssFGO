#!/usr/bin/env python3
"""Add matched native/fused ETH1 trajectories to a saved CBS factor-graph RRD.

This is visualization only. The same lever arm and Kabsch transforms saved by
score_grandtour_prism.py are applied to *all* output poses, including those
outside the intervals where Leica prism ground truth is available.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb


PRISM_LEVER_CPT7 = np.array([0.308837802, -0.038312997, -0.234944116])
COLORS = {
    "ground_truth": [45, 125, 255, 255],
    "glim": [20, 205, 90, 255],
    "kimera": [255, 165, 0, 255],
    "gnssfgo": [235, 55, 55, 255],
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    values = np.loadtxt(path, dtype=float)
    if values.ndim != 2 or values.shape[1] < 8:
        raise ValueError(f"invalid TUM trajectory: {path}")
    return values[:, 0], values[:, 1:4], values[:, 4:8]


def read_kimera(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(newline="", encoding="utf-8")))
    times = np.asarray([float(row["field.header.stamp"]) * 1e-9 for row in rows])
    xyz = np.asarray([
        [float(row[f"field.pose.pose.position.{axis}"]) for axis in "xyz"]
        for row in rows
    ])
    quat = np.asarray([
        [float(row[f"field.pose.pose.orientation.{axis}"]) for axis in "xyzw"]
        for row in rows
    ])
    if len(times) < 2:
        raise ValueError(f"empty Kimera trajectory: {path}")
    return times, xyz, quat


def quaternion_rotate(q_xyzw: np.ndarray, vector: np.ndarray) -> np.ndarray:
    q = q_xyzw / np.linalg.norm(q_xyzw, axis=1)[:, None]
    qv = q[:, :3]
    twice_cross = 2.0 * np.cross(qv, vector[None, :])
    return vector[None, :] + q[:, 3:4] * twice_cross + np.cross(qv, twice_cross)


def aligned_full_path(
    xyz: np.ndarray, quat: np.ndarray, metrics_path: Path
) -> tuple[np.ndarray, dict]:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    if metrics["source_frame"] != "cpt7_imu" or metrics["scale_correction"]:
        raise ValueError(f"unexpected scorer convention in {metrics_path}")
    if not np.allclose(metrics["source_to_prism_translation_m"], PRISM_LEVER_CPT7):
        raise ValueError(f"unexpected prism lever arm in {metrics_path}")
    rotation = np.asarray(metrics["alignment_rotation"], dtype=float)
    translation = np.asarray(metrics["alignment_translation_m"], dtype=float)
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8):
        raise ValueError(f"invalid metric alignment in {metrics_path}")
    prism = xyz + quaternion_rotate(quat, PRISM_LEVER_CPT7)
    return prism @ rotation.T + translation, metrics


def check_aligned_csv(times: np.ndarray, xyz: np.ndarray, path: Path) -> dict:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    csv_times = np.asarray([float(row["timestamp"]) for row in rows])
    csv_xyz = np.asarray([
        [float(row[f"aligned_{axis}"]) for axis in "xyz"] for row in rows
    ])
    if len(csv_times) < 2:
        raise ValueError(f"no matched scorer poses in {path}")
    index = np.searchsorted(times, csv_times)
    index = np.clip(index, 0, len(times) - 1)
    previous = np.maximum(index - 1, 0)
    choose_previous = np.abs(times[previous] - csv_times) < np.abs(times[index] - csv_times)
    index[choose_previous] = previous[choose_previous]
    max_time = float(np.max(np.abs(times[index] - csv_times)))
    max_position = float(np.max(np.linalg.norm(xyz[index] - csv_xyz, axis=1)))
    if max_time > 2e-6 or max_position > 1e-5:
        raise ValueError(
            f"full-path alignment disagrees with scorer CSV {path}: "
            f"dt={max_time}, position={max_position}"
        )
    return {
        "matched_rows": len(rows),
        "max_timestamp_difference_s": max_time,
        "max_position_difference_m": max_position,
    }


def read_ground_truth(path: Path) -> tuple[np.ndarray, np.ndarray]:
    values = np.loadtxt(path, dtype=float)
    if values.ndim != 2 or values.shape[1] < 4:
        raise ValueError(f"invalid ground truth: {path}")
    return values[:, 0], values[:, 1:4]


def line_strips(times: np.ndarray, xyz: np.ndarray, gap_s: float) -> list[np.ndarray]:
    boundaries = np.flatnonzero(np.diff(times) > gap_s) + 1
    return [piece for piece in np.split(xyz, boundaries) if len(piece) >= 2]


def log_path(name: str, times: np.ndarray, xyz: np.ndarray, anchor: np.ndarray) -> None:
    color = COLORS[name]
    points = xyz - anchor
    strips = line_strips(times, points, 0.5 if name == "ground_truth" else 1.0)
    if not strips:
        raise ValueError(f"no connected path segments for {name}")
    rr.log(
        f"gnssfgo_presentation/trajectories/{name}/full_path",
        rr.LineStrips3D(
            strips, colors=[color] * len(strips), radii=rr.Radius.ui_points(3.0)
        ),
        static=True,
    )
    rr.log(
        f"gnssfgo_presentation/trajectories/{name}/start",
        rr.Points3D([points[0]], colors=[color], radii=0.2),
        static=True,
    )
    stride = max(1, len(times) // 700)
    for index in range(0, len(times), stride):
        rr.set_time("time", timestamp=float(times[index]))
        rr.log(
            f"gnssfgo_presentation/trajectories/{name}/current",
            rr.Points3D([points[index]], colors=[color], radii=0.3),
        )


def make_blueprint(center: np.ndarray, extent: float) -> rrb.Blueprint:
    glim_graph = rrb.Spatial3DView(
        origin="/glim/factor_graph_inspector/spatial",
        contents=["+ /glim/factor_graph_inspector/spatial/**", "- /__properties/**"],
        name="GLIM graph — Kimera→GLIM CBS factors",
        background=[16, 18, 22],
    )
    kimera_graph = rrb.Spatial3DView(
        origin="/kimera/factor_graph_inspector/spatial",
        contents=[
            "+ /kimera/factor_graph_inspector/spatial/context_trajectory",
            "+ /kimera/factor_graph_inspector/spatial/states/**",
            "+ /kimera/factor_graph_inspector/spatial/factors/**",
            "+ /kimera/factor_graph_inspector/spatial/factor_markers/cbs_g2k_pose_between",
            "- /__properties/**",
        ],
        name="Kimera graph — GLIM→Kimera CBS factors",
        background=[16, 18, 22],
    )
    trajectories = rrb.Spatial3DView(
        origin="/gnssfgo_presentation/trajectories",
        contents=["+ /gnssfgo_presentation/trajectories/**", "- /__properties/**"],
        name="ETH1 — ground truth, native GLIM/Kimera, gnssFGO fusion",
        background=[16, 18, 22],
        line_grid=rrb.LineGrid3D(
            visible=True, spacing=5.0, stroke_width=1.0, color=[90, 96, 105, 90]
        ),
        eye_controls=rrb.EyeControls3D(
            kind=rrb.Eye3DKind.Orbital,
            position=(float(center[0]), float(center[1] - 1.2 * extent), float(center[2] + 0.9 * extent)),
            look_target=tuple(float(value) for value in center),
            eye_up=(0.0, 0.0, 1.0),
        ),
    )
    glim_external = rrb.TimeSeriesView(
        origin="/glim/factor_graph_inspector/counts",
        contents=[
            "+ /glim/factor_graph_inspector/counts/cbs_k2g_pose_between_factors",
            "- /__properties/**",
        ],
        name="Injected Kimera→GLIM factors",
    )
    kimera_external = rrb.TimeSeriesView(
        origin="/kimera/factor_graph_inspector/counts",
        contents=[
            "+ /kimera/factor_graph_inspector/counts/cbs_g2k_pose_between_factors",
            "- /__properties/**",
        ],
        name="Injected GLIM→Kimera factors",
    )
    notes = rrb.TextDocumentView(
        origin="/gnssfgo_presentation/notes", name="Run identity and metrics"
    )
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(glim_graph, kimera_graph, trajectories, column_shares=[1, 1, 1.3]),
            rrb.Horizontal(glim_external, kimera_external, notes, column_shares=[1, 1, 1.3]),
            row_shares=[2, 1],
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded", play_state="paused", loop_mode="off"),
        auto_layout=False,
        auto_views=False,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--graph-trajectory", required=True, type=Path)
    parser.add_argument("--graph-metrics", required=True, type=Path)
    parser.add_argument("--graph-aligned-csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    evaluation = run_dir / "evaluation"
    recording_id = (run_dir / "rerun_recording_id.txt").read_text().strip()
    inputs = {
        "glim": (run_dir / "odom_imu.txt", evaluation / "glim_metrics_prism.json", evaluation / "glim_prism_aligned.csv"),
        "kimera": (run_dir / "kimera_odometry.csv", evaluation / "kimera_metrics_prism.json", evaluation / "kimera_prism_aligned.csv"),
        "gnssfgo": (args.graph_trajectory.resolve(), args.graph_metrics.resolve(), args.graph_aligned_csv.resolve()),
    }
    for paths in inputs.values():
        for path in paths:
            if not path.is_file():
                raise FileNotFoundError(path)

    gt_path = evaluation / "ground_truth_prism.tum"
    gt_times, gt_xyz = read_ground_truth(gt_path)
    anchor = gt_xyz[0].copy()
    outputs = {}
    checks = {}
    for name, (trajectory, metrics, aligned_csv) in inputs.items():
        times, xyz, quaternion = read_kimera(trajectory) if name == "kimera" else read_tum(trajectory)
        aligned_xyz, result = aligned_full_path(xyz, quaternion, metrics)
        checks[name] = check_aligned_csv(times, aligned_xyz, aligned_csv)
        outputs[name] = (times, aligned_xyz, result)

    all_xyz = np.concatenate([gt_xyz - anchor] + [item[1] - anchor for item in outputs.values()])
    lower = np.min(all_xyz, axis=0)
    upper = np.max(all_xyz, axis=0)
    center = (lower + upper) / 2
    extent = max(float(np.max(upper - lower)), 10.0)

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    overlay = output.parent / "gnssfgo_overlay.rrd"
    blueprint = output.parent / "gnssfgo_dashboard.rbl"
    rr.init("cbsms", recording_id=recording_id)
    rr.save(overlay)
    rr.log("gnssfgo_presentation/trajectories", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    log_path("ground_truth", gt_times, gt_xyz, anchor)
    for name, (times, xyz, _) in outputs.items():
        log_path(name, times, xyz, anchor)
    lines = [
        "# ETH1 CBS-ON, 0.15× — matched presentation replay",
        "",
        "Blue: Leica prism GT (only where observed). Green: native GLIM. Orange: native Kimera. Red: gnssFGO GLIM+Kimera.",
        "All estimator paths show their **full output**; GT gaps are disconnected. Poses are transformed to the prism point and independently rigid-aligned without scale, using this replay's saved evaluator transforms. Only observed prism intervals contribute to ATE.",
        "",
        "| Trajectory | Prism ATE RMSE | Full path | Matched prism coverage |",
        "|---|---:|---:|---:|",
    ]
    for name, (_, _, metrics) in outputs.items():
        lines.append(
            f"| {name} | {metrics['ape_rmse_m']:.3f} m | {metrics['estimate_full_path_m']:.1f} m | {metrics['matched_elapsed_coverage_s']:.1f} s |"
        )
    lines.extend([
        "", "The graph panels are the **live source inspectors** from this same CBS-ON replay. The red trajectory is an offline causal gnssFGO replay of beliefs exported from its saved bag; it was not estimated live inside GLIM or Kimera.",
        "", f"Recording ID: `{recording_id}`",
    ])
    rr.log(
        "gnssfgo_presentation/notes",
        rr.TextDocument("\n".join(lines), media_type=rr.MediaType.MARKDOWN),
        static=True,
    )
    rr.disconnect()
    make_blueprint(center, extent).save(application_id="cbsms", path=blueprint)
    source_rrd = run_dir / "recordings" / f"{run_dir.name}.rrd"
    if not source_rrd.is_file():
        raise FileNotFoundError(source_rrd)
    subprocess.run(
        ["rerun", "rrd", "merge", "-o", str(output), str(source_rrd), str(overlay), str(blueprint)],
        check=True,
    )
    subprocess.run(["rerun", "rrd", "verify", str(output)], check=True)
    validation = {
        "recording_id": recording_id,
        "source_rrd": str(source_rrd),
        "source_rrd_sha256": sha256(source_rrd),
        "output_rrd": str(output),
        "output_rrd_sha256": sha256(output),
        "trajectory_sha256": {name: sha256(paths[0]) for name, paths in inputs.items()},
        "aligned_csv_checks": checks,
        "gt_rows": len(gt_times),
        "trajectory_rows": {name: len(item[0]) for name, item in outputs.items()},
    }
    (output.parent / "gnssfgo_rerun_validation.json").write_text(
        json.dumps(validation, indent=2) + "\n", encoding="utf-8"
    )
    print(output)


if __name__ == "__main__":
    main()
