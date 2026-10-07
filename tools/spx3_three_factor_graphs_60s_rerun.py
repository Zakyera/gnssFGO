#!/usr/bin/env python3
"""Combine real source graph inspectors with receiver spatial graph snapshots.

The source panels are the estimators' recorded inspectors.  The gnssFGO panel
uses optimized pose snapshots and active relative-factor keys read back from
iSAM2 by the existing receiver's optional visualization logger. IMU/WNOJ
connections are the unchanged adjacent-support factors in that same graph.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from grandtour_eth1_gnssfgo_rerun import read_kimera, read_tum, sampled_indices


COLORS = {
    "gt": [48, 126, 250, 255],
    "glim": [25, 215, 95, 255],
    "kimera": [255, 170, 30, 255],
    "fused": [245, 58, 68, 255],
    "imu": [50, 195, 240, 255],
    "wnoj": [185, 100, 245, 255],
}
LANES = {"X": 0.0, "V": -1.0, "B": -2.0, "W": 1.0}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def table(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def live_revisions(path: Path, source_config: Path) -> list[dict[str, str]]:
    """Read the exact source identities/receipt times sent to the live receiver."""
    config = json.loads(source_config.read_text(encoding="utf-8"))
    agents = {source["id"]: {"glim": "g", "kimera": "k"}[source["kind"]]
              for source in config["sources"]}
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for event in csv.reader(stream):
            if event[0] != "B":
                continue
            if len(event) != 52 or event[2] not in agents:
                raise ValueError("invalid live source-belief event")
            rows.append({
                "source_agent": agents[event[2]],
                "ordinal": event[4],
                "revision": event[5],
                "from_index": event[6],
                "to_index": event[7],
                "from_timestamp_ns": event[8],
                "to_timestamp_ns": event[9],
                "record_timestamp_ns": event[1],
            })
    if not rows:
        raise ValueError("live journal contains no source-belief events")
    return rows


def aligned(path: Path, times: np.ndarray) -> np.ndarray:
    rows = table(path)
    found = np.asarray([float(row["timestamp"]) for row in rows])
    if len(found) != len(times) or np.max(np.abs(found - times)) > 2e-6:
        raise ValueError(f"alignment file has a different timestamp grid: {path}")
    xyz = np.asarray([[float(row[f"aligned_{axis}"]) for axis in "xyz"] for row in rows])
    if not np.isfinite(xyz).all():
        raise ValueError(f"nonfinite aligned positions: {path}")
    return xyz


def support_keys(timestamp_ns: int, grid: list[int]) -> list[tuple[str, int]]:
    index = bisect.bisect_left(grid, timestamp_ns)
    if index < len(grid) and grid[index] == timestamp_ns:
        return [("X", index)]
    if index == 0 or index == len(grid):
        raise ValueError(f"relative endpoint outside receiver support: {timestamp_ns}")
    return [(name, support) for support in (index - 1, index) for name in ("X", "V", "W")]


def receiver_events(revisions: list[dict], ledger_rows: list[dict], grid: list[int]) -> tuple[list[dict], dict]:
    by_row = {}
    for row in revisions:
        key = tuple(int(row[field]) for field in (
            "ordinal", "from_index", "to_index", "revision", "record_timestamp_ns"))
        if key in by_row:
            raise ValueError(f"ambiguous source event key: {key}")
        by_row[key] = row
    result = []
    outcomes = {"insert": 0, "replace": 0, "duplicate": 0, "stale": 0}
    for row in ledger_rows:
        outcome = row["outcome"]
        if outcome not in outcomes:
            raise ValueError(f"unexpected receiver outcome: {outcome}")
        outcomes[outcome] += 1
        if outcome in ("duplicate", "stale"):
            continue
        key = tuple(int(row[field]) for field in (
            "ordinal", "from_index", "to_index", "revision", "record_timestamp_ns"))
        source = by_row.get(key)
        if source is None:
            raise ValueError(f"accepted receiver event absent from input stream: {key}")
        from_time = int(source["from_timestamp_ns"])
        to_time = int(source["to_timestamp_ns"])
        endpoints = list(dict.fromkeys(
            support_keys(from_time, grid) + support_keys(to_time, grid)))
        latest_support = max(grid[index] for _, index in endpoints)
        record_time = int(source["record_timestamp_ns"])
        if record_time < to_time:
            raise ValueError("source arrival before its physical endpoint")
        result.append({
            "source": source["source_agent"],
            "id": (source["source_agent"], int(source["from_index"]), int(source["to_index"])),
            "revision": int(source["revision"]),
            "record_ns": record_time,
            "display_ns": max(record_time, latest_support),
            "from_ns": from_time,
            "to_ns": to_time,
            "keys": endpoints,
            "factor_index": int(row["new_factor_index"]),
            "outcome": outcome,
        })
    result.sort(key=lambda event: (event["display_ns"], event["record_ns"], event["id"], event["revision"]))
    return result, outcomes


def path_history(path: str, times: np.ndarray, xyz: np.ndarray, color: list[int], gap_s: float) -> None:
    gap_indices = np.flatnonzero(np.diff(times) > gap_s) + 1
    for index in sampled_indices(times, 0.35):
        rr.set_time("time", timestamp=float(times[index]))
        rr.log(f"{path}/current", rr.Points3D([xyz[index]], colors=[color], radii=0.23))
        if index:
            pieces = [piece for piece in np.split(xyz[: index + 1], gap_indices[gap_indices <= index])
                      if len(piece) > 1]
            if pieces:
                rr.log(f"{path}/trail", rr.LineStrips3D(
                    pieces, colors=[color] * len(pieces), radii=rr.Radius.ui_points(2.5)))


def draw_receiver_graph(
    grid: list[int], events: list[dict], *, window_seconds: float = 4.0,
) -> dict:
    origin_ns = grid[0]
    last_ns = max(grid[-1], max((event["display_ns"] for event in events), default=grid[-1]))
    snapshots = list(range(origin_ns, last_ns + 1, 1_000_000_000))
    if snapshots[-1] != last_ns:
        snapshots.append(last_ns)
    active: dict[tuple[str, int, int], dict] = {}
    next_event = 0
    last_revision_by_id: dict[tuple[str, int, int], int] = {}
    for now_ns in snapshots:
        while next_event < len(events) and events[next_event]["display_ns"] <= now_ns:
            event = events[next_event]
            prior = last_revision_by_id.get(event["id"], -1)
            if event["revision"] <= prior:
                raise ValueError("receiver accepted a non-increasing source revision")
            last_revision_by_id[event["id"]] = event["revision"]
            active[event["id"]] = event
            next_event += 1
        now = (now_ns - origin_ns) * 1e-9
        left = now - window_seconds
        display_x = lambda stamp: window_seconds + (stamp - now_ns) * 1e-9  # noqa: E731
        indices = [i for i, timestamp in enumerate(grid)
                   if left - 0.25 <= (timestamp - origin_ns) * 1e-9 <= now + 1e-9]
        if not indices:
            continue
        rr.set_time("time", timestamp=now_ns * 1e-9)
        for name, lane in LANES.items():
            points = [[display_x(grid[i]), lane] for i in indices]
            rr.log(f"gnssfgo/factor_graph/keys/{name}", rr.Points2D(
                points, colors=[195, 205, 215, 255], radii=rr.Radius.ui_points(3.4)))
        for name, factor_y, key_names in (
            ("imu", -1.55, ("X", "V", "B")),
            ("wnoj", 1.55, ("X", "V", "W")),
        ):
            centers, arms = [], []
            for i in indices:
                if not i or i - 1 not in indices or grid[i] > now_ns:
                    continue
                x0 = display_x(grid[i - 1])
                x1 = display_x(grid[i])
                center = [(x0 + x1) / 2, factor_y]
                centers.append(center)
                for x in (x0, x1):
                    for key_name in key_names:
                        arms.append([center, [x, LANES[key_name]]])
            rr.log(f"gnssfgo/factor_graph/factors/{name}/nodes", rr.Points2D(
                centers, colors=COLORS[name], radii=rr.Radius.ui_points(3.6)))
            rr.log(f"gnssfgo/factor_graph/factors/{name}/links", rr.LineStrips2D(
                arms, colors=COLORS[name], radii=rr.Radius.ui_points(1.0)))
        visible_counts = {}
        for source, lane, color in (("g", 2.6, COLORS["glim"]), ("k", 3.6, COLORS["kimera"])):
            centers, arms = [], []
            visible = [event for event in active.values()
                       if event["source"] == source
                       and left - 0.25 <= (event["to_ns"] - origin_ns) * 1e-9 <= now]
            for event in visible:
                x0 = display_x(event["from_ns"])
                x1 = display_x(event["to_ns"])
                center = [(x0 + x1) / 2, lane]
                centers.append(center)
                for key_name, index in event["keys"]:
                    arms.append([center, [display_x(grid[index]), LANES[key_name]]])
            rr.log(f"gnssfgo/factor_graph/factors/{'glim' if source == 'g' else 'kimera'}/nodes",
                   rr.Points2D(centers, colors=color, radii=rr.Radius.ui_points(5.2)))
            rr.log(f"gnssfgo/factor_graph/factors/{'glim' if source == 'g' else 'kimera'}/links",
                   rr.LineStrips2D(arms, colors=color, radii=rr.Radius.ui_points(1.9)))
            visible_counts[source] = len(visible)
        rr.log("gnssfgo/counts/active_glim_ids", rr.Scalars(float(sum(e["source"] == "g" for e in active.values()))))
        rr.log("gnssfgo/counts/active_kimera_ids", rr.Scalars(float(sum(e["source"] == "k" for e in active.values()))))
        rr.log("gnssfgo/factor_graph/window_note", rr.TextDocument(
            f"Receiver factor-key incidence, last {window_seconds:g} s of *physical* support. "
            f"Current arrival time {now:.2f} s; visible GLIM/Kimera IDs "
            f"{visible_counts['g']}/{visible_counts['k']}. Older active factors remain in the "
            "full-history graph but are outside this display window.",
            media_type=rr.MediaType.MARKDOWN))
    return {
        "snapshots": len(snapshots), "accepted_versions": len(events),
        "active_glim_ids": sum(event["source"] == "g" for event in active.values()),
        "active_kimera_ids": sum(event["source"] == "k" for event in active.values()),
    }


def draw_receiver_spatial_graph(
    grid: list[int], pose_rows: list[dict[str, str]],
    factor_rows: list[dict[str, str]], online_xyz: np.ndarray,
    source_kinds: dict[str, str] | None = None,
    snapshot_time_tolerance_ns: int = 0,
) -> dict:
    """Display the growing, complete iSAM2 pose/factor history at each snapshot."""
    poses_by_snapshot: dict[int, list[dict[str, str]]] = {}
    factors_by_snapshot: dict[int, list[dict[str, str]]] = {}
    for row in pose_rows:
        poses_by_snapshot.setdefault(int(row["snapshot_index"]), []).append(row)
    for row in factor_rows:
        factors_by_snapshot.setdefault(int(row["snapshot_index"]), []).append(row)
    snapshot_indices = sorted(poses_by_snapshot)
    if snapshot_indices not in (list(range(len(grid))), list(range(len(grid) + 1))):
        raise ValueError("missing or extra receiver graph pose snapshots")
    if len(online_xyz) != len(grid):
        raise ValueError("receiver online pose count does not match support grid")
    snapshot_factor_counts = []
    visible_factor_counts = {"glim": 0, "kimera": 0}
    rr.log("gnssfgo/spatial_graph", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for snapshot_index in snapshot_indices:
        rows = poses_by_snapshot[snapshot_index]
        stamp = int(rows[0]["snapshot_timestamp_ns"])
        current_index = min(snapshot_index, len(grid) - 1)
        if snapshot_index < len(grid) and abs(stamp - grid[snapshot_index]) > snapshot_time_tolerance_ns:
            raise ValueError("support snapshot timestamp disagrees with support grid")
        if snapshot_index == len(grid) and (
            stamp < grid[-1] or (stamp == grid[-1] and snapshot_time_tolerance_ns == 0)
        ):
            raise ValueError("final arrival snapshot must follow the final support")
        if any(int(row["snapshot_timestamp_ns"]) != stamp for row in rows):
            raise ValueError("pose snapshot timestamp disagrees with support grid")
        raw_points = {
            int(row["state_index"]): np.asarray(
                [float(row["px"]), float(row["py"]), float(row["pz"])])
            for row in rows
        }
        if len(raw_points) != len(rows) or set(raw_points) != set(range(current_index + 1)):
            raise ValueError("receiver graph snapshot has duplicate or missing historical states")
        points = raw_points
        indices = sorted(points)
        rr.set_time("time", timestamp=stamp * 1e-9)
        current = points[current_index]
        rr.log("gnssfgo/spatial_graph/states/support_poses", rr.Points3D(
            [points[index] for index in indices],
            colors=[[205, 211, 220, 255]] * len(indices),
            radii=rr.Radius.ui_points(5)))
        rr.log("gnssfgo/spatial_graph/states/active_path", rr.LineStrips3D(
            [[points[index] for index in indices]],
            colors=[[150, 160, 174, 255]], radii=rr.Radius.ui_points(2.0)))
        rr.log("gnssfgo/spatial_graph/states/as_played_context", rr.LineStrips3D(
            [online_xyz[:current_index + 1]],
            colors=[[105, 112, 122, 100]],
            radii=rr.Radius.ui_points(1.0)))
        rr.log("gnssfgo/spatial_graph/states/current_robot", rr.Points3D(
            [current], labels=[f"X{current_index}"], colors=[[245, 58, 68, 255]],
            radii=rr.Radius.ui_points(11)))
        current_row = next(row for row in rows if int(row["state_index"]) == current_index)
        qx, qy, qz, qw = (float(current_row[key]) for key in ("qx", "qy", "qz", "qw"))
        forward = np.asarray([
            1 - 2 * (qy * qy + qz * qz),
            2 * (qx * qy + qw * qz),
            2 * (qx * qz - qw * qy),
        ])
        rr.log("gnssfgo/spatial_graph/states/robot_heading", rr.Arrows3D(
            origins=[current], vectors=[0.3 * forward],
            colors=[[245, 58, 68, 255]], radii=rr.Radius.ui_points(3)))
        for name, offset, key_color in (
            ("combined_imu", -0.10, COLORS["imu"]),
            ("wnoj", 0.10, COLORS["wnoj"]),
        ):
            centers, arms = [], []
            for index in indices:
                if index - 1 not in points:
                    continue
                center = 0.5 * (points[index - 1] + points[index])
                center = center + np.asarray([0.0, 0.0, offset])
                centers.append(center)
                arms.extend([[center, points[index - 1]], [center, points[index]]])
            rr.log(f"gnssfgo/spatial_graph/factors/{name}/nodes", rr.Points3D(
                centers, colors=[key_color] * len(centers),
                radii=rr.Radius.ui_points(3.5)))
            rr.log(f"gnssfgo/spatial_graph/factors/{name}/links", rr.LineStrips3D(
                arms, colors=[key_color] * len(arms),
                radii=rr.Radius.ui_points(1.4)))
        by_source: dict[str, tuple[list[np.ndarray], list[list[np.ndarray]]]] = {
            "glim": ([], []), "kimera": ([], [])}
        for row in factors_by_snapshot.get(snapshot_index, []):
            if int(row["snapshot_timestamp_ns"]) != stamp:
                raise ValueError("factor snapshot timestamp disagrees with support grid")
            source = (source_kinds or {}).get(row["source"], row["source"])
            if source not in by_source:
                raise ValueError(f"unexpected receiver factor source: {source}")
            tokens = row["keys"].split(";")
            actual_keys = {(token[0].upper(), int(token[1:])) for token in tokens}
            expected_keys = set(support_keys(int(row["from_timestamp_ns"]), grid) +
                                support_keys(int(row["to_timestamp_ns"]), grid))
            if actual_keys != expected_keys:
                raise ValueError(f"iSAM2 active factor-key mismatch at snapshot {snapshot_index}")
            pose_keys = sorted(index for name, index in actual_keys if name == "X")
            if any(index not in points for index in pose_keys):
                raise ValueError("active source factor references a missing historical pose")
            support_xyz = np.asarray([points[index] for index in pose_keys])
            offset = 0.22 if source == "glim" else 0.42
            center = np.mean(support_xyz, axis=0) + np.asarray([0.0, 0.0, offset])
            centers, arms = by_source[source]
            centers.append(center)
            arms.extend([[center, point] for point in support_xyz])
        for source, (centers, arms) in by_source.items():
            color = COLORS[source]
            visible_factor_counts[source] += len(centers)
            rr.log(f"gnssfgo/spatial_graph/factors/{source}/nodes", rr.Points3D(
                centers, colors=[color] * len(centers),
                radii=rr.Radius.ui_points(5.5)))
            rr.log(f"gnssfgo/spatial_graph/factors/{source}/links", rr.LineStrips3D(
                arms, colors=[color] * len(arms),
                radii=rr.Radius.ui_points(2.0)))
        snapshot_factor_counts.append(len(factors_by_snapshot.get(snapshot_index, [])))
    return {
        "snapshots": len(snapshot_indices),
        "factor_snapshot_rows": sum(snapshot_factor_counts),
        "final_active_source_factors": snapshot_factor_counts[-1],
        "maximum_source_factors_in_snapshot": max(snapshot_factor_counts),
        "visible_factor_instances": visible_factor_counts,
        "layout": "complete optimized iSAM2 pose history in a fixed receiver frame; source factors use actual active keys",
    }


def make_blueprint(center: np.ndarray, extent: float,
                   receiver_center: np.ndarray, receiver_extent: float) -> rrb.Blueprint:
    view3d = lambda path, name: rrb.Spatial3DView(  # noqa: E731
        origin=path, contents=f"+ {path}/**", name=name, background=[16, 18, 22])
    trajectory_view = rrb.Spatial3DView(
        origin="/spx3/trajectories", contents="+ /spx3/trajectories/**",
        name="GT / native GLIM / native Kimera / gnssFGO final", background=[16, 18, 22],
        eye_controls=rrb.EyeControls3D(
            kind=rrb.Eye3DKind.Orbital,
            position=(float(center[0]), float(center[1] - 1.3 * extent),
                      float(center[2] + 0.9 * extent)),
            look_target=tuple(float(value) for value in center),
            eye_up=(0.0, 0.0, 1.0),
        ),
    )
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                view3d("/glim/factor_graph_inspector/spatial", "GLIM actual active smoother graph"),
                view3d("/kimera/factor_graph_inspector/spatial", "Kimera actual active smoother graph"),
                rrb.Spatial3DView(
                    origin="/gnssfgo/spatial_graph", contents="+ /gnssfgo/spatial_graph/**",
                    name="gnssFGO complete growing factor graph",
                    background=[16, 18, 22],
                    eye_controls=rrb.EyeControls3D(
                        kind=rrb.Eye3DKind.Orbital,
                        position=(float(receiver_center[0]),
                                  float(receiver_center[1] - 0.9 * receiver_extent),
                                  float(receiver_center[2] + 0.65 * receiver_extent)),
                        look_target=tuple(float(value) for value in receiver_center),
                        eye_up=(0.0, 0.0, 1.0))),
                column_shares=[1, 1, 1.25],
            ),
            rrb.Horizontal(
                trajectory_view,
                rrb.TimeSeriesView(origin="/gnssfgo/counts", name="Active receiver GLIM/Kimera IDs"),
                rrb.TextDocumentView(origin="/spx3/notes", name="How to read the three graphs"),
                column_shares=[1.5, 1, 1.3],
            ),
            row_shares=[2, 1],
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded", play_state="following", loop_mode="off"),
        auto_layout=False, auto_views=False,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-rrd", required=True, type=Path)
    parser.add_argument("--source-run", required=True, type=Path)
    parser.add_argument("--receiver", required=True, type=Path)
    parser.add_argument("--receiver-snapshots", required=True, type=Path)
    revision_input = parser.add_mutually_exclusive_group(required=True)
    revision_input.add_argument("--joint-revisions", type=Path)
    revision_input.add_argument("--live-events", type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--glim-aligned", required=True, type=Path)
    parser.add_argument("--kimera-aligned", required=True, type=Path)
    parser.add_argument("--fused-aligned", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    run_name = args.source_run.name
    if (args.source_run / "resolved_glim_params.yaml").read_text().find(
            "glim_factor_graph_inspector_enable: true") < 0:
        raise ValueError("GLIM real graph inspector was not enabled")
    if (args.source_run / "resolved_kimera_params.yaml").read_text().find(
            "rerun_factor_graph_inspector_enable: true") < 0:
        raise ValueError("Kimera real graph inspector was not enabled")
    states = table(args.receiver / "states.csv")
    grid = [int(row["timestamp_ns"]) for row in states]
    if not grid or grid != sorted(set(grid)):
        raise ValueError("receiver support timestamps are not strictly ordered")
    revisions = (live_revisions(args.live_events, args.receiver.parent / "resolved_config.json")
                 if args.live_events else table(args.joint_revisions))
    events, outcomes = receiver_events(revisions, table(args.receiver / "revision_events.csv"), grid)
    gt_times, gt_xyz, _ = read_tum(args.ground_truth)
    glim_times, _, _ = read_tum(args.source_run / "odom_imu.txt")
    kimera_times, _, _ = read_kimera(args.source_run / "kimera_odometry.csv")
    fused_times, _, _ = read_tum(args.receiver / "trajectory.tum")
    online_times, online_xyz, _ = read_tum(args.receiver / "online_trajectory.tum")
    gt_window = (gt_times >= min(glim_times[0], kimera_times[0], fused_times[0])) & (
        gt_times <= max(glim_times[-1], kimera_times[-1], fused_times[-1]))
    gt_times, gt_xyz = gt_times[gt_window], gt_xyz[gt_window]
    if len(gt_times) < 2:
        raise ValueError("the selected SPX-3 window has insufficient prism ground truth")
    paths = {
        "gt": (gt_times, gt_xyz),
        "glim": (glim_times, aligned(args.glim_aligned, glim_times)),
        "kimera": (kimera_times, aligned(args.kimera_aligned, kimera_times)),
        "fused": (fused_times, aligned(args.fused_aligned, fused_times)),
    }
    if len(fused_times) != len(grid) or np.max(np.abs(fused_times - np.asarray(grid) * 1e-9)) > 2e-6:
        raise ValueError("receiver state and fused trajectory timestamp grids differ")
    if len(online_times) != len(grid) or np.max(np.abs(online_times - np.asarray(grid) * 1e-9)) > 2e-6:
        raise ValueError("receiver online trajectory and support timestamp grids differ")
    recording_id = (args.source_run / "rerun_recording_id.txt").read_text().strip()
    supplement = output.with_name(output.stem + "_receiver_data.rrd")
    blueprint_path = output.with_name(output.stem + "_blueprint.rbl")
    rr.init("cbsms", recording_id=recording_id)
    rr.save(supplement)
    rr.log("spx3/trajectories", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for name, (times, xyz) in paths.items():
        path_history(f"spx3/trajectories/{name}", times, xyz, COLORS[name],
                     0.5 if name == "gt" else 1.0)
    pose_rows = table(args.receiver_snapshots / "pose_snapshots.csv")
    source_kinds = (
        {source["id"]: source["kind"] for source in json.loads(
            (args.receiver.parent / "resolved_config.json").read_text())["sources"]}
        if args.live_events else None)
    graph_summary = draw_receiver_spatial_graph(
        grid, pose_rows,
        table(args.receiver_snapshots / "factor_snapshots.csv"), online_xyz,
        source_kinds, 100_000 if args.live_events else 0)
    if graph_summary["final_active_source_factors"] != outcomes["insert"]:
        raise ValueError("final complete graph does not contain every accepted source factor")
    metadata = dict(line.strip().split("=", 1) for line in
                    (args.source_run / "run_metadata.txt").read_text().splitlines() if "=" in line)
    bag_start = float(metadata["bag_start_sec"])
    duration = float(metadata["sensor_window_sec"])
    live = args.live_events is not None
    note = "\n".join([
        f"# SPX-3 classic CBS-ON, bag seconds {bag_start:g}–{bag_start + duration:g} ({run_name})",
        "",
        f"**Top left/middle:** genuine source estimator inspector recordings from this {duration:g} s run. "
        "They show *active fixed-lag* keys/factors, including each source's received CBS factor. "
        "Kimera's smart-stereo factors may be summarized rather than individually drawn by its inspector.",
        "",
        "**Top right:** gnssFGO's *complete growing spatial graph*, from the receiver's actual "
        "iSAM2-optimized pose states and active GLIM/Kimera factor keys recorded at each support "
        "update. The red marker/arrow is its current robot pose/heading; gray points are "
        "all optimized support poses incorporated so far. Blue Combined IMU connects adjacent X/V/B states, "
        "purple GP-WNOJ connects adjacent X/V/W states, green GLIM joins exact endpoint poses, "
        "and orange Kimera connects the pose supports of both interpolated endpoints "
        "(its V/W keys are validated but co-located visually with those supports). "
        "A fixed receiver-world camera lets the whole history accumulate as playback advances; "
        "every factor/state position comes from the receiver's actual optimized positions. "
        "Colored factor nodes and arms "
        "are not a residual- or information-weight plot. The thin gray context trail is "
        "the full causal as-played path up to the current time. "
        "All historical states and currently active factors are shown; revised factors replace "
        "their earlier versions, rather than accumulating duplicate copies. "
        "This is not a Bayes-tree view.",
        "",
        "**Bottom left:** blue AP20 prism GT, green *native GLIM*, orange *native Kimera ROS "
        "odometry*, red gnssFGO **final retrospectively revised** joint trajectory. "
        "Each is independently no-scale aligned by the existing evaluator. "
        + ("The receiver ran live concurrently with GLIM and Kimera."
           if live else "The receiver replayed recorded causal belief arrival timestamps offline."),
        "",
        "No GNSS factors are present. Source exchange uses classic CBS; receiver uses stable-ID revisions. "
        f"This {duration:g} s graph-logging run is fresh and must not be treated as bit-identical "
        "to the frozen 134 s quantitative experiment.",
    ])
    rr.log("spx3/notes", rr.TextDocument(note, media_type=rr.MediaType.MARKDOWN), static=True)
    rr.disconnect()
    all_xyz = np.concatenate([xyz for _, xyz in paths.values()])
    center = (np.min(all_xyz, axis=0) + np.max(all_xyz, axis=0)) / 2
    extent = max(float(np.max(np.max(all_xyz, axis=0) - np.min(all_xyz, axis=0))), 10.0)
    final_index = max(int(row["snapshot_index"]) for row in pose_rows)
    final_poses = np.asarray([[float(row[field]) for field in ("px", "py", "pz")]
                              for row in pose_rows if int(row["snapshot_index"]) == final_index])
    if len(final_poses) != len(grid):
        raise ValueError("final receiver graph snapshot does not contain full pose history")
    receiver_center = (np.min(final_poses, axis=0) + np.max(final_poses, axis=0)) / 2
    receiver_extent = max(float(np.max(np.max(final_poses, axis=0) -
                                       np.min(final_poses, axis=0))), 10.0)
    make_blueprint(center, extent, receiver_center, receiver_extent).save(
        application_id="cbsms", path=blueprint_path)
    subprocess.run(["rerun", "rrd", "merge", "-o", str(output), str(args.source_rrd),
                    str(supplement), str(blueprint_path)], check=True)
    subprocess.run(["rerun", "rrd", "verify", str(output)], check=True)
    validation = {
        "source_run": str(args.source_run.resolve()),
        "receiver": str(args.receiver.resolve()),
        "source_inspectors": "recorded actual fixed-lag graph snapshots",
        "receiver_view": "complete optimized iSAM2 pose history + all active source-factor keys at every snapshot",
        "receiver_outcomes": outcomes,
        "receiver_graph": graph_summary,
        "path_rows": {name: len(times) for name, (times, _) in paths.items()},
        "inputs_sha256": {str(path.resolve()): digest(path) for path in (
            args.source_rrd, args.live_events or args.joint_revisions,
            args.receiver / "revision_events.csv",
            args.receiver / "states.csv", args.receiver / "trajectory.tum",
            args.receiver / "online_trajectory.tum",
            args.receiver_snapshots / "pose_snapshots.csv",
            args.receiver_snapshots / "factor_snapshots.csv",
            args.glim_aligned, args.kimera_aligned, args.fused_aligned,
        )},
        "output_sha256": digest(output),
    }
    output.with_name(output.stem + "_validation.json").write_text(
        json.dumps(validation, indent=2) + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
