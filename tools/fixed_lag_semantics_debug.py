#!/usr/bin/env python3
"""Bounded, source-only fixed-lag semantics analysis of retained audit logs."""
import argparse
import csv
import json
import re
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from eig2_single_source_preservation import exp_se3, inverse, log_se3, pose, read_online_csv


def stats(a):
    a = np.asarray(a, dtype=float)
    if not a.size:
        return {"count": 0}
    return dict(count=int(a.size), median=float(np.median(a)),
                rms=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(a, 95)),
                maximum=float(np.max(a)))


def write_rows(path, rows):
    if rows:
        with path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


def norm_error(a, b):
    e = log_se3(inverse(a) @ b)
    return float(np.linalg.norm(e[3:])), float(np.degrees(np.linalg.norm(e[:3])))


def decode(v, w_first=False):
    v = list(map(float, v))
    return pose(v[:3], v[4:7] + [v[3]] if w_first else v[3:7])


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def load_kimera(run, seconds):
    online = read_online_csv(run / "kimera_odometry.csv")
    t0 = online[0][0] * 1e-9
    windows, versions, retired = defaultdict(dict), defaultdict(list), []
    pattern = re.compile(r"KIMERA_CBS_REVISION_(EDGE|POSE_RETIRE)_ROW,([^\x1b\r\n]+)")
    for line in (run / "estimators.log").open(errors="replace"):
        m = pattern.search(line)
        if m is None:
            continue
        v = m[2].split(",")
        u = int(v[0])
        if online[u][0] * 1e-9 > t0 + seconds + 1e-5:
            break
        if m[1] == "POSE_RETIRE":
            retired.append((u, int(v[1])))
            continue
        i, j = int(v[2]), int(v[3])
        a, b = decode(v[8:15], True), decode(v[15:22], True)
        for index, value in ((i, a), (j, b)):
            if index in windows[u] and np.max(np.abs(windows[u][index] - value)) > 1e-11:
                raise ValueError("inconsistent same-snapshot endpoint")
            windows[u][index] = value
        versions[(i, j)].append((u, a, b))
    times = {u: x[0]*1e-9 for u, x in online.items() if x[0]*1e-9 <= t0+seconds+1e-5}
    windows[0][0] = online[0][1]
    # Keep missing logged states explicit: never silently carry them forward.
    missing = {u: sorted(set(range(max(0, u-25), u+1))-set(windows[u])) for u in times}
    missing = {u: ids for u, ids in missing.items() if ids}
    return windows, times, online, versions, retired, missing


def load_semantics(path, seconds):
    windows, times, outputs = defaultdict(dict), {}, {}
    pattern = re.compile(r"KIMERA_SEMANTICS_(ACTIVE|OUTPUT)_ROW,([^\x1b\r\n]+)")
    for line in path.open(errors="replace"):
        m = pattern.search(line)
        if not m:
            continue
        v = m[2].split(",")
        u = int(v[0])
        if m[1] == "ACTIVE":
            windows[u][int(v[1])] = decode(v[3:10], True)
        else:
            times[u] = float(v[1])
            outputs[u] = decode(v[2:9], True)
    t0 = times[min(times)]
    keep = {u for u in times if times[u] <= t0+seconds+1e-5}
    return ({u: windows[u] for u in sorted(keep)}, {u: times[u] for u in keep},
            {u: outputs[u] for u in keep})


def marginalization_events(path, max_update):
    stages, retirements = [], []
    pattern = re.compile(r"CBS_FIXED_LAG_(STAGE|MARGINALIZATION)_ROW,([^\x1b\r\n]+)")
    for line in path.open(errors="replace"):
        match = pattern.search(line)
        if not match:
            continue
        v = match[2].split(",")
        if float(v[1]) > max_update:
            continue
        if match[1] == "MARGINALIZATION" and int(v[2]):
            retirements.append(dict(update=int(float(v[1])), total_keys=int(v[2]),
                                    pose_keys=int(v[3]), prior_factors=int(v[6])))
        elif match[1] == "STAGE" and v[2] == "marginalize_leaves":
            stages.append(dict(update=int(float(v[1])), translation_max_m=float(v[5]),
                               rotation_max_deg=float(v[7])))
    return dict(first=retirements[0] if retirements else None,
                nonempty_events=len(retirements),
                immediate_translation_max_m=max([r["translation_max_m"] for r in stages], default=0),
                immediate_rotation_max_deg=max([r["rotation_max_deg"] for r in stages], default=0))


def load_glim(run, seconds):
    windows, times, first_snapshots = defaultdict(dict), {}, {}
    pattern = re.compile(r"GLIM_CBS_(ACTIVE_POSE|EXPORT_ENDPOINT)_ROW,([^\x1b\r\n]+)")
    t0 = None
    for line in (run / "estimators.log").open(errors="replace"):
        m = pattern.search(line)
        if not m:
            continue
        v = m[2].split(",")
        u = int(v[1])
        if m[1] == "ACTIVE_POSE":
            i, stamp = int(v[3]), float(v[4])
            if t0 is None:
                t0 = stamp
            if stamp > t0+seconds+1e-5:
                # Stop after completing the previous update.
                windows.pop(u, None)
                times.pop(u, None)
                break
            windows[u][i] = decode(v[5:12])
            times[u] = max(times.get(u, stamp), stamp)
        else:
            first_snapshots.setdefault((int(v[3]), int(v[4])),
                                       (u, decode(v[7:14]), decode(v[14:21])))
    return windows, times, first_snapshots


def logs_batch(rotations, translations):
    phi = Rotation.from_matrix(rotations).as_rotvec()
    theta = np.linalg.norm(phi, axis=1)
    coefficient = np.full(len(phi), 1/12.)
    large = theta > 1e-5
    a = theta[large]
    coefficient[large] = (1-0.5*a/np.tan(0.5*a))/(a*a)
    cross = np.cross(phi, translations)
    rho = translations - 0.5*cross + coefficient[:, None]*np.cross(phi, cross)
    return np.c_[phi, rho]


def fit_window(before, after):
    x, y = np.asarray(before), np.asarray(after)
    xr, yr, xt, yt = x[:, :3, :3], y[:, :3, :3], x[:, :3, 3], y[:, :3, 3]
    initial = np.eye(4)
    initial[:3, :3] = Rotation.from_matrix(yr @ xr.transpose(0, 2, 1)).mean().as_matrix()
    initial[:3, 3] = np.mean(yt - xt @ initial[:3, :3].T, axis=0)

    def residual(g):
        pr = g[:3, :3] @ xr
        pt = xt @ g[:3, :3].T + g[:3, 3]
        return logs_batch(pr.transpose(0, 2, 1) @ yr,
                          np.einsum("nij,nj->ni", pr.transpose(0, 2, 1), yt-pt))
    solved = least_squares(lambda v: residual(exp_se3(v) @ initial).ravel(),
                           np.zeros(6), method="lm", ftol=1e-12, xtol=1e-12,
                           gtol=1e-12, max_nfev=150)
    if not solved.success:
        raise ValueError("window fit did not converge")
    g = exp_se3(solved.x) @ initial
    raw, aligned = residual(np.eye(4)), residual(g)
    if np.sum(aligned**2) > np.sum(raw**2)+1e-10:
        raise ValueError("fit increases objective")
    return g, raw, aligned


def summarize_window(name, windows, times, outputs, out):
    ordered = sorted(windows)
    rows, pose_rows, gaps = [], [], []
    first_retirement = None
    for previous, current in zip(ordered[:-1], ordered[1:]):
        common = sorted(set(windows[previous]) & set(windows[current]))
        lost = sorted(set(windows[previous])-set(windows[current]))
        if lost and first_retirement is None:
            first_retirement = dict(update=current, time=times[current], lost=lost)
        if not common:
            continue
        g, raw, aligned = fit_window([windows[previous][i] for i in common],
                                     [windows[current][i] for i in common])
        centroid = np.mean([windows[previous][i][:3, 3] for i in common], axis=0)
        fields = dict(update=current, time=times[current], elapsed=times[current]-times[ordered[0]],
                      common_poses=len(common), active_poses=len(windows[current]),
                      lost_ids=";".join(map(str, lost)),
                      common_translation_m=float(np.linalg.norm(g[:3, 3])),
                      common_rotation_deg=float(np.degrees(Rotation.from_matrix(g[:3, :3]).magnitude())),
                      common_centroid_displacement_m=float(np.linalg.norm(g[:3,:3]@centroid+g[:3,3]-centroid)),
                      raw_translation_rms_m=float(np.sqrt(np.mean(np.sum(raw[:, 3:]**2, axis=1)))),
                      raw_rotation_rms_deg=float(np.degrees(np.sqrt(np.mean(np.sum(raw[:, :3]**2, axis=1))))),
                      residual_translation_rms_m=float(np.sqrt(np.mean(np.sum(aligned[:, 3:]**2, axis=1)))),
                      residual_rotation_rms_deg=float(np.degrees(np.sqrt(np.mean(np.sum(aligned[:, :3]**2, axis=1))))),
                      raw_energy=float(np.sum(raw**2)), residual_energy=float(np.sum(aligned**2)))
        rows.append(fields)
        for n, i in enumerate(common):
            pose_rows.append(dict(update=current, index=i, time=times[current],
                                  raw_translation_m=np.linalg.norm(raw[n, 3:]),
                                  raw_rotation_deg=np.degrees(np.linalg.norm(raw[n, :3])),
                                  residual_translation_m=np.linalg.norm(aligned[n, 3:]),
                                  residual_rotation_deg=np.degrees(np.linalg.norm(aligned[n, :3]))))
    for u in ordered:
        if u in outputs and u in windows[u]:
            t, r = norm_error(windows[u][u], outputs[u])
            gaps.append(dict(update=u, time=times[u], elapsed=times[u]-times[ordered[0]],
                             translation_m=t, rotation_deg=r))
    write_rows(out/f"{name}_window.csv", rows)
    write_rows(out/f"{name}_pose_movements.csv", pose_rows)
    write_rows(out/f"{name}_state_increment_gap.csv", gaps)
    summary = dict(updates=len(ordered), first_retirement=first_retirement,
                   raw_and_residual={key: stats([r[key] for r in pose_rows])
                                     for key in ("raw_translation_m", "raw_rotation_deg",
                                                 "residual_translation_m", "residual_rotation_deg")},
                   common_transform={key: stats([r[key] for r in rows])
                                     for key in ("common_translation_m", "common_rotation_deg", "common_centroid_displacement_m")},
                   fitted_energy_fraction=1-sum(r["residual_energy"] for r in rows)/
                                          max(sum(r["raw_energy"] for r in rows), 1e-30),
                   state_increment_gap={key: stats([r[key] for r in gaps])
                                        for key in ("translation_m", "rotation_deg")})
    if first_retirement:
        for phase, selected in (
                ("before_retirement", [r for r in rows if r["update"] < first_retirement["update"]]),
                ("after_retirement", [r for r in rows if r["update"] >= first_retirement["update"]])):
            summary[phase] = {key: stats([r[key] for r in selected]) for key in
                              ("common_translation_m", "common_rotation_deg",
                               "residual_translation_rms_m", "residual_rotation_rms_deg")}
    return summary, rows, gaps


def local_revisions(products, reference, t0, seconds, name, out, start=0.):
    grouped = defaultdict(list)
    for r in read_csv(products/"revisions.csv"):
        if t0+start <= float(r["to_timestamp_sec"]) <= t0+seconds+1e-5:
            grouped[(int(r["from_index"]), int(r["to_index"]))].append(r)
    rt, rp, rq = reference["times"], reference["positions"], reference["quaternions"]
    def ref(t):
        j = np.searchsorted(rt, t)
        if j == 0 or j == len(rt) or rt[j]-rt[j-1] > 0.05:
            return None
        a = (t-rt[j-1])/(rt[j]-rt[j-1])
        q = Slerp(rt[j-1:j+1], Rotation.from_quat(rq[j-1:j+1]))([t]).as_quat()[0]
        return pose((1-a)*rp[j-1]+a*rp[j], q)
    errors, changes = [], []
    for (i, j), versions in sorted(grouped.items()):
        versions.sort(key=lambda r: int(r["revision"]))
        ti, tj = float(versions[0]["from_timestamp_sec"]), float(versions[0]["to_timestamp_sec"])
        ri, rj = ref(ti), ref(tj)
        if ri is None or rj is None:
            continue
        zref = inverse(ri) @ rj
        ages = np.array([int(r["record_timestamp_ns"]) for r in versions], dtype=np.int64)
        duration = ages[-1]-ages[0]
        native0 = exp_se3(np.array([float(versions[0][f"relative_mu_{k}"]) for k in range(6)]))
        for fraction in (0., 0.25, 0.5, 0.75, 1.):
            index = int(np.argmin(np.abs((ages-ages[0])-duration*fraction)))
            r = versions[index]
            z = exp_se3(np.array([float(r[f"relative_mu_{k}"]) for k in range(6)]))
            delta = log_se3(inverse(zref) @ z)
            errors.append(dict(i=i, j=j, ti=ti, tj=tj, fraction=fraction,
                               revision=int(r["revision"]), age_sec=(ages[index]-ages[0])/1e9,
                               translation_m=np.linalg.norm(z[:3, 3]-zref[:3, 3]),
                               log_rho_m=np.linalg.norm(delta[3:]),
                               rotation_deg=np.degrees(np.linalg.norm(delta[:3]))))
        zlast = exp_se3(np.array([float(versions[-1][f"relative_mu_{k}"]) for k in range(6)]))
        dt, dr = norm_error(native0, zlast)
        changes.append(dict(i=i, j=j, elapsed=tj-t0, translation_m=dt, rotation_deg=dr))
    write_rows(out/f"{name}_local_revision_errors.csv", errors)
    write_rows(out/f"{name}_first_last_change.csv", changes)
    return {str(f): {key: stats([r[key] for r in errors if r["fraction"] == f])
                     for key in ("translation_m", "rotation_deg", "age_sec")}
            for f in (0., 0.25, 0.5, 0.75, 1.)}, changes


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--kimera-run", type=Path, required=True)
    p.add_argument("--glim-run", type=Path, required=True)
    p.add_argument("--reference", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=25.)
    p.add_argument("--edge-seconds", type=float, default=20.)
    p.add_argument("--start-sec", type=float, default=0.)
    p.add_argument("--glim-online", type=Path)
    p.add_argument("--baseline-log", type=Path)
    p.add_argument("--ablation-log", type=Path)
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    # Independent known rigid-motion recovery validates the numerical fit.
    x = [exp_se3(np.array([.01*i, -.03*i, .05*i, i*.1, i*.03, -.2])) for i in range(8)]
    truth = exp_se3(np.array([.04, -.02, .06, .1, -.3, .2]))
    fitted, _, residual = fit_window(x, [truth @ t for t in x])
    assert np.max(np.abs(fitted-truth)) < 1e-10 and np.max(np.abs(residual)) < 1e-10
    horizon = args.start_sec + args.seconds
    kw, kt, ko, kv, retired, missing = load_kimera(args.kimera_run, horizon)
    t0 = ko[0][0]*1e-9
    kw = {u: w for u, w in kw.items() if kt[u] >= t0+args.start_sec}
    missing = {u: ids for u, ids in missing.items() if u in kw}
    h1 = []
    for row in read_csv(args.kimera_run/"revision_products/first.csv"):
        i, j = int(row["from_index"]), int(row["to_index"])
        if not t0+args.start_sec <= ko[j][0]*1e-9 <= t0+horizon+1e-5:
            continue
        z = exp_se3(np.array([float(row[f"relative_mu_{k}"]) for k in range(6)]))
        dt, dr = norm_error(z, inverse(ko[i][1]) @ ko[j][1])
        h1.append(dict(i=i, j=j, translation_m=dt, rotation_deg=dr))
    write_rows(args.out/"kimera_increment_vs_first.csv", h1)
    output = dict(h1={key: stats([r[key] for r in h1]) for key in ("translation_m", "rotation_deg")},
                  recorded_snapshot_missing_states=missing,
                  recorded_first_retirement=retired[0])
    if args.baseline_log:
        bw, bt, bo = load_semantics(args.baseline_log, args.seconds)
        checks = [norm_error(kw[u][i], bw[u][i]) for u in bw if u in kw
                  for i in kw[u] if i in bw[u]]
        output["baseline_vs_recorded_common_snapshots"] = {
            "translation_m": stats([v[0] for v in checks]),
            "rotation_deg": stats([v[1] for v in checks])}
        output["baseline_vs_recorded_output_max_matrix_difference"] = max(
            float(np.max(np.abs(bo[u]-ko[u][1]))) for u in bo)
        kw, kt = bw, bt
        output["baseline_marginalization"] = marginalization_events(args.baseline_log, max(bw))
    elif missing:
        output["window_population_warning"] = "Recorded endpoint union omits states; see missing map."
    output["kimera"], kr, kg = summarize_window("kimera", kw, kt, {u: v[1] for u,v in ko.items()}, args.out)
    gw, gt, first = load_glim(args.glim_run, horizon+1)
    gw = {u: w for u, w in gw.items() if t0+args.start_sec <= gt[u] <= t0+horizon}
    output["glim"], gr, _ = summarize_window("glim", gw, gt, {}, args.out)
    if args.glim_online:
        online = np.load(args.glim_online)["poses"]
        pairs, increments = [], []
        poses_by_id = {}
        for u, w in gw.items():
            j = int(np.argmin(np.abs(online[:, 0]-gt[u])))
            if abs(online[j, 0]-gt[u]) > 1e-6:
                raise ValueError("GLIM online timestamp unmatched")
            value = decode(online[j, 1:8])
            poses_by_id[max(w)] = value
            a, b = norm_error(w[max(w)], value)
            pairs.append(dict(update=u, translation_m=a, rotation_deg=b))
        for (i, j), (u, a, b) in first.items():
            if i in poses_by_id and j in poses_by_id:
                dt, dr = norm_error(inverse(a)@b, inverse(poses_by_id[i])@poses_by_id[j])
                increments.append(dict(i=i, j=j, translation_m=dt, rotation_deg=dr))
        for name, values in (("glim_online_vs_latest_state", pairs),
                             ("glim_online_increment_vs_first", increments)):
            write_rows(args.out/(name+".csv"), values)
            output[name] = {k: stats([r[k] for r in values]) for k in ("translation_m", "rotation_deg")}
    reference = np.load(args.reference)
    output["kimera_local_revision"], changes = local_revisions(
        args.kimera_run/"revision_products", reference, t0, args.start_sec+args.edge_seconds, "kimera", args.out, args.start_sec)
    output["glim_local_revision"], _ = local_revisions(
        args.glim_run/"revision_products", reference, t0, args.start_sec+args.edge_seconds, "glim", args.out, args.start_sec)
    for name in ("kimera", "glim"):
        rows = read_csv(args.out/f"{name}_first_last_change.csv")
        output[name+"_first_last_change"] = {k: stats([r[k] for r in rows])
                                               for k in ("translation_m", "rotation_deg")}
    if args.ablation_log:
        aw, at, ao = load_semantics(args.ablation_log, args.seconds)
        output["ablation_marginalization"] = marginalization_events(args.ablation_log, max(aw))
        output["ablation"], ar, ag = summarize_window("ablation", aw, at, ao, args.out)
        matching = [u for u in aw if u in kw and u < 26]
        errors = [norm_error(kw[u][i], aw[u][i]) for u in matching
                  for i in set(kw[u]) & set(aw[u])]
        output["ablation_pre_marginalization_equality"] = {
            "translation_m": stats([v[0] for v in errors]), "rotation_deg": stats([v[1] for v in errors])}
        subset = {u: {i: aw[u][i] for i in kw[u]} for u in kw if u in aw}
        output["ablation_matched_population"], _, _ = summarize_window(
            "ablation_matched_population", subset, at, ao, args.out)
        comparisons = []
        for u in sorted(subset):
            d = [norm_error(kw[u][i], aw[u][i]) for i in kw[u]]
            comparisons.append(dict(update=u, time=kt[u],
                                    max_position_difference_m=max(v[0] for v in d),
                                    max_rotation_difference_deg=max(v[1] for v in d)))
        write_rows(args.out/"lag_control_same_pose_comparison.csv", comparisons)
        output["ablation_first_difference_1e_9"] = next((r for r in comparisons if
            r["max_position_difference_m"] > 1e-9 or r["max_rotation_difference_deg"] > 1e-9), None)
    fig, axs = plt.subplots(4, 2, figsize=(12, 11), sharex=True)
    for row, keys in enumerate((("common_translation_m", "common_rotation_deg"),
                                ("residual_translation_rms_m", "residual_rotation_rms_deg"))):
        for col, key in enumerate(keys):
            axs[row,col].plot([r["elapsed"] for r in kr], [r[key] for r in kr], label="Kimera lag 25")
            if args.ablation_log:
                axs[row,col].plot([r["elapsed"] for r in ar], [r[key] for r in ar], label="No retirement")
            axs[row,col].set_ylabel(key.replace("_", " "))
    for col,key in enumerate(("translation_m", "rotation_deg")):
        axs[2,col].plot([r["elapsed"] for r in kg], [r[key] for r in kg], label="State vs increment")
        if args.ablation_log:
            axs[2,col].plot([r["elapsed"] for r in ag], [r[key] for r in ag], label="No retirement")
        axs[2,col].set_ylabel("State/increment " + key)
        axs[3,col].plot([r["elapsed"] for r in changes], [r[key] for r in changes])
        axs[3,col].set_ylabel("First-to-last edge " + key)
        axs[3,col].set_xlabel("Seconds since Kimera initialization")
    onset = ko[retired[0][0]][0]*1e-9-t0
    for ax in axs.flat:
        ax.axvline(onset, color="k", linestyle="--", alpha=.5)
        ax.grid(alpha=.25)
    axs[0,0].legend()
    fig.tight_layout()
    if args.start_sec == 0:
        fig.savefig(args.out/"kimera_marginalization_onset.png", dpi=160)
    plt.close(fig)
    (args.out/"summary.json").write_text(json.dumps(output, indent=2)+"\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
