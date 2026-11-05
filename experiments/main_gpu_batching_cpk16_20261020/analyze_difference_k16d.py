"""Failure analysis, not a replacement/relaxation of the matched acceptance gate."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from validate_k16d import P, load, stats
a, ta, xa, ba = load("reference")
b, tb, xb, bb = load("batched")
assert np.array_equal(ta, tb)
both = sorted(ba.keys() & bb.keys())
mean_d, cov_d, cov_rel = [], [], []
for key in both:
    l, r = ba[key], bb[key]
    assert (l["ti"], l["tj"]) == (r["ti"], r["tj"])
    lm, rm = np.array(l["mean"]), np.array(r["mean"])
    lc, rc = np.array(l["covariance"]), np.array(r["covariance"])
    mean_d.append(np.max(np.abs(lm-rm)))
    cov_d.append(np.max(np.abs(lc-rc)))
    cov_rel.append(np.linalg.norm(lc-rc) / np.linalg.norm(lc))
dp = np.linalg.norm(xa[:, :3]-xb[:, :3], axis=1)
dr = (Rotation.from_quat(xa[:, 3:]).inv()*Rotation.from_quat(xb[:, 3:])).magnitude()
first = next(i for i in range(len(dp)) if dp[i] > 1e-9 or dr[i] > 1e-10)
counts = {}
for mode in ("reference", "batched"):
    rows = []
    for line in (P / mode / "node.log").open():
        if "GLIM_ODOM_IMU_TIMING_ROW," in line:
            f = line.split("GLIM_ODOM_IMU_TIMING_ROW,", 1)[1].strip().split(",")
            rows.append([int(v) for v in f[1:7]])
    counts[mode] = rows
count_comparison = {}
for col, name in enumerate(("frame_id", "preprocessed_points", "imu_count",
                           "new_factors", "active_frames", "marginalized_frames")):
    differing = [(x[0], x[col], y[col]) for x, y in zip(counts["reference"], counts["batched"])
                 if x[col] != y[col]]
    count_comparison[name] = dict(different_updates=len(differing), first_examples=differing[:5])
ratios = {
    bucket: {metric: {q: b["timing"][bucket][metric][q]/a["timing"][bucket][metric][q]
                      for q in ("median", "p95", "max")}
             for metric in ("scan_ms", "main_ms", "export_ms")}
    for bucket in a["timing"]}
result = dict(
    status="REAL_SOURCE_PRESERVATION_NOT_VALIDATED",
    acceptance_passed=False,
    failure="source ID/revision topology mismatch; native poses and LOCAL covariance differ",
    original=a, batched=b, matched_native_timestamps=len(ta),
    original_belief_versions=len(ba), batched_belief_versions=len(bb), common_versions=len(both),
    reference_only_versions=[list(k) for k in sorted(ba.keys()-bb.keys())],
    batched_only_versions=[list(k) for k in sorted(bb.keys()-ba.keys())],
    initialization_pose_exact=bool(np.array_equal(xa[0], xb[0])),
    first_native_difference=dict(frame=first, timestamp_ns=int(ta[first]),
                                 position_m=float(dp[first]), rotation_rad=float(dr[first])),
    native_position_difference_m=stats(dp), native_orientation_difference_rad=stats(dr),
    common_belief_mean_maxabs_difference=stats(mean_d),
    common_covariance_maxabs_difference=stats(cov_d),
    common_covariance_relative_frobenius_difference=stats(cov_rel),
    source_timing_metadata_comparison=count_comparison,
    timing_candidate_over_original=ratios,
    limitations="Fresh runs, not frozen post-preprocessing GPU state. This evidence does not distinguish "
    "a candidate error from live preprocessing/GPU-state nondeterminism. Do not dismiss discrepancies "
    "as harmless or certify mathematical preservation from speed/finite outputs. No automatic retry.")
with (P / "difference_analysis.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps({k: v for k, v in result.items() if k not in
                 ("original", "batched", "reference_only_versions", "batched_only_versions")}, indent=2))
