"""Matched native-output/LOCAL-export comparison, never aligns trajectories to GT."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from validate_k16d import P, load, stats

a, ta, xa, ba = load("reference")
b, tb, xb, bb = load("batched")
assert np.array_equal(ta, tb), "native timestamp/count mismatch"
assert ba.keys() == bb.keys(), "source ID/revision topology mismatch"
mean, cov, relative, ratio = [], [], [], []
for key in ba:
    l, r = ba[key], bb[key]
    assert (l["ti"], l["tj"]) == (r["ti"], r["tj"]), key
    if key[2]:
        assert l["source_update"] == r["source_update"], key
    lm, rm = np.asarray(l["mean"]), np.asarray(r["mean"])
    lc, rc = np.asarray(l["covariance"]), np.asarray(r["covariance"])
    mean.append(float(np.max(np.abs(lm-rm))))
    delta = float(np.max(np.abs(lc-rc)))
    cov.append(delta)
    relative.append(float(np.linalg.norm(lc-rc) / np.linalg.norm(lc)))
    ratio.append(delta / (1e-10 + 1e-6*np.max(np.abs(lc))))
dp = np.linalg.norm(xa[:, :3]-xb[:, :3], axis=1)
dr = (Rotation.from_quat(xa[:, 3:]).inv() * Rotation.from_quat(xb[:, 3:])).magnitude()
timing_ratios = {
    bucket: {metric: {q: b["timing"][bucket][metric][q] / a["timing"][bucket][metric][q]
                      for q in ("median", "p95", "max")}
             for metric in ("scan_ms", "main_ms", "post_callback_ms", "export_ms")}
    for bucket in a["timing"]}
params_a = json.loads((P / "reference/effective_params.json").read_text())
params_b = json.loads((P / "batched/effective_params.json").read_text())
param_diffs = {k: [params_a[k], params_b[k]] for k in params_a if params_a[k] != params_b[k]}
assert set(param_diffs) == {"glim_dump_path", "glim_factor_graph_inspector_recording_id"}
work_same = a["profile_main_work"]["gpu_linearize"] == b["profile_main_work"]["gpu_linearize"]
result = dict(
    structural_passed=True, matched_native_poses=len(ta), matched_belief_versions=len(ba),
    original=a, batched=b, native_position_difference_m=stats(dp),
    native_orientation_difference_rad=stats(dr), belief_mean_maxabs_difference=stats(mean),
    covariance_maxabs_difference=stats(cov), covariance_relative_frobenius_difference=stats(relative),
    covariance_inherited_bound_ratio_max=max(ratio), same_derivative_count=work_same,
    same_input_fixture_bounds_also_met_on_this_capture=bool(max(dp) <= 1e-9 and max(dr) <= 1e-10 and max(ratio) <= 1),
    timing_candidate_over_original=timing_ratios, output_only_param_diffs=param_diffs,
    lower_whole_scan_median_and_p95=all(timing_ratios["whole"]["scan_ms"][q] < 1 for q in ("median", "p95")),
    input_manifest_sha256=hashlib.sha256((P / "input_manifest.json").read_bytes()).hexdigest(),
    limitations="One matched source-only pair, no live Kimera/fuser; no 1x/full-sequence/retention certificate. "
                "Live preprocessing and scheduling are not an exact frozen-GPU-state oracle.")
with (P / "comparison.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps({k: v for k, v in result.items() if k not in ("original", "batched")}, indent=2))
