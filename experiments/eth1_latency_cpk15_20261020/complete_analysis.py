"""Additional saved-profile/static-binary evidence; does not load the binaries."""
import csv
import hashlib
import json
import subprocess
from pathlib import Path

from analyze import START, WS, OUT, fields, stats, tracked, inputs

result = {}
for case, rate in (("cpk13", .3), ("cpk14", .2)):
    p = WS / f"runs/gnssfgo_eth1_full_rate{int(rate*100):03d}_{case}_20261020/prepared"
    s = p / "full_off_source"
    profile = [json.loads(l) for l in tracked(s/"e1b_profile.jsonl").open()]
    main = [x for x in profile if x["context"] == 1]
    log = next((s/"ros_logs").glob("*/glim_rosnode-1-stdout.log"))
    scans, keyframes, async_rows = [], [], []
    for line in tracked(log).open(errors="replace"):
        if "GLIM_ODOM_IMU_TIMING_ROW," in line:
            f = fields(line, "GLIM_ODOM_IMU_TIMING_ROW,")
            scans.append([float(x) for x in f[:24]])
        if "GLIM_GPU_TIMING_ROW,create_factors," in line:
            f = fields(line, "GLIM_GPU_TIMING_ROW,create_factors,")
            keyframes.append([float(x) for x in f[:-1]])
        if "GLIM_ASYNC_ODOM_TIMING_ROW," in line:
            f = fields(line, "GLIM_ASYNC_ODOM_TIMING_ROW,")
            if f[-1] == "processed":
                async_rows.append([float(x) for x in f[:-1]])
    bins = {}
    for begin in range(0, 330, 30):
        rows = [x for x in main if max(5,begin) <= x["timestamp"]-START/1e9 < begin+30]
        ss = [x for x in scans if max(5,begin) <= x[0]-START/1e9 < begin+30]
        if not rows or not ss:
            continue
        def vals(stage, i):
            return [x["owner_wall"].get(stage,[0,0,0,0])[i] for x in rows]
        bins[str(begin)] = {
            "main_wall_ms": stats(v/1e6 for v in vals("root",1)),
            "sync_gpu_calls": stats(vals("gpu_sync_linearize",0)),
            "graph_linearize_calls": stats(vals("factor_graph_linearize",0)),
            "main_sync_wall_fraction": sum(vals("gpu_sync_linearize",1))/sum(vals("root",1)),
            "keyframe_targets": stats(x[3] for x in keyframes
                                     if max(5,begin) <= x[0]-START/1e9 < begin+30),
            "scan_mean_ms": sum(x[23] for x in ss)/len(ss),
            "fraction_over_budget": sum(x[23]>100/rate for x in ss)/len(ss),
            "new_factor_count": stats(x[4] for x in ss),
        }
    root_ns = sum(x["owner_wall"]["root"][1] for x in main)
    sync_ns = sum(x["owner_wall"].get("gpu_sync_linearize",[0,0,0,0])[1] for x in main)
    scan_ns = sum(x[23]*1e6 for x in scans)
    result[case] = {
        "main_sync_seconds": sync_ns/1e9, "main_seconds": root_ns/1e9,
        "all_scan_seconds": scan_ns/1e9,
        "main_sync_fraction": sync_ns/root_ns,
        "main_sync_fraction_of_scan": sync_ns/scan_ns,
        "local_context_rows": sum(x["context"]==2 for x in profile),
        "gpu_cache_hits": sum(x["metrics"][7] for x in profile),
        "bins": bins,
        "last20s_before_failure": stats(x[23] for x in scans
            if (116 if case=="cpk13" else 287) <= x[0]-START/1e9 < (136.45 if case=="cpk13" else 307.23)),
    }

recipe = tracked(WS/"runs/e1b4a_local_update_batching_20261012/release_2/libraries_commands.json")
cmd = next(r["command"] for r in json.loads(recipe.read_text())
           if any(v.endswith("/glim_ros.cpp") for v in r["command"]))
assert "-DBUILD_GTSAM_POINTS_GPU" not in cmd
bridge = tracked(WS/"runs/local_revision_covariance_cpi1_20261019/libglim_ros.so")
obj = tracked(WS/"runs/e1b4a_local_update_batching_20261012/release_2/glim_ros.cpp.o")
symbols = {}
for path in (bridge,obj):
    run = subprocess.run(["nm","-C",str(path)],text=True,capture_output=True,check=True)
    symbols[str(path)] = {
        "register_hook": [l for l in run.stdout.splitlines() if "LinearizationHook::register_hook" in l],
        "create_gpu_set": [l for l in run.stdout.splitlines() if "create_nonlinear_factor_set_gpu" in l],
        "glim_ros_constructor": [l for l in run.stdout.splitlines() if "GlimROS::GlimROS(" in l],
    }
    assert not symbols[str(path)]["register_hook"]
    assert not symbols[str(path)]["create_gpu_set"]
result["gpu_registration"] = {"archived_compile_command":cmd, "symbol_evidence":symbols}
check = json.loads((WS/"runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared/full_off_source/effective_mode_check.json").read_text())
result["gpu_registration"]["effective_saved_runtime_gpu_paths"] = {
    k: [l for l in v.splitlines() if "libglim_ros.so" in l or "libgtsam_points" in l]
    for k,v in check.items() if isinstance(v,str) and ("libglim_ros.so" in v or "libgtsam_points" in v)}
result["inputs"] = inputs
target = OUT/"supplement.json"
with target.open("x") as f:
    json.dump(result,f,indent=2)
print(target)
