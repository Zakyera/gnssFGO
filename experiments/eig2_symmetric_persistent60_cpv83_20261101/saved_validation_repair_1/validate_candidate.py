"""CP-V4 source lifecycle gate; no nonlinear-stationarity claim."""
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_symmetric_persistent60_cpv83_20261101/prepared')
mode = sys.argv[1]
assert mode in ("off", "on")
S = R / f"full_{mode}_source"
assert (R / f"full_{mode}_control/complete.json").exists()
assert not (R / f"full_{mode}_control/failed.txt").exists()
assert not (Path('/home/yeranis/repos/gnssFGO/experiments/eig2_symmetric_persistent60_cpv83_20261101/saved_validation_repair_1') / "candidate_validation.json").exists()
assert not (S / "acceptance.csv").exists(), "obsolete FL4q executed"
with (S / "objective.csv").open() as f:
    rows = [[float(x) for x in row] for row in csv.reader(f)]
with (S / "kimera_odometry.csv").open() as f:
    poses = list(csv.DictReader(f))
coverage = json.loads((S / 'source_coverage.json').read_text())
times = [int(p['field.header.stamp']) for p in poses]
epoch = 1730638816563546034
assert len(times)>280 and 0<=(times[0]-epoch)*1e-9<=1
assert 59<=(times[-1]-epoch)*1e-9<=60.1
assert max(b-a for a,b in zip(times,times[1:]))<=1_100_000_000
assert coverage['passed'] and coverage['coverage_contract'] == 'native_keyframes_cpk22'
assert coverage['kimera_processing']['passed']
assert len(rows) == len(poses) == coverage['kimera_processing']['backend_processed'] and len(rows) > 10
assert all(len(r) == 10 and all(map(math.isfinite, r)) for r in rows)
assert [r[0] for r in rows] == list(range(len(rows)))
assert all(r[1] == 0 for r in rows), "not ordinary incremental mode"
import re
node_logs = list(S.glob("ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log"))
assert len(node_logs) == 1, node_logs
launcher_log = (S / "estimators.log").read_text()
assert "CPK9_PRIOR_REFRESH," not in launcher_log
log = launcher_log + "\n" + re.sub(r"\x1b\[[0-9;]*m", "", node_logs[0].read_text())
glim_logs=list(S.glob("ros_logs/*/glim_rosnode-*-stdout.log"))
assert len(glim_logs)==1,glim_logs
log += "\n" + re.sub(r"\x1b\[[0-9;]*m", "", glim_logs[0].read_text())
for token in ("FL4Q", "CPK9 ", "CPV40 ", "CPV76 ", "active_window_rollback",
              "active_window_retry_local_update_", "Smoother recovery",
              "Persistent BPSAM recovery", "process has died", "terminate called",
              "Segmentation fault", "rollback_and_retry_without_staged_cbs",
              "rollback_remove_active_and_retry", "CPV40_FATAL", "CPV73_READINESS_FAILURE"):
    assert token not in log, token
effective = json.loads((S / "effective_mode_check.json").read_text())
assert effective["passed"]
assert effective["kimera"]["cbs_experimental_native_imu_mean_alignment"] == "true"
assert effective["kimera_candidate_env"] == {
    "CPK9_PRIOR_REFRESH": "1", "FL4Q_ENABLE": "0", "FL4M_RETIREMENT": "remove", "CPV40_REFRESH": "1", "CPV38_OFF_REFRESH": "0", "CPV76_PERSISTENT_COMPAT": "1"}
want = "b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8"
assert hashlib.sha256((R / "private_build/libkimera_vio.so").read_bytes()).hexdigest() == want
meta = (S / "run_metadata.txt").read_text()
counts = {k: int(next(line.split("=", 1)[1] for line in meta.splitlines()
                      if line.startswith(k + "=")))
          for k in ("g2k_injected", "k2g_injected")}
assert all(v == 0 if mode == "off" else v > 0 for v in counts.values())
promotions = []
for line in log.splitlines():
    if "CPK9_PRIOR_REFRESH," not in line:
        continue
    fields = line.split("CPK9_PRIOR_REFRESH,", 1)[1].split(",")
    assert len(fields) == 5
    t, retired, priors, released, error = map(float, fields)
    assert all(map(math.isfinite, (t, retired, priors, released, error)))
    assert retired > 0 and priors > 0 and released > 0 and 0 <= error <= 1e-8
    promotions.append(dict(time=t, retired=int(retired), priors=int(priors),
                           released_keys=int(released), cache_error=error))
retirements = int(sum(r[2] for r in rows))
assert sum(p["retired"] for p in promotions) == retirements
by_update = {int(p["time"]): p["retired"] for p in promotions}
assert len(by_update) == len(promotions)
assert all(by_update.get(int(r[0]), 0) == r[2] for r in rows)
if mode == "off":
    assert not promotions and retirements == 0
else:
    assert not promotions and retirements == 0, "general persistent must not perform temporary retirement"
    assert effective["kimera"]["cbs_odom_factor_mode"] == "persistent"
    assert effective["glim"]["glim_cbs_mode"] == "inject_persistent"
    assert effective["glim_persistent_compat"] == "1"
    assert "injected_persistent" in log
    assert "active_window_temporary_graph_queued" not in log
    assert "GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,removed" not in log
    assert "persistent_odom_applied" in log
    assert "active_window_temporary_odom_applied" not in log
    assert effective["mixed_prior_feedback_user_accepted"] and not effective["kimera_native_only_outgoing_information"]
result = dict(passed=True, mode=mode, native_updates=len(rows),
    source_injections=counts, planned_retirements=retirements,
    prior_refresh_updates=len(promotions),
    max_cache_error=max((p["cache_error"] for p in promotions), default=0),
    observed_total_cost_increases=sum(r[5] > r[4] + 1e-10 for r in rows),
    imu_factor_cost=dict(median=statistics.median(r[7] for r in rows),
                        max=max(r[7] for r in rows), last=rows[-1][7]),
    stationarity_certified=False, obsolete_FL4q_executed=False,
    candidate_sha256=want,
    kimera_native_only_outgoing_information=False, mixed_prior_feedback_user_accepted=True,
    note="CP76 persistent ownership/refresh guards active; zero temporary retirements. No explicit absorption count telemetry; no stationarity or native-only information claim.")
with (Path('/home/yeranis/repos/gnssFGO/experiments/eig2_symmetric_persistent60_cpv83_20261101/saved_validation_repair_1') / "candidate_validation.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
