"""Record failed-capture closeout without running any estimator or validation."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import subprocess
import tomllib

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared")
V = Path("/home/yeranis/results/gnssfgo_eth1_full_rate020_cpk14_20261020")

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

evidence = json.loads((H / "failure_evidence.json").read_text())
assert not evidence["capture_passed"] and not evidence["on_started"]
assert not V.exists()
assert not (P / "full_off_control/complete.json").exists()
assert not any((P / f"full_on_{part}").exists() for part in ("control", "source", "receiver"))
states = {name: subprocess.check_output(
    ["docker", "inspect", "--format", "{{.State.Status}}", name], text=True).strip()
    for name in ("cbsms_ws_gpu", "ubuntu-jammy-humble-nvidia")}
assert all(s == "exited" for s in states.values()), states
processes = subprocess.check_output(["ps", "-eo", "pid,args"], text=True)
owned = [line for line in processes.splitlines() if str(P) in line and any(
    name in line for name in ("run_case.py", "run_case_on.py", "validate_case.py",
                             "validate_local.py", "validate_candidate.py",
                             "launch_grandtour_live_receiver.py"))]
assert not owned, owned
automation = tomllib.loads(Path("/home/yeranis/.codex/automations/complete-frozen-eth1-live-wnoa-comparison/automation.toml").read_text())
assert automation["status"] == "PAUSED"
manifest = json.loads((P / "protected_manifest.json").read_text())
for path, expected in manifest.items():
    assert sha(path) == expected, path
subprocess.run(["git", "-C", "/home/yeranis/repos/gnssFGO", "diff", "--check"], check=True)
result = dict(capture_passed=False, cleanup_verified=True, on_started=False,
    closed_utc=datetime.now(timezone.utc).isoformat(), protected_unchanged=len(manifest),
    container_states=states, heartbeat_status=automation["status"],
    report_sha256=sha(H / "REPORT.txt"),
    failure_evidence_sha256=sha(H / "failure_evidence.json"),
    note="Stopped on failure; no source retry, journal replay, scoring, tuning or next experiment.")
with (H / "failure_closeout.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
