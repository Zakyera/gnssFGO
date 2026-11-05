"""Close the failed CP-K13 run without converting it into a passed experiment."""
from pathlib import Path
import hashlib
import json
import subprocess
import datetime

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_full_rate030_cpk13_20261020/prepared")
def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()
evidence = json.loads((H / "failure_evidence.json").read_text())
assert evidence["capture_passed"] is False
assert (P / "full_off_control/failed.txt").exists()
assert not (P / "full_off_control/complete.json").exists()
assert not any((P / f"full_on_{part}").exists() for part in ("control", "source", "receiver"))
assert not (H / "results.json").exists() and not (H / "final_checks.json").exists()
manifest = json.loads((P / "protected_manifest.json").read_text())
for p, expected in manifest.items():
    assert sha(p) == expected, p
states = {name: subprocess.check_output(
    ["docker", "inspect", "--format", "{{.State.Status}}", name], text=True).strip()
    for name in ("cbsms_ws_gpu", "ubuntu-jammy-humble-nvidia")}
assert all(s == "exited" for s in states.values())
subprocess.run(["git", "-C", "/home/yeranis/repos/gnssFGO", "diff", "--check"], check=True)
import tomllib
automation = tomllib.loads(Path("/home/yeranis/.codex/automations/complete-frozen-eth1-live-wnoa-comparison/automation.toml").read_text())
assert automation["status"] == "PAUSED"
result = dict(
    capture_passed=False, cleanup_verified=True, on_started=False,
    closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    protected_unchanged=len(manifest), container_states=states,
    heartbeat_status=automation["status"],
    report_sha256=sha(H / "REPORT.txt"),
    failure_evidence_sha256=sha(H / "failure_evidence.json"),
    note="Stopped on failure; no retry, replay, scoring, tuning or next experiment.")
with (H / "failure_closeout.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
