"""Read-only integrity check; writes only this diagnostic's final receipt."""
import datetime
import hashlib
import json
from pathlib import Path

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared")
manifest = json.loads((P / "protected_manifest.json").read_text())
changed = []
for name, expected in manifest.items():
    with Path(name).open("rb") as f:
        actual = hashlib.file_digest(f, "sha256").hexdigest()
    if actual != expected:
        changed.append(name)
evidence = json.loads((H / "evidence_v2.json").read_text())
assert evidence["cpk13"]["failed_event_detail"]["event"]["event"] == 93339
assert evidence["cpk14"]["failed_event_detail"]["event"]["event"] == 214556
assert evidence["cpk14"]["failed_event_detail"]["event"]["history_s"] > 11
assert evidence["cpk14"]["failed_event_detail"]["backend_timers"]["backend"] < 65
supplement = json.loads((H / "supplement.json").read_text())
assert supplement["cpk14"]["main_sync_fraction"] > .95
assert supplement["cpk14"]["local_context_rows"] == 0
assert supplement["cpk14"]["gpu_cache_hits"] == 0
result = {
    "checked_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "protected_files": len(manifest), "changed": changed,
    "passed": not changed,
    "source_solver_or_replay_executions": 0,
    "builds": 0,
    "production_or_launch_edits": 0,
    "assertions": "Exact first uncommitted events and main diagnostic findings checked.",
    "container_state": "Separate read-only docker inspect: both exited,false; no container started.",
}
with (H / "integrity.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
if changed:
    raise SystemExit("Protected inputs changed")
