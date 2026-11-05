"""Close out CP-K12 without changing any captured or protected artifact."""
from pathlib import Path
import datetime
import hashlib
import json
import subprocess

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_speed60_cpk12_20261020/prepared")
V = Path("/home/yeranis/results/gnssfgo_eth1_speed60_cpk12_20261020/on")

def load(path):
    return json.loads(path.read_text())

def sha(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

assert not (H / "final_checks.json").exists()
assert load(H / "results.json")["passed"]
assert (P / "speed60_on_control/complete.json").exists()
assert not (P / "speed60_on_control/failed.txt").exists()
gates = [
    P / "speed60_on_source/local_provenance.json",
    P / "speed60_on_source/candidate_validation.json",
    V / "validation.json",
    V / "repeat_validation.json",
]
assert all(load(p)["passed"] for p in gates)
manifest = load(P / "protected_manifest.json")
assert all(sha(p) == expected for p, expected in manifest.items())
containers = {}
for name in ("cbsms_ws_gpu", "ubuntu-jammy-humble-nvidia"):
    state = subprocess.check_output(
        ["docker", "inspect", "--format", "{{.State.Status}}", name], text=True
    ).strip()
    assert state == "exited", (name, state)
    containers[name] = state
subprocess.run(["git", "-C", "/home/yeranis/repos/gnssFGO", "diff", "--check"], check=True)
receipt = dict(
    passed=True,
    utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    protected_unchanged=len(manifest),
    gates={str(p): sha(p) for p in gates},
    containers=containers,
    report_sha256=sha(H / "REPORT.txt"),
    results_sha256=sha(H / "results.json"),
    scope="One 60s CBS-ON capture at0.30x; receiver-only deterministic replay; no retry, tuning or new run.",
)
with (H / "final_checks.json").open("x") as f:
    json.dump(receipt, f, indent=2)
print(json.dumps(receipt, indent=2))
