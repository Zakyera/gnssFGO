"""Mechanical fresh CP-K13 harness copy: playback0.20 only; no model changes."""
from pathlib import Path
import ast
import difflib
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET

H = Path(__file__).resolve().parent
W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
F = H.parent / "eth1_full_rate030_cpk13_20261020"
S = W / "runs/gnssfgo_eth1_full_rate030_cpk13_20261020/prepared"
R = W / "runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared"
V = Path("/home/yeranis/results/gnssfgo_eth1_full_rate020_cpk14_20261020")

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def once(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)

def common(s):
    return (s.replace(S.parent.name, R.parent.name)
             .replace("eth1_cpk13_full", "eth1_cpk14_full")
             .replace("11461", "11462").replace("CP-K13", "CP-K14"))

assert not R.parent.exists() and not V.exists(), "refuse duplicate preparation"
assert shutil.disk_usage(W).free > 20 * 1024**3
closed = json.loads((F / "failure_closeout.json").read_text())
assert closed["cleanup_verified"] and not closed["capture_passed"] and not closed["on_started"]
assert json.loads((H.parent / "eth1_speed60_cpk12_20261020/final_checks.json").read_text())["passed"]
manifest = json.loads((S / "protected_manifest.json").read_text())
for p, want in manifest.items():
    assert sha(p) == want, p
print("CP-K13 frozen inputs and cleanup verified", flush=True)
check = (F / "inspect_bags.py").read_text()
proc = subprocess.run(["docker", "exec", "-i", "cbsms_ws_gpu", "bash", "-lc",
    "source /opt/ros/noetic/setup.bash && python3 -"], input=check,
    text=True, capture_output=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
bags = json.loads(proc.stdout)
assert bags["passed"] and bags["window"]["duration"] == 346.32
assert bags["window"]["query_start_ns"] == 1727775007855396309
assert bags["window"]["query_end_ns"] == 1727775354175396309
R.mkdir(parents=True)
shutil.copytree(S / "private_build", R / "private_build", symlinks=True)
shutil.copytree(S / "input_profiles", R / "input_profiles")
files = ["experiment.py", "coverage_checks.py", "validate_case.py", "validate_candidate.py",
    "validate_local.py", "map_guard.py", "source.env", "receiver_fl4e.sh",
    "run_case.py", "run_case_on.py", "verify_effective.py", "verify_effective_on.py",
    "source_runner.sh", "source_runner_on.sh", "active_window.launch", "active_window_on.launch",
    "glim_profile.launch", "glim_profile_on.launch", "grandtour_eth1_kimera_stereo.launch",
    "kimera_vio_ros_newer_college_2021_stereo.launch", "kimera_vio_ros.launch",
    "full_off.json", "full_on.json", "evaluate_fl4f.py", "preflight_on.py"]
diff = []
for name in files:
    old = (S / name).read_text()
    s = common(old)
    if name in ("run_case.py", "run_case_on.py"):
        s = once(s, "live at 0.3x", "live at 0.2x")
        s = once(s, "duration/0.30+600", "duration/0.20+600")
    elif name == "source.env":
        s = once(s, "CBSMS_BAG_RATE=0.30", "CBSMS_BAG_RATE=0.20")
    elif name in ("source_runner.sh", "source_runner_on.sh"):
        s = once(s, '"${bag_rate}" == 0.30', '"${bag_rate}" == 0.20')
        s = s.replace("full346.32-second 0.30x", "full346.32-second 0.20x")
    elif name in ("verify_effective.py", "verify_effective_on.py"):
        s = once(s, "['CBSMS_BAG_RATE'] == '0.30'", "['CBSMS_BAG_RATE'] == '0.20'")
        s = once(s, "playback_rate=0.30", "playback_rate=0.20")
    elif name == "validate_case.py":
        s = s.replace("live 0.3x", "live 0.2x")
    (R / name).write_text(s)
    shutil.copymode(S / name, R / name)
    diff.extend(difflib.unified_diff(old.splitlines(True), s.splitlines(True),
        fromfile=str(S / name), tofile=str(R / name)))
shutil.copytree(S / "transport_tools", R / "transport_tools",
    ignore=shutil.ignore_patterns("__pycache__"))
for p in (R / "transport_tools").rglob("*.py"):
    p.write_text(common(p.read_text()))
shutil.copy2(H / "PROTOCOL.txt", R / "PROTOCOL.txt")
shutil.copy2(F / "inspect_bags.py", H / "inspect_bags.py")
(H / "harness.diff").write_text("".join(diff))
for name in ("summarize.py", "final_checks.py"):
    s = common((F / name).read_text())
    if name == "summarize.py":
        s = once(s, "pair at0.30x", "pair at0.20x")
    (H / name).write_text(s)

def config(path):
    c = json.loads(path.read_text())
    for k in ("sequence", "ros_master_uri", "receiver_binary"):
        c.pop(k)
    for source in c["sources"]:
        source.pop("session")
    return c

assert config(R / "full_off.json") == config(R / "full_on.json") == config(S / "full_off.json")
for name in ("receiver_fl4e.sh", "coverage_checks.py"):
    assert sha(R / name) == sha(S / name)
for folder in ("private_build", "input_profiles"):
    for p in (S / folder).rglob("*"):
        if p.is_file():
            assert sha(p) == sha(R / p.relative_to(S)), p
assert (R / "evaluate_fl4f.py").read_text() == common((S / "evaluate_fl4f.py").read_text())
for p in R.rglob("*"):
    if p.suffix == ".py":
        ast.parse(p.read_text())
    elif p.suffix == ".launch":
        ET.parse(p)
    elif p.suffix in (".sh", ".env"):
        subprocess.run(["bash", "-n", str(p)], check=True)
for p in R.rglob("*"):
    if p.is_file() and "__pycache__" not in p.parts:
        manifest[str(p)] = sha(p)
for p in H.iterdir():
    if p.is_file() and (p.suffix == ".py" or p.name == "PROTOCOL.txt"):
        ast.parse(p.read_text()) if p.suffix == ".py" else None
        manifest[str(p)] = sha(p)
(R / "protected_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
(R / "preflight").mkdir()
preflight = dict(passed=True, template=str(S), bags=bags,
    requested_s=346.32, playback_rate=.20, bag_start_s=0, protected_files=len(manifest),
    binaries_identical=True, receiver_model_identical=True, profiles_identical=True,
    scoring_helpers_unchanged=True, coverage_checks_unchanged=True,
    only_capture_change="0.30 to0.20 playback; deadline and output identities",
    initial_container_state="both stopped before authorized preparation",
    free_bytes=shutil.disk_usage(W).free)
(R / "preflight/validation.json").write_text(json.dumps(preflight, indent=2) + "\n")
print(json.dumps({k: v for k, v in preflight.items() if k != "bags"}, indent=2))
