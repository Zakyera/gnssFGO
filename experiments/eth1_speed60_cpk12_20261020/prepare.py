"""Mechanically rebase the validated CP-K11 ON harness for one 60s/0.30x trial."""
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
F = H.parent / "kimera_prior_matched_live120_cpk11_20261020"
T = W / "runs/gnssfgo_eth1_prior_live120_cpk11_20261020/prepared"
R = W / "runs/gnssfgo_eth1_speed60_cpk12_20261020/prepared"
V = Path("/home/yeranis/results/gnssfgo_eth1_speed60_cpk12_20261020")

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def once(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)

def common(s):
    return (s.replace(T.parent.name, R.parent.name)
            .replace("eth1_cpk11_live120", "eth1_cpk12_speed60")
            .replace("live120", "speed60").replace("11459", "11460")
            .replace("CP-K11", "CP-K12").replace("bag0-120s", "bag0-60s"))

assert not R.parent.exists() and not V.exists(), "refuse duplicate preparation"
assert json.loads((F / "final_checks.json").read_text())["passed"]
assert shutil.disk_usage(W).free > 15 * 1024**3
manifest = json.loads((T / "protected_manifest.json").read_text())
for p, want in manifest.items():
    assert sha(p) == want, p
print("CP-K11 frozen inputs verified", flush=True)

check = (F / "inspect_bags.py").read_text().replace("120000000000", "60000000000")
check = once(check, "duration=120", "duration=60")
proc = subprocess.run(["docker", "exec", "-i", "cbsms_ws_gpu", "bash", "-lc",
    "source /opt/ros/noetic/setup.bash; python3 -"], input=check,
    capture_output=True, text=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
bags = json.loads(proc.stdout)
assert bags["passed"] and bags["window"]["duration"] == 60
R.mkdir(parents=True)
shutil.copytree(T / "private_build", R / "private_build", symlinks=True)
shutil.copytree(T / "input_profiles", R / "input_profiles")
files = [
    "experiment.py", "coverage_checks.py", "validate_case.py", "validate_local.py",
    "validate_candidate.py", "map_guard.py", "source.env", "receiver_fl4e.sh",
    "run_case_on.py", "verify_effective_on.py", "source_runner_on.sh",
    "active_window_on.launch", "glim_profile_on.launch",
    "grandtour_eth1_kimera_stereo.launch",
    "kimera_vio_ros_newer_college_2021_stereo.launch",
    "kimera_vio_ros.launch", "live120_on.json",
]
diff = []
for name in files:
    old = (T / name).read_text()
    s = common(old)
    if name == "run_case_on.py":
        s = once(s, "duration=120", "duration=60")
        s = once(s, "live at 0.1x", "live at 0.3x")
        s = once(s, "duration*10+600", "duration/0.30+600")
        s = once(s, "*source.glob('ros_logs/*/glim_rosnode-*-stdout.log')]",
                 "*source.glob('ros_logs/*/glim_rosnode-*-stdout.log'),"
                 "*source.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]")
        s = once(s, "'Persistent BPSAM recovery']",
                 "'Persistent BPSAM recovery','process has died','terminate called','Segmentation fault']")
    elif name == "source.env":
        s = once(s, "CBSMS_BAG_DURATION_SEC:-120.0", "CBSMS_BAG_DURATION_SEC:-60.0")
        s = once(s, "CBSMS_BAG_RATE=0.10", "CBSMS_BAG_RATE=0.30")
    elif name == "source_runner_on.sh":
        s = once(s, '"${duration_sec}" == 120.0',
                 '"${duration_sec}" == 60.0 && "${bag_rate}" == 0.30')
        s = s.replace("120-second", "60-second 0.30x")
        s = once(s, "min_glim_rows=1170", "min_glim_rows=570")
        s = once(s, "min_kimera_rows=570", "min_kimera_rows=270")
    elif name == "verify_effective_on.py":
        s = once(s, "['CBSMS_BAG_DURATION_SEC'] == '120.0'",
                 "['CBSMS_BAG_DURATION_SEC'] == '60.0'\n"
                 "assert os.environ['CBSMS_BAG_RATE'] == '0.30'")
        s = once(s, "passed=True, checked_before_playback=True,",
                 "passed=True, checked_before_playback=True, playback_rate=0.30, requested_s=60,")
    elif name == "validate_case.py":
        s = once(s, "['requested_s']==120", "['requested_s']==60")
        s = once(s, "['rows']>=1170", "['rows']>=570")
        s = once(s, "['native_kimera']['rows']>=570", "['native_kimera']['rows']>=270")
        s = once(s, "1e-9>=118.", "1e-9>=58.")
        s = once(s, "live 0.1x bounded pilot", "live 0.3x bounded pilot")
    elif name == "validate_local.py":
        s = once(s, "first>=1170", "first>=570")
    elif name == "validate_candidate.py":
        s = once(s, "len(rows) >= 570", "len(rows) >= 270")
        s = once(s, 'log = (S / "estimators.log").read_text()',
                 'import re\n'
                 'node_logs = list(S.glob("ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log"))\n'
                 'assert len(node_logs) == 1, node_logs\n'
                 'launcher_log = (S / "estimators.log").read_text()\n'
                 'assert "CPK9_PRIOR_REFRESH," not in launcher_log\n'
                 'log = launcher_log + "\\n" + re.sub(r"\\x1b\\[[0-9;]*m", "", node_logs[0].read_text())')
        s = once(s, 'assert sum(p["retired"] for p in promotions) == retirements',
                 'assert sum(p["retired"] for p in promotions) == retirements\n'
                 'by_update = {int(p["time"]): p["retired"] for p in promotions}\n'
                 'assert len(by_update) == len(promotions)\n'
                 'assert all(by_update.get(int(r[0]), 0) == r[2] for r in rows)')
    target = R / name.replace("live120", "speed60")
    target.write_text(s)
    shutil.copymode(T / name, target)
    diff.extend(difflib.unified_diff(old.splitlines(True), s.splitlines(True),
                                   fromfile=str(T / name), tofile=str(target)))
shutil.copytree(T / "transport_tools", R / "transport_tools",
               ignore=shutil.ignore_patterns("__pycache__"))
for p in (R / "transport_tools").rglob("*.py"):
    p.write_text(common(p.read_text()))
shutil.copy2(H / "PROTOCOL.txt", R / "PROTOCOL.txt")
(H / "inspect_bags.py").write_text(check)
(H / "harness.diff").write_text("".join(diff))

# The latest explicit user request authorizes standalone ON, not a new OFF run.
assert json.loads((T / "live120_on_source/candidate_validation.json").read_text())["passed"]
assert json.loads((F / "results.json").read_text())["passed"]
admission = dict(passed=True, admission="Explicit user-authorized standalone ON speed test",
                baseline=str(T), baseline_validated=True, requested_s=60, playback_rate=0.30)
with (R / "preflight_on.json").open("x") as f:
    json.dump(admission, f, indent=2)

def config(path):
    c = json.loads(path.read_text())
    for k in ("sequence", "ros_master_uri", "receiver_binary"):
        c.pop(k)
    for source in c["sources"]:
        source.pop("session")
    return c

assert config(R / "speed60_on.json") == config(T / "live120_on.json")
for name in ("receiver_fl4e.sh", "coverage_checks.py"):
    assert sha(R / name) == sha(T / name)
for directory in ("private_build", "input_profiles"):
    for p in (T / directory).rglob("*"):
        if p.is_file():
            assert sha(p) == sha(R / p.relative_to(T)), p
for p in R.rglob("*"):
    if p.suffix == ".py":
        ast.parse(p.read_text())
    elif p.suffix == ".launch":
        ET.parse(p)
    elif p.suffix in (".sh", ".env"):
        subprocess.run(["bash", "-n", str(p)], check=True)
for base in (T, R):
    for p in base.rglob("*"):
        if base == T and p.parts[len(T.parts)] not in [*files, "input_profiles", "private_build", "transport_tools"]:
            continue
        if p.is_file() and "__pycache__" not in p.parts:
            manifest[str(p)] = sha(p)
for p in H.iterdir():
    if p.is_file() and (p.suffix == ".py" or p.name == "PROTOCOL.txt"):
        manifest[str(p)] = sha(p)
(R / "protected_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
(R / "preflight").mkdir()
preflight = dict(passed=True, template=str(T), bags=bags, requested_s=60,
                 playback_rate=0.30, bag_start_s=0, protected_files=len(manifest),
                 estimator_binaries_identical=True, receiver_model_identical=True,
                 profiles_identical=True, guards="preserved; actual Kimera node log also monitored",
                 initial_container_state="both stopped", free_bytes=shutil.disk_usage(W).free)
(R / "preflight/validation.json").write_text(json.dumps(preflight, indent=2) + "\n")
print(json.dumps({k: v for k, v in preflight.items() if k != "bags"}, indent=2))
