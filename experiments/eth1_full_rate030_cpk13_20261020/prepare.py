"""Rebase validated CP-K11/K12 harness for matched full ETH1 at0.30x."""
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
K = H.parent / "eth1_speed60_cpk12_20261020"
T = W / "runs/gnssfgo_eth1_prior_live120_cpk11_20261020/prepared"
S = W / "runs/gnssfgo_eth1_speed60_cpk12_20261020/prepared"
R = W / "runs/gnssfgo_eth1_full_rate030_cpk13_20261020/prepared"
V = Path("/home/yeranis/results/gnssfgo_eth1_full_rate030_cpk13_20261020")
DURATION = 346.32

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def once(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)

def common(s):
    for root, label, port in ((T, "eth1_cpk11_live120", "11459"),
                              (S, "eth1_cpk12_speed60", "11460")):
        s = s.replace(root.parent.name, R.parent.name).replace(label, "eth1_cpk13_full").replace(port, "11461")
    return (s.replace("live120", "full").replace("speed60", "full")
             .replace("CP-K11", "CP-K13").replace("CP-K12", "CP-K13")
             .replace("bag0-120s", "bag0-346.32s").replace("bag0-60s", "bag0-346.32s"))

assert not R.parent.exists() and not V.exists(), "refuse duplicate preparation"
assert shutil.disk_usage(W).free > 20 * 1024**3
assert json.loads((F / "final_checks.json").read_text())["passed"]
assert json.loads((K / "final_checks.json").read_text())["passed"]
manifest = json.loads((S / "protected_manifest.json").read_text())
for p, want in manifest.items():
    assert sha(p) == want, p
print("CP-K11/K12 frozen inputs verified", flush=True)

check = (K / "inspect_bags.py").read_text()
check = once(check, "start=epoch;end=(query_start_ns+60000000000)*1e-9",
    "import rospy\n"
    "with rosbag.Bag(out['hesai']['path']) as b:\n"
    " last_selected_record_ns=None\n"
    " for topic,msg,record in b.read_messages(topics=required['hesai'],"
    "start_time=rospy.Time.from_sec(out['hesai']['record_end']-1)):\n"
    "  last_selected_record_ns=record.to_nsec()\n"
    " assert last_selected_record_ns is not None\n"
    "duration_ns=((last_selected_record_ns-query_start_ns+999999)//1000000)*1000000\n"
    "assert duration_ns==346320000000,duration_ns\n"
    "start=epoch;end=(query_start_ns+duration_ns)*1e-9")
check = once(check, "out[k]['record_end']>=end", "out[k]['record_end']+0.001>=end")
check = once(check, "duration=60", "duration=346.32")
check = once(check, "query_end_ns=query_start_ns+60000000000",
    "query_end_ns=query_start_ns+duration_ns,last_selected_hesai_record_ns=last_selected_record_ns,"
    "end_rule='last selected Hesai record rounded up to1ms from first selected input; no camera/IMU-only tails'")
proc = subprocess.run(["docker", "exec", "-i", "cbsms_ws_gpu", "bash", "-lc",
    "source /opt/ros/noetic/setup.bash; python3 -"], input=check, text=True, capture_output=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
bags = json.loads(proc.stdout)
assert bags["passed"] and bags["window"]["duration"] == DURATION
R.mkdir(parents=True)
shutil.copytree(S / "private_build", R / "private_build", symlinks=True)
shutil.copytree(S / "input_profiles", R / "input_profiles")
files = ["experiment.py", "coverage_checks.py", "validate_case.py", "validate_candidate.py",
    "validate_local.py", "map_guard.py", "source.env", "receiver_fl4e.sh",
    "run_case.py", "run_case_on.py", "verify_effective.py", "verify_effective_on.py",
    "source_runner.sh", "source_runner_on.sh", "active_window.launch", "active_window_on.launch",
    "glim_profile.launch", "glim_profile_on.launch", "grandtour_eth1_kimera_stereo.launch",
    "kimera_vio_ros_newer_college_2021_stereo.launch", "kimera_vio_ros.launch",
    "live120_off.json", "live120_on.json", "evaluate_fl4f.py", "preflight_on.py"]
corrected = {"validate_case.py", "validate_candidate.py", "validate_local.py"}
diff = []
for name in files:
    template = (S if name in corrected else T) / name
    old = template.read_text()
    s = common(old)
    if name in ("run_case.py", "run_case_on.py"):
        s = once(s, "duration=120", "duration=346.32")
        s = once(s, "live at 0.1x", "live at 0.3x")
        s = once(s, "duration*10+600", "duration/0.30+600")
        s = once(s, "*source.glob('ros_logs/*/glim_rosnode-*-stdout.log')]",
            "*source.glob('ros_logs/*/glim_rosnode-*-stdout.log'),"
            "*source.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]")
        s = once(s, "'Persistent BPSAM recovery']",
            "'Persistent BPSAM recovery','process has died','terminate called','Segmentation fault']")
    elif name == "source.env":
        s = once(s, "CBSMS_BAG_DURATION_SEC:-120.0", "CBSMS_BAG_DURATION_SEC:-346.32")
        s = once(s, "CBSMS_BAG_RATE=0.10", "CBSMS_BAG_RATE=0.30")
    elif name in ("source_runner.sh", "source_runner_on.sh"):
        s = once(s, '"${duration_sec}" == 120.0', '"${duration_sec}" == 346.32 && "${bag_rate}" == 0.30')
        s = s.replace("120-second", "full346.32-second 0.30x")
        s = once(s, "min_glim_rows=1170", "min_glim_rows=3430")
        s = once(s, "min_kimera_rows=570", "min_kimera_rows=1700")
    elif name in ("verify_effective.py", "verify_effective_on.py"):
        s = once(s, "['CBSMS_BAG_DURATION_SEC'] == '120.0'",
            "['CBSMS_BAG_DURATION_SEC'] == '346.32'\nassert os.environ['CBSMS_BAG_RATE'] == '0.30'")
        s = once(s, "passed=True, checked_before_playback=True,",
            "passed=True, checked_before_playback=True, playback_rate=0.30, requested_s=346.32,")
    elif name == "validate_case.py":
        s = once(s, "['requested_s']==60", "['requested_s']==346.32")
        s = once(s, "['native_glim']['rows']>=570", "['native_glim']['rows']>=3430")
        s = once(s, "['native_kimera']['rows']>=270", "['native_kimera']['rows']>=1700")
        s = once(s, "1e-9>=58.", "1e-9>=344.32")
        s = s.replace("bounded pilot", "full ETH1 capture")
    elif name == "validate_candidate.py":
        s = once(s, "len(rows) >= 270", "len(rows) >= 1700")
    elif name == "validate_local.py":
        s = once(s, "first>=570", "first>=3430")
    elif name == "preflight_on.py":
        s = s.replace("same120s OFF", "same346.32s OFF")
    target = R / name.replace("live120", "full")
    target.write_text(s)
    shutil.copymode(template, target)
    diff.extend(difflib.unified_diff(old.splitlines(True), s.splitlines(True),
        fromfile=str(template), tofile=str(target)))
shutil.copytree(S / "transport_tools", R / "transport_tools", ignore=shutil.ignore_patterns("__pycache__"))
for p in (R / "transport_tools").rglob("*.py"):
    p.write_text(common(p.read_text()))
shutil.copy2(H / "PROTOCOL.txt", R / "PROTOCOL.txt")
(H / "inspect_bags.py").write_text(check)
(H / "harness.diff").write_text("".join(diff))
for name in ("summarize.py", "final_checks.py"):
    s = common((F / name).read_text()).replace("matched120s ETH1", "matched full346.32s ETH1")
    if name == "summarize.py":
        s = once(s, "on['span_s'])>=118", "on['span_s'])>=344.32")
        s = s.replace("pair at0.10x", "pair at0.30x")
    (H / name).write_text(s)

def config(path):
    c = json.loads(path.read_text())
    for k in ("sequence", "ros_master_uri", "receiver_binary"):
        c.pop(k)
    for source in c["sources"]:
        source.pop("session")
    return c

assert config(R / "full_off.json") == config(R / "full_on.json") == config(S / "speed60_on.json")
for name in ("receiver_fl4e.sh", "coverage_checks.py"):
    assert sha(R / name) == sha(S / name)
for folder in ("private_build", "input_profiles"):
    for p in (S / folder).rglob("*"):
        if p.is_file():
            assert sha(p) == sha(R / p.relative_to(S)), p
assert (R / "evaluate_fl4f.py").read_text() == common((T / "evaluate_fl4f.py").read_text())
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
preflight = dict(passed=True, template=str(T), corrected_checks=str(S), bags=bags,
    requested_s=DURATION, playback_rate=.30, bag_start_s=0, protected_files=len(manifest),
    binaries_identical=True, receiver_model_identical=True, profiles_identical=True,
    scoring_helpers_unchanged=True, coverage_checks_unchanged=True,
    initial_container_state="both stopped before authorized preparation",
    free_bytes=shutil.disk_usage(W).free)
(R / "preflight/validation.json").write_text(json.dumps(preflight, indent=2) + "\n")
print(json.dumps({k:v for k,v in preflight.items() if k != "bags"}, indent=2))
