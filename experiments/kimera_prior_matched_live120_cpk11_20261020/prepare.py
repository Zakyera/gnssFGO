"""Prepare one matched120s CP-K11 harness from CP-R6; no estimator edits."""
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
F = H.parent / "local_covariance_eth1_cpr6_20261019"
T = W / "runs/gnssfgo_eth1_local_cov60_cpr6_20261019/prepared"
R = W / "runs/gnssfgo_eth1_prior_live120_cpk11_20261020/prepared"
CANDIDATE = W / "runs/kimera_incremental_prior_cpk9_20261019/private_build"

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def once(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)

def common(s):
    return (s.replace(T.parent.name, R.parent.name)
        .replace("eth1_cpr6_live60_", "eth1_cpk11_live120_")
        .replace("live60", "live120").replace("11452", "11459")
        .replace("CP-R6", "CP-K11").replace("bag0-60s", "bag0-120s"))

assert not R.parent.exists(), "refuse duplicate preparation"
assert not Path("/home/yeranis/results/gnssfgo_eth1_prior_live120_cpk11_20261020").exists()
assert shutil.disk_usage(W).free > 20 * 1024**3
assert json.loads((F / "final_checks.json").read_text())["passed"]
assert sha(CANDIDATE / "libkimera_vio.so") == "be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9"
assert sha(CANDIDATE / "libcbs.so") == sha(T / "private_build/libcbs.so")
manifest = json.loads((T / "protected_manifest.json").read_text())
for p, expected in manifest.items():
    assert sha(p) == expected, p
print("Frozen CP-R6 inputs and CP-K9 candidate verified", flush=True)

check = (F / "inspect_bags.py").read_text()
check = once(check, "  first={};last={};count=",
    "  selected_topic,selected_message,selected_record=next(b.read_messages(topics=topics))\n"
    "  out[kind]['first_selected_record_ns']=selected_record.to_nsec()\n"
    "  out[kind]['first_selected_topic']=selected_topic\n"
    "  first={};last={};count=")
check = once(check, "epoch=min(out[k]['record_start'] for k in ('hesai','cpt7_imu','alphasense'))",
    "query_start_ns=min(out[k]['first_selected_record_ns'] for k in ('hesai','cpt7_imu','alphasense'))\n"
    "epoch=query_start_ns*1e-9")
check = once(check, "start=epoch;end=start+60", "start=epoch;end=(query_start_ns+120000000000)*1e-9")
check = once(check, "bag_offset=0,duration=60",
    "bag_offset=0,duration=120,query_start_ns=query_start_ns,query_end_ns=query_start_ns+120000000000")
proc = subprocess.run(["docker", "exec", "-i", "cbsms_ws_gpu", "bash", "-lc",
    "source /opt/ros/noetic/setup.bash; python3 -"], input=check,
    capture_output=True, text=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
bags = json.loads(proc.stdout)
assert bags["passed"] and bags["window"]["duration"] == 120

R.mkdir(parents=True)
shutil.copytree(T / "private_build", R / "private_build", symlinks=True)
shutil.copy2(CANDIDATE / "libkimera_vio.so", R / "private_build/libkimera_vio.so")
shutil.copytree(T / "input_profiles", R / "input_profiles")
files = ["experiment.py", "coverage_checks.py", "validate_case.py",
    "validate_candidate.py", "map_guard.py", "source.env", "receiver_fl4e.sh",
    "run_case.py", "run_case_on.py", "verify_effective.py", "verify_effective_on.py",
    "source_runner.sh", "source_runner_on.sh", "active_window.launch",
    "active_window_on.launch", "glim_profile.launch", "glim_profile_on.launch",
    "grandtour_eth1_kimera_stereo.launch", "kimera_vio_ros_newer_college_2021_stereo.launch",
    "kimera_vio_ros.launch", "live60_off.json", "live60_on.json",
    "validate_local.py", "evaluate_fl4f.py", "preflight_on.py"]
diff = []
for name in files:
    old = (T / name).read_text()
    s = common(old)
    if name in ("run_case.py", "run_case_on.py"):
        s = once(s, "duration=60", "duration=120")
        s = once(s, "'SHARED_NATIVE_PROVIDER_FAILURE','FL4Q'",
                    "'SHARED_NATIVE_PROVIDER_FAILURE','CPK9 ','FL4Q'")
    if name == "source.env":
        s = once(s, "CBSMS_BAG_DURATION_SEC:-60.0", "CBSMS_BAG_DURATION_SEC:-120.0")
    if name in ("source_runner.sh", "source_runner_on.sh"):
        s = once(s, '"${duration_sec}" == 60.0', '"${duration_sec}" == 120.0')
        s = s.replace("60-second", "120-second")
        s = once(s, "min_glim_rows=570", "min_glim_rows=1170")
        s = once(s, "min_kimera_rows=270", "min_kimera_rows=570")
    if name == "kimera_vio_ros.launch":
        s = once(s, '<env name="FL4Q_ENABLE" value="1"/>',
                    '<env name="FL4Q_ENABLE" value="0"/>\n    <env name="CPK9_PRIOR_REFRESH" value="1"/>')
    if name in ("verify_effective.py", "verify_effective_on.py"):
        s = once(s, "['CBSMS_BAG_DURATION_SEC'] == '60.0'",
                    "['CBSMS_BAG_DURATION_SEC'] == '120.0'")
        s = once(s, "assert env(pid)['FL4Q_ENABLE']=='1'",
                    "assert env(pid)['FL4Q_ENABLE']=='0'\nassert env(pid)['CPK9_PRIOR_REFRESH']=='1'")
        s = once(s, "assert 'FL4Q_ENABLE' not in env(glim_pid)",
                    "assert 'FL4Q_ENABLE' not in env(glim_pid)\nassert 'CPK9_PRIOR_REFRESH' not in env(glim_pid)")
        s = once(s, "import hashlib\n",
            "import hashlib\n"
            "assert hashlib.sha256((root/'private_build/libkimera_vio.so').read_bytes()).hexdigest()"
            "=='be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9'\n")
        s = once(s, "kimera=expected_flags, kimera_pid=pid,",
            "kimera=expected_flags, kimera_candidate_env={k:env(pid)[k] for k in "
            "('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT')}, kimera_pid=pid,")
    if name == "validate_case.py":
        s = once(s, "['requested_s']==60", "['requested_s']==120")
        s = s.replace("['rows']>=570", "['rows']>=1170").replace("['rows']>=270", "['rows']>=570")
        s = once(s, "1e-9>=58.", "1e-9>=118.")
    if name == "validate_candidate.py":
        s = (H / name).read_text()
    if name == "validate_local.py":
        s = once(s, "first>=570", "first>=1170")
    if name == "preflight_on.py":
        s = s.replace("same60s OFF", "same120s OFF")
    target = R / name.replace("live60", "live120")
    target.write_text(s)
    shutil.copymode(T / name, target)
    diff.extend(difflib.unified_diff(old.splitlines(True), s.splitlines(True),
        fromfile=str(T / name), tofile=str(target)))
shutil.copytree(T / "transport_tools", R / "transport_tools",
               ignore=shutil.ignore_patterns("__pycache__"))
for p in (R / "transport_tools").rglob("*.py"):
    p.write_text(common(p.read_text()))
shutil.copy2(H / "PROTOCOL.txt", R / "PROTOCOL.txt")
for name in ("summarize.py", "final_checks.py"):
    s = common((F / name).read_text()).replace("matched60s ETH1", "matched120s ETH1")
    if name == "summarize.py":
        s = once(s, "on['span_s'])>=58", "on['span_s'])>=118")
        s = once(s, "'cached_provider_sha256','kimera'",
                    "'cached_provider_sha256','kimera','kimera_candidate_env'")
    (H / name).write_text(s)
(H / "inspect_bags.py").write_text(check)
(H / "harness.diff").write_text("".join(diff))

assert sha(R / "receiver_fl4e.sh") == sha(T / "receiver_fl4e.sh")
assert sha(R / "coverage_checks.py") == sha(T / "coverage_checks.py")
def config(p):
    obj = json.loads(p.read_text())
    for key in ("sequence", "ros_master_uri", "receiver_binary"):
        obj.pop(key)
    for source in obj["sources"]:
        source.pop("session")
    return obj
assert config(R / "live120_off.json") == config(R / "live120_on.json") == config(T / "live60_off.json")
def functions(code):
    return {x.name: ast.dump(x, include_attributes=False) for x in ast.parse(code).body
            if isinstance(x, ast.FunctionDef)}
assert functions(common((T / "evaluate_fl4f.py").read_text())) == functions((R / "evaluate_fl4f.py").read_text())
for p in R.rglob("*"):
    if p.suffix == ".py":
        ast.parse(p.read_text())
    if p.suffix == ".launch":
        ET.parse(p)
    if p.suffix in (".sh", ".env"):
        subprocess.run(["bash", "-n", str(p)], check=True)
for base in (T, R):
    for p in base.rglob("*"):
        if base == T and p.parts[len(T.parts)] not in [
                *files, "input_profiles", "private_build", "transport_tools"]:
            continue
        if p.is_file() and "__pycache__" not in p.parts:
            manifest[str(p)] = sha(p)
for p in H.iterdir():
    if p.is_file() and (p.suffix == ".py" or p.name == "PROTOCOL.txt"):
        manifest[str(p)] = sha(p)
manifest[str(CANDIDATE / "libkimera_vio.so")] = sha(CANDIDATE / "libkimera_vio.so")
(R / "protected_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
(R / "preflight").mkdir()
preflight = dict(passed=True, template=str(T), bags=bags, requested_s=120,
    bag_start_s=0, protected_files=len(manifest), free_bytes=shutil.disk_usage(W).free,
    only_estimator_change="CP-K9 Kimera private DSO with prior refresh1, FL4q0, remove; BOTH modes",
    same_candidate_in_both_modes=True, receiver_model_identical=True,
    scoring_helpers_unchanged=True, coverage_checks_unchanged=True,
    initial_container_state="both stopped before authorized preparation",
    recording="same diagnostic load as CP-R6; no live inspectors")
(R / "preflight/validation.json").write_text(json.dumps(preflight, indent=2) + "\n")
print(json.dumps(preflight, indent=2))
