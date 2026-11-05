"""One bounded container-owned GLIM capture; no automatic retry or other estimator."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import traceback
import xmlrpc.client
import yaml

P = Path(__file__).resolve().parent
W = Path("/workspace/cbs_gtsam4.3")
R = P.parent
mode = sys.argv[1]
assert mode in ("reference", "batched")
if mode == "batched":
    assert json.loads((P / "reference/validation.json").read_text())["passed"]
out = P / mode
assert not out.exists(), "No overwrites or retries"
port = 11463
with socket.socket() as sock:
    assert sock.connect_ex(("127.0.0.1", port)) != 0, "test port occupied"
for proc in Path("/proc").iterdir():
    if not proc.name.isdigit():
        continue
    try:
        argv = (proc / "cmdline").read_bytes().split(b"\0")
    except OSError:
        continue
    if argv and Path(argv[0].decode(errors="replace")).name in (
            "glim_rosnode", "kimera_vio_ros_node", "candidate_live"):
        raise RuntimeError("competing estimator " + str(proc))
manifest = json.loads((P / "prepared.json").read_text())
for name, expected in manifest["files"].items():
    assert hashlib.sha256((P / name).read_bytes()).hexdigest() == expected, name
inputs = json.loads((P / "input_manifest.json").read_text())
for spec in inputs["topics"].values():
    st = Path(spec["bag"]).stat()
    assert (st.st_size, st.st_mtime_ns) == (spec["bytes"], spec["mtime_ns"])
out.mkdir()
env = os.environ.copy()
for key in list(env):
    if key.startswith(("CPK16", "GLIM_SHARED_NATIVE", "GLIM_REVISION_COVARIANCE",
                       "CBS_E1B", "LD_PRELOAD")):
        del env[key]
env.update(ROS_MASTER_URI="http://127.0.0.1:" + str(port), ROS_IP="127.0.0.1",
           ROS_LOG_DIR=str(out / "ros_logs"))
libs = [W / "runs/native_covariance_cache_cpi3_20261019/release_v2",
        W / "runs/local_revision_covariance_cpi1_20261019",
        W / "runs/shared_linearization_s4_provider_20261012/release_2"]
if mode == "batched":
    libs.insert(0, R / "release_1")
node_env = env.copy()
node_env.update(LD_LIBRARY_PATH=":".join(map(str, libs)) + ":" + env.get("LD_LIBRARY_PATH", ""),
    GLIM_SHARED_NATIVE_PROVIDER="shared_native", GLIM_REVISION_COVARIANCE_POLICY="native_local",
    CBS_E1B_LOCAL_UPDATE_GPU_BATCHING="1", CBS_E1B_PROFILE_PATH=str(out / "e1b_profile.jsonl"))
if mode == "batched":
    node_env["CPK16_MAIN_GPU_BATCHING"] = "1"
expected_dso = ("6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca"
    if mode == "batched" else "3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7")
assert hashlib.sha256((libs[0] / "libgtsam_points.so.1.2.1").read_bytes()).hexdigest() == expected_dso
owned = {}
handles = []
start_wall = time.monotonic()
def save(name, data):
    with (out / name).open("x") as f:
        json.dump(data, f, indent=2)
def launch(name, args, child_env=env):
    log = (out / (name + ".log")).open("x")
    handles.append(log)
    proc = subprocess.Popen(args, env=child_env, cwd=out, start_new_session=True,
                            stdout=log, stderr=subprocess.STDOUT)
    owned[name] = proc
    save(name + "_process.json", dict(pid=proc.pid, args=args))
    return proc
def stop(name):
    proc = owned.get(name)
    if proc is None or proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=45)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=10)
        raise RuntimeError(name + " required TERM after graceful timeout")
def wait_until(test, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p in owned.values()):
            raise RuntimeError("owned job exited while waiting for " + label)
        try:
            if test():
                return
        except (ConnectionRefusedError, OSError, xmlrpc.client.ProtocolError):
            pass
        time.sleep(.2)
    raise RuntimeError(label + " timeout")
def interrupted(signum, frame):
    raise InterruptedError("controller interrupted")
signal.signal(signal.SIGTERM, interrupted)
signal.signal(signal.SIGINT, interrupted)
failure = None
try:
    launch("master", ["roscore", "-p", str(port)])
    rpc = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
    wait_until(lambda: rpc.getPid("/cpk16d")[0] == 1, 20, "master")
    params = yaml.safe_load((P / "frozen_glim_params.yaml").read_text())
    params["config_path"] = str(P / "glim_config")
    params["glim_dump_path"] = str(out / "glim_dump")
    params["glim_factor_graph_inspector_recording_id"] = "cpk16d_" + mode
    assert params["glim_cbs_receive_external_beliefs"] is False
    assert params["glim_cbs_publish_beliefs"] is True
    assert params["glim_cbs_live_revision_enable"] is False
    assert params["glim_cbs_odom_max_horizon_pairs_per_update"] == 1
    assert params["glim_cbs_odom_covariance_mode"] == "schur_relative_between"
    assert params["glim_cbs_experimental_local_gpu_batching"] is True
    for key, value in (("/use_sim_time", True), ("/glim_rosnode", params)):
        assert rpc.setParam("/cpk16d", key, value)[0] == 1
    save("effective_params.json", rpc.getParam("/cpk16d", "/glim_rosnode")[2])
    subprocess.run(["rostopic", "pub", "-1", "/clock", "rosgraph_msgs/Clock",
                    "{clock: {secs: 1727775007, nsecs: 0}}"], env=env,
                   stdout=subprocess.DEVNULL, timeout=10, check=True)
    node = launch("node", [str(W / "devel/lib/glim_ros/glim_rosnode"),
        "__name:=glim_rosnode", "/os_cloud_node/imu:=/boxi/cpt7/imu",
        "/os_cloud_node/points:=/boxi/hesai/points"], node_env)
    # Frozen GLIM startup waits for a nonzero ROS clock.
    subprocess.run(["rostopic", "pub", "-1", "/clock", "rosgraph_msgs/Clock",
                    "{clock: {secs: 1727775007, nsecs: 0}}"], env=env,
                   stdout=subprocess.DEVNULL, timeout=10, check=True)
    def node_ready():
        info = rpc.lookupNode("/cpk16d", "/glim_rosnode")
        if info[0] != 1:
            return False
        api = xmlrpc.client.ServerProxy(info[2])
        subs = dict(api.getSubscriptions("/cpk16d")[2])
        return "/boxi/cpt7/imu" in subs and "/boxi/hesai/points" in subs
    wait_until(node_ready, 35, "GLIM sensor subscriptions")
    maps = Path("/proc/%s/maps" % node.pid).read_text()
    (out / "maps.txt").write_text(maps)
    assert str(libs[0] / "libgtsam_points.so.1.2.1") in maps
    bridge = W / "runs/local_revision_covariance_cpi1_20261019/libglim_ros.so"
    assert str(bridge) in maps
    assert hashlib.sha256(bridge.read_bytes()).hexdigest() == "42df73c172232a612da965bf43ead8ec454caf08434b84e2da5de74d26cdb79f"
    assert "/CPK16" not in maps
    launch("observer", ["python3", str(P / "observe_k16d.py"), str(out)])
    launch("record", ["rosbag", "record", "--buffsize=256", "-O", str(out / "outputs.bag"),
                     "/glim/cbs/odometry", "/kimera/cbs/odom_belief_in",
                     "__name:=cpk16d_record"])
    wait_until(lambda: (out / "observer_ready").exists() and
        rpc.lookupNode("/cpk16d", "/cpk16d_record")[0] == 1, 20, "recorders")
    time.sleep(2)
    state = rpc.getSystemState("/cpk16d")[2]
    assert not any(t in ("/boxi/cpt7/imu", "/boxi/hesai/points",
                        "/kimera/cbs/odom_belief_out") and pubs for t, pubs in state[0])
    save("startup.json", dict(passed=True, state=state, dso=expected_dso,
        environment={k: v for k, v in node_env.items() if
            k in ("LD_LIBRARY_PATH", "CPK16_MAIN_GPU_BATCHING", "GLIM_SHARED_NATIVE_PROVIDER",
                  "GLIM_REVISION_COVARIANCE_POLICY", "CBS_E1B_LOCAL_UPDATE_GPU_BATCHING",
                  "CBS_E1B_PROFILE_PATH")}))
    bag = launch("bag", ["rosbag", "play", "--quiet", "--clock", "--delay=2.0", "--rate=0.20",
        "--start=0", "--duration=" + str(inputs["playback_duration_s"]),
        *[x["bag"] for x in inputs["topics"].values()], "--topics", *inputs["topics"]])
    deadline = time.monotonic() + 400
    while bag.poll() is None:
        assert time.monotonic() < deadline, "bounded playback deadline"
        for name in ("node", "observer", "record", "master"):
            assert owned[name].poll() is None, name + " exited during playback"
        time.sleep(.5)
    assert bag.returncode == 0
    time.sleep(25)
    assert node.poll() is None
    stop("node")
    assert node.returncode == 0, ("node shutdown", node.returncode)
    time.sleep(1)
    stop("record")
    stop("observer")
    save("capture_complete.json", dict(passed=True, mode=mode,
        wall_seconds=time.monotonic() - start_wall, node_returncode=node.returncode,
        bag_returncode=bag.returncode, requested_window_s=60, playback_rate=.20))
except BaseException as exc:
    failure = exc
    (out / "failed.txt").write_text(traceback.format_exc())
finally:
    for name in ("bag", "node", "record", "observer", "master"):
        try:
            stop(name)
        except BaseException as exc:
            failure = failure or exc
            with (out / "cleanup_failure.txt").open("a") as f:
                f.write(repr(exc) + "\n")
    for handle in handles:
        handle.close()
    save("cleanup.json", dict(returncodes={k: p.poll() for k, p in owned.items()},
                             all_exited=all(p.poll() is not None for p in owned.values())))
if failure is not None:
    raise failure
print(json.dumps(dict(mode=mode, captured=True, validation_pending=True)))
