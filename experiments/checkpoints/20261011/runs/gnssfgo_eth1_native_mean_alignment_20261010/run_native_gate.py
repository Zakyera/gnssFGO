"""8 s Kimera-only isolation. Run inside existing Noetic container, once/case."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client
import yaml

root = Path(__file__).resolve().parent
ws = root.parents[1]
old = ws/'runs/gnssfgo_eth1_active_window40_prepared_20261008/pilot_on_source'
parser = argparse.ArgumentParser()
parser.add_argument('case', choices=['no_cbs','raw','aligned'])
args = parser.parse_args()
assert (root/'unit_v2/unit_validation.json').exists()
out = root/('native8_'+args.case)
out.mkdir(exist_ok=False)
(out/'kimera_logs').mkdir()
env = os.environ.copy()
env.update(ROS_MASTER_URI='http://127.0.0.1:11390', ROS_LOG_DIR=str(out/'ros_logs'),
           KIMERA_CBS_REVISION_AUDIT='1')
env.pop('KIMERA_IMU_SAMPLE_SUPPORT',None)
# Select the new library only for these owned processes. No install/default swap.
env['LD_LIBRARY_PATH'] = str(ws/'build/kimera_vio')+':'+str(ws/'devel/lib')+':'+env.get('LD_LIBRARY_PATH','')
processes, logs = [], []
def start(name, command):
    log = (out/(name+'.log')).open('x'); logs.append(log)
    p = subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p)
    return p
def cmd(command,check=True):
    return subprocess.run(command,env=env,capture_output=True,text=True,check=check)
def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=10)
try:
    assert cmd(['rosparam','get','/rosversion'],False).returncode != 0, 'owned master port busy'
    master = start('master',['roscore','-p','11390'])
    for _ in range(60):
        if cmd(['rosparam','get','/rosversion'],False).returncode == 0: break
        time.sleep(.25)
    cmd(['rosparam','set','/use_sim_time','true'])
    # Use saved effective ROS settings, with only receiving disabled in control.
    settings = yaml.safe_load((old/'resolved_kimera_params.yaml').read_text())
    settings['cbs_belief_receive_enable'] = args.case != 'no_cbs'
    settings['rerun_recording_id'] = out.name
    with (out/'resolved_params.yaml').open('x') as f: yaml.safe_dump(settings,f)
    cmd(['rosparam','load',str(out/'resolved_params.yaml'),'/kimera_vio_ros/kimera_vio_ros_node'])
    for side in ['left','right']:
        start('camera_'+side,['rosrun','kimera_vio_ros','compressed_to_raw_bridge.py',
              '__name:=mean_gate_camera_'+side,
              '_input_topic:=/boxi/alphasense/front_'+side+'/image_raw/compressed',
              '_output_topic:=/grandtour/front_'+side+'/image_raw',
              '_output_encoding:=mono8','_queue_size:=30'])
    saved = json.loads((old/'effective_mode_check.json').read_text())['kimera_argv']
    argv = [a for a in saved if not a.startswith('__log:=')]
    argv = ['--output_path='+str(out/'kimera_logs') if a.startswith('--output_path=') else a for a in argv]
    argv.append('--cbs_experimental_native_imu_mean_alignment='+str(args.case=='aligned').lower())
    # roslaunch supplied the namespace through its process environment.
    argv.append('__ns:=/kimera_vio_ros')
    node = start('estimators',argv)
    for _ in range(120):
        if '/kimera_vio_ros/kimera_vio_ros_node' in cmd(['rosnode','list'],False).stdout: break
        assert node.poll() is None, 'Kimera failed at startup'
        time.sleep(.25)
    else: raise RuntimeError('Kimera readiness timeout')
    # Seed simulated clock exactly as the original runner before bag startup.
    seed = start('clock_seed',['rostopic','pub','-1','/clock','rosgraph_msgs/Clock','{clock: {secs: 1727775007, nsecs: 0}}'])
    seed.wait(timeout=10)
    record = start('recorder',['rosbag','record','--buffsize=256','-O',str(out/'native.bag'),
                    '/kimera/cbs/odom_belief_in','/kimera/cbs/odom_belief_out',
                    '/kimera_vio_ros/odometry','/boxi/cpt7/imu',
                    '/boxi/alphasense/front_left/image_raw/compressed',
                    '/boxi/alphasense/front_right/image_raw/compressed'])
    time.sleep(2)
    data = ws/'src/datasets/GrandTour'
    bags = [data/('2024-10-01-11-29-55_'+s+'.bag') for s in ['hesai','cpt7_imu','alphasense']]
    bags.append(old/'recordings/cbs.bag')
    command = ['rosbag','play','--quiet','--clock','--delay=2.0','--rate=0.10','--start=0','--duration=8']
    command += [str(b) for b in bags]
    command += ['--topics','/boxi/cpt7/imu','/boxi/alphasense/front_left/image_raw/compressed',
                '/boxi/alphasense/front_right/image_raw/compressed','/kimera/cbs/odom_belief_in']
    def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
    with (out/'execution.json').open('x') as f:
        json.dump(dict(argv=argv,playback=command,case=args.case,
            library_sha256=sha(ws/'build/kimera_vio/libkimera_vio.so'),
            node_sha256=sha(argv[0]), source_profile=str(old/'kimera_profile'),
            description='8-second native Kimera isolation, not a full-system 40-second result'),f,indent=2)
    player = start('playback',command)
    deadline = time.monotonic()+140
    while player.poll() is None:
        assert node.poll() is None, 'Kimera stopped during playback'
        assert time.monotonic()<deadline, 'playback timeout'
        time.sleep(.5)
    assert player.returncode == 0
    time.sleep(10)
    assert node.poll() is None, 'Kimera failed during drain'
    stop(record)
    stop(node)
    with (out/'capture_complete.json').open('x') as f:
        json.dump(dict(capture_complete=True,validation_required=True),f)
    print('Captured '+args.case+' native 8-second diagnostic',flush=True)
except BaseException as e:
    (out/'failed.txt').write_text(str(e)+'\n')
    raise
finally:
    for p in reversed(processes): stop(p)
    for f in logs: f.close()
