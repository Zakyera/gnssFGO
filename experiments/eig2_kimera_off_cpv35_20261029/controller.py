"""Finite one-shot capture controller; restore the originally stopped container."""
import fcntl,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_kimera_off_cpv35_20261029')
assert (H/'ready.json').exists()
lock=(H/'controller.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert not (P/'capture').exists() and not (H/'start.json').exists()
def write(name,d):
    with (H/name).open('x') as f:json.dump(d,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
write('start.json',dict(pid=os.getpid(),wall_ns=time.time_ns(),original_container_states={'cbsms_ws_gpu':False,'ubuntu-jammy-humble-nvidia':False}))
cmd=['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash && source /workspace/cbs_gtsam4.3/devel/setup.bash && export PYTHONDONTWRITEBYTECODE=1 && exec python3 /workspace/cbs_gtsam4.3/runs/eig2_kimera_off_cpv35_20261029/run_inside.py']
rc=None
try:
    with (H/'capture_controller.log').open('x') as f:
        job=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
        write('capture_process.json',dict(host_pid=job.pid,argv=cmd));rc=job.wait(timeout=2200)
    assert rc==0,('capture failed',rc)
    write('capture_finished.json',dict(returncode=rc,wall_ns=time.time_ns(),analysis_pending=True))
except BaseException as exc:
    write('controller_failed.json',dict(error=repr(exc),returncode=rc));raise
finally:
    cleanup=json.loads((P/'capture/cleanup.json').read_text()) if (P/'capture/cleanup.json').exists() else None
    top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
    leftovers=[l for l in top.splitlines()[1:] if any(x in l for x in ('rosbag','roslaunch','roscore','kimera_vio_ros_node','run_inside.py'))]
    if leftovers or (cleanup and (cleanup['errors'] or not cleanup['kimera_process_gone'])):
        write('cleanup_blocked.json',dict(top=top,cleanup=cleanup));raise RuntimeError('owned cleanup not verified; container left running')
    stopped=call(['docker','stop','-t','20','cbsms_ws_gpu'])
    state=call(['docker','inspect','--format','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    write('container_restoration.json',dict(states=state,stop_receipt=stopped,cleanup=cleanup,top_before_stop=top))
