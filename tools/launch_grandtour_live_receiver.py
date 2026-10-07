#!/usr/bin/env python3
"""Launch receiver only in existing containers; never start a bag/estimator.

Requires a running ROS master before bag playback. Uses a fresh shared run
directory and explicit fresh source-run log directory. --check is read-only.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time

from live_receiver_protocol import validate_config

SHARED = Path('/home/yeranis/repos/V4RL')


def ros1_path(path):
    return '/workspace/'+str(Path(path).resolve().relative_to(SHARED))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--source-run', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--graph-snapshots', action='store_true', help='extra IO; off for timing tests')
    action = p.add_mutually_exclusive_group(required=True)
    action.add_argument('--check', action='store_true')
    action.add_argument('--start', action='store_true')
    args = p.parse_args()
    config = json.loads(args.config.read_text())
    if config.get('sequence', '').startswith('SET_'):
        raise ValueError('set the explicit mission/window in a copy of the preset first')
    validate_config(config)
    out = args.out.resolve()
    source_run = args.source_run.resolve()
    inside_out = ros1_path(out)
    inside_source = ros1_path(source_run)
    if out.exists():
        raise ValueError('output directory already exists')
    if (source_run/'ros_logs').exists():
        raise ValueError('start before source logging; do not attach this preset to a completed run')
    for source in config['sources']:
        source['log_glob'] = source['log_glob'].replace('${SOURCE_RUN}', inside_source)
    ros1 = config['ros1_container']
    receiver = config['receiver_container']
    binary = config['receiver_binary']
    for container in (ros1, receiver):
        result = subprocess.check_output(['docker', 'inspect', '--format', '{{.State.Running}}', container], text=True)
        if result.strip() != 'true':
            raise RuntimeError('container is not running: '+container)
    subprocess.run(['docker', 'exec', receiver, 'test', '-x', binary], check=True)
    check = ('source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI='+shlex.quote(config['ros_master_uri'])+
             ' && python3 -c '+shlex.quote("import numpy, rospy; import rosgraph; "
                                          "assert rosgraph.Master('/gnssfgo_preflight').getParam('/use_sim_time') is True"))
    subprocess.run(['docker', 'exec', ros1, 'bash', '-lc', check], check=True)
    print(json.dumps(dict(config=config, source_run=str(source_run), output=str(out),
                          note='Receiver only. Start native source runner after transport_ready.marker.'), indent=2), flush=True)
    if args.check:
        return
    out.mkdir(parents=True, exist_ok=False)
    (out/'resolved_config.json').write_text(json.dumps(config, indent=2)+'\n')
    transport_dir = out/'transport_code'
    transport_dir.mkdir()
    for name in ('live_receiver_protocol.py', 'grandtour_live_receiver_ros1.py'):
        shutil.copyfile(Path(__file__).parent/name, transport_dir/name)
    # Save implementation provenance without modifying any source workspace.
    manifest = dict(config_sha256=hashlib.sha256((out/'resolved_config.json').read_bytes()).hexdigest(),
                    source_run=str(source_run), started_wall_ns=time.time_ns())
    manifest['binary_sha256'] = subprocess.check_output(
        ['docker', 'exec', receiver, 'sha256sum', binary], text=True).split()[0]
    manifest['transport_sha256'] = {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in transport_dir.iterdir()}
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    fifo = out/'events.fifo'
    os.mkfifo(fifo, 0o600)
    jobs = []

    def launch(container, command, pid_path, logfile):
        setup = '/opt/ros/noetic/setup.bash' if container == ros1 else '/opt/ros/humble/setup.bash'
        script = 'set -e; source '+setup+'; printf "%s\\n" "$$" > '+shlex.quote(pid_path)+'; exec '+shlex.join(command)
        stream = (out/logfile).open('x')
        proc = subprocess.Popen(['docker', 'exec', container, 'bash', '-lc', script], stdout=stream, stderr=subprocess.STDOUT)
        jobs.append((container, proc, pid_path, stream))
        return proc

    def stop_owned():
        # Kill only this launcher’s saved PID if its command still contains its
        # unique output directory; never pkill sources, roscore, or other runs.
        code = ('import os,signal,sys; from pathlib import Path; p=Path(sys.argv[1]); '
                'pid=int(p.read_text()) if p.exists() else 0; '
                'c=Path("/proc/%d/cmdline"%pid); '
                'ok=pid>1 and c.exists() and sys.argv[2].encode() in c.read_bytes(); '
                'os.kill(pid,signal.SIGTERM) if ok else None')
        for container, proc, pid, stream in jobs:
            if proc.poll() is None:
                token = inside_out if container == ros1 else str(out)
                subprocess.run(['docker', 'exec', container, 'python3', '-c', code, pid, token], check=False)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.terminate()
            stream.close()

    try:
        command = [binary, '--event-stream', str(fifo), '--out', str(out/'receiver'),
                   '--mode', 'sources', '--timeline', 'record', '--solver', 'isam2',
                   '--support-policy', 'uniform_imu', '--support-period-ns', '100000000',
                   '--receiver-startup-samples', '59']
        if config.get('receiver_model', 'imu_wnoj') != 'imu_wnoj':
            command += ['--receiver-model', config['receiver_model']]
        if args.graph_snapshots:
            command += ['--graph-snapshots-out', str(out/'graph')]
        if config.get('receiver_environment'):
            command = ['env', *[key+'='+value for key, value in
                               sorted(config['receiver_environment'].items())], *command]
        solver = launch(receiver, command, str(out/'receiver.pid'), 'receiver.log')
        adapter = launch(ros1, ['python3', inside_out+'/transport_code/grandtour_live_receiver_ros1.py',
                               '--config', inside_out+'/resolved_config.json', '--out', inside_out,
                               '--fifo', inside_out+'/events.fifo'], inside_out+'/transport.pid', 'transport.log')
        print('Waiting for live events. Finish ONLY after sources drain: create '+str(out/'STOP'), flush=True)
        while solver.poll() is None or adapter.poll() is None:
            if (solver.poll() not in (None, 0) or adapter.poll() not in (None, 0)
                    or (out/'transport_failed.txt').exists()):
                raise RuntimeError('live receiver failed; inspect receiver.log / transport.log')
            time.sleep(0.2)
        if solver.returncode or adapter.returncode or not (out/'transport_complete.json').exists():
            raise RuntimeError('no certified transport completion')
        (out/'complete.marker').write_text('stream and adapter completed; validate source accounting before science\n')
    except BaseException as exc:
        (out/'failed.txt').write_text(str(exc)+'\n')
        raise
    finally:
        stop_owned()


if __name__ == '__main__':
    main()
