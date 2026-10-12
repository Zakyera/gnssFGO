#!/usr/bin/env python3
"""Owned-process orchestration for one frozen live case; no automatic tuning."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import time
from experiment import ROOT, REPO, file_sha
import fcntl

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('phase', choices=['profile40'])
    parser.add_argument('mode', choices=['on'])
    parser.add_argument('--attempt',type=int,choices=[1],default=1)
    args = parser.parse_args()
    base_tag = args.phase+'_'+args.mode
    tag = base_tag + ('_attempt'+str(args.attempt) if args.attempt>1 else '')
    lock = (ROOT/'controller.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    assert json.loads((ROOT/'preflight/validation.json').read_text())['passed']
    # Reuse the completed matched OFF; rerun only failed ON with the tested diagnostic fix.
    assert json.loads((ROOT.parents[1]/'gnssfgo_eth1_covariance160_20261010/prepared/full160_off_validation.json').read_text())['passed']
    for name, expected in json.loads((ROOT/'protected_manifest.json').read_text()).items():
        assert file_sha(name)==expected, name
    source = ROOT/(tag+'_source')
    receiver = ROOT/(tag+'_receiver')
    assert not source.exists() and not receiver.exists()
    own = ROOT/(tag+'_control')
    own.mkdir(exist_ok=False)
    inside = '/workspace/'+str(own.relative_to('/home/yeranis/repos/V4RL'))
    env_inside = '/workspace/'+str((ROOT/'source.env').relative_to('/home/yeranis/repos/V4RL'))
    master_uri = 'http://127.0.0.1:11422'
    jobs = {}
    streams = []
    def start(name, command):
        stream = (own/(name+'.log')).open('x')
        streams.append(stream)
        jobs[name] = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        return jobs[name]
    def ros(code, check=True):
        return subprocess.run(['docker','exec','cbsms_ws_gpu','bash','-lc',
            'source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI='+master_uri+' && '+code],
            text=True, capture_output=True, check=check)
    def stop_pid(name):
        # The PID is recorded by this case, command must match the owned port/run.
        token = '11422' if name=='master' else str(source.name)
        script = ('import os,signal,sys; from pathlib import Path; p=Path(sys.argv[1]); '
          'pid=int(p.read_text()) if p.exists() else 0; c=Path("/proc/%d/cmdline"%pid); '
          'e=Path("/proc/%d/environ"%pid); '
          'ok=pid>1 and c.exists() and (sys.argv[2].encode() in c.read_bytes() or '
          '(e.exists() and (b"CBSMS_RUN_NAME="+sys.argv[2].encode()+b"\\x00") in e.read_bytes())); '
          'os.killpg(os.getpgid(pid),signal.SIGINT) if ok else None')
        subprocess.run(['docker','exec','cbsms_ws_gpu','python3','-c',script,
                        inside+'/'+name+'.pid',token],check=False)
    started = time.time()
    try:
        assert ros('rosparam get /rosversion',check=False).returncode != 0, 'port is already in use'
        master_script = 'source /opt/ros/noetic/setup.bash; echo $$ > '+shlex.quote(inside+'/master.pid')+'; exec roscore -p 11422'
        start('master',['docker','exec','cbsms_ws_gpu','setsid','--wait','bash','-lc',master_script])
        for _ in range(60):
            if ros('rosparam get /rosversion',check=False).returncode==0:
                break
            time.sleep(.5)
        ros('rosparam set /use_sim_time true')
        launcher = ['python3',str(REPO/'tools/launch_grandtour_live_receiver.py'),
                    '--config',str(ROOT/(base_tag+'.json')),'--source-run',str(source),'--out',str(receiver)]
        checked = subprocess.run(launcher+['--check'],capture_output=True,text=True)
        (own/'preflight.log').write_text(checked.stdout+checked.stderr)
        assert checked.returncode==0, checked.stderr
        start('receiver',launcher+['--start'])
        deadline=time.monotonic()+60
        while not (receiver/'transport_ready.marker').exists():
            assert jobs['receiver'].poll() is None, 'receiver startup failed'
            assert time.monotonic()<deadline, 'transport startup timeout'
            time.sleep(.2)
        duration=40
        source_script = ('export CBSMS_RUN_NAME='+tag+'_source CBSMS_CBS_ENABLE='+('true' if args.mode=='on' else 'false')+
          ' CBSMS_BAG_DURATION_SEC='+str(float(duration))+'; source '+shlex.quote(env_inside)+
          '; echo $$ > '+shlex.quote(inside+'/source.pid')+
          '; exec bash '+shlex.quote('/workspace/'+str((ROOT/'source_runner.sh').relative_to('/home/yeranis/repos/V4RL'))))
        start('source',['docker','exec','cbsms_ws_gpu','setsid','--wait','bash','-lc',source_script])
        samples = (own/'monitor.jsonl').open('x',buffering=1)
        streams.append(samples)
        print(f'STARTED {tag}: native GLIM, native Kimera and WNOA receiver live at 0.1x',flush=True)
        log_positions={}
        def provider_failures():
            for log in [source/'estimators.log',*source.glob('ros_logs/*/glim_rosnode-*-stdout.log')]:
                if not log.exists():continue
                with log.open() as f:
                    f.seek(log_positions.get(str(log),0));chunk=f.read();log_positions[str(log)]=f.tell()
                for token in ['SHARED_NATIVE_SHADOW_FAILURE','SHARED_NATIVE_SHADOW_SETUP_FAILURE','SHARED_NATIVE_PROVIDER_FAILURE']:
                    assert token not in chunk, token+': '+str(log)
        while jobs['source'].poll() is None:
            provider_failures()
            assert jobs['receiver'].poll() is None, 'receiver stopped during playback'
            assert time.time()-started < duration*10+600, 'source deadline exceeded'
            status_path=receiver/'transport_status.json'
            if status_path.exists():
                try:
                    status=json.loads(status_path.read_text())
                    status.update(wall_ns=time.time_ns(),source_running=True)
                    samples.write(json.dumps(status)+'\n')
                    assert status['queued']<1000 and status['max_queue']<1000, 'live queue gate exceeded'
                except json.JSONDecodeError:
                    pass
            time.sleep(2)
        provider_failures()
        assert jobs['source'].returncode==0 and (source/'complete.marker').exists(), 'source failed'
        (receiver/'STOP').write_text('source process exited successfully; drain all captured revisions\n')
        deadline=time.monotonic()+650
        while jobs['receiver'].poll() is None:
            assert time.monotonic()<deadline, 'receiver drain deadline exceeded'
            time.sleep(1)
        assert jobs['receiver'].returncode==0 and (receiver/'complete.marker').exists(), 'receiver failed'
        (own/'complete.json').write_text(json.dumps(dict(seconds=time.time()-started, phase=args.phase,mode=args.mode),indent=2))
        print('CAPTURE COMPLETE '+tag+'; separate validation required',flush=True)
    except BaseException as exc:
        (own/'failed.txt').write_text(str(exc)+'\n')
        raise
    finally:
        if 'source' in jobs and jobs['source'].poll() is None:
            stop_pid('source')
        if receiver.exists() and 'receiver' in jobs and jobs['receiver'].poll() is None:
            (receiver/'STOP').write_text('controller stopping; not a certification of source completion\n')
        if 'master' in jobs:
            stop_pid('master')
        for stream in streams:
            stream.close()

if __name__=='__main__':
    main()
