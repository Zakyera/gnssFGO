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
from prepare import ROOT, REPO

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['off_gaussian','on_gaussian','on_huber'])
    args = parser.parse_args()
    tag = base_tag = args.mode
    assert json.loads((ROOT/'preflight.json').read_text())['passed']
    for previous in {'off_gaussian':[], 'on_gaussian':['off_gaussian'],
                     'on_huber':['off_gaussian','on_gaussian']}[tag]:
        assert json.loads((ROOT/(previous+'_validation.json')).read_text())['passed']
    for name, expected in json.loads((ROOT/'protected_manifest.json').read_text()).items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected, name
    source = ROOT/(tag+'_source')
    receiver = ROOT/(tag+'_receiver')
    assert not source.exists() and not receiver.exists()
    own = ROOT/(tag+'_control')
    own.mkdir(exist_ok=False)
    inside = '/workspace/'+str(own.relative_to('/home/yeranis/repos/V4RL'))
    env_inside = '/workspace/'+str((ROOT/'source.env').relative_to('/home/yeranis/repos/V4RL'))
    master_uri = 'http://127.0.0.1:11389'
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
        token = '11389' if name=='master' else str(source.name)
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
        master_script = 'source /opt/ros/noetic/setup.bash; echo $$ > '+shlex.quote(inside+'/master.pid')+'; exec roscore -p 11389'
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
        duration=30
        loss='huber' if tag=='on_huber' else 'gaussian'
        source_script = ('export CBS_GLIM_SOURCE_LOSS='+loss+' CBS_KIMERA_SOURCE_LOSS='+loss+'; export CBSMS_RUN_NAME='+tag+'_source CBSMS_CBS_ENABLE='+('false' if tag=='off_gaussian' else 'true')+
          ' CBSMS_BAG_DURATION_SEC='+str(duration)+'; source '+shlex.quote(env_inside)+
          '; echo $$ > '+shlex.quote(inside+'/source.pid')+
          '; exec bash /workspace/cbs_gtsam4.3/runs/source_huber_eth1_30s_opencv42_20261008/source_runner.sh')
        start('source',['docker','exec','cbsms_ws_gpu','setsid','--wait','bash','-lc',source_script])
        samples = (own/'monitor.jsonl').open('x',buffering=1)
        streams.append(samples)
        print(f'STARTED {tag}: native GLIM, native Kimera and WNOA receiver live at 0.1x',flush=True)
        while jobs['source'].poll() is None:
            assert jobs['receiver'].poll() is None, 'receiver stopped during playback'
            assert time.time()-started < duration*10+600, 'source deadline exceeded'
            if not (own/'runtime_maps.json').exists():
                maps=ros('python3 /workspace/cbs_gtsam4.3/runs/source_huber_eth1_30s_opencv42_20261008/runtime_maps.py '+tag+'_source',check=False)
                assert maps.returncode in (0,2), maps.stderr
                if maps.returncode==0:
                    (own/'runtime_maps.json').write_text(maps.stdout)
            status_path=receiver/'transport_status.json'
            if status_path.exists():
                try:
                    status=json.loads(status_path.read_text())
                    status.update(wall_ns=time.time_ns(),source_running=True)
                    samples.write(json.dumps(status)+'\n')
                except json.JSONDecodeError:
                    pass
            time.sleep(2)
        assert jobs['source'].returncode==0 and (source/'complete.marker').exists(), 'source failed'
        assert (own/'runtime_maps.json').exists(), 'both isolated source library maps were not verified'
        (receiver/'STOP').write_text('source process exited successfully; drain all captured revisions\n')
        deadline=time.monotonic()+650
        while jobs['receiver'].poll() is None:
            assert time.monotonic()<deadline, 'receiver drain deadline exceeded'
            time.sleep(1)
        assert jobs['receiver'].returncode==0 and (receiver/'complete.marker').exists(), 'receiver failed'
        (own/'complete.json').write_text(json.dumps(dict(seconds=time.time()-started, duration=30,mode=args.mode),indent=2))
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
