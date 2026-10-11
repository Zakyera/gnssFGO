#!/usr/bin/env python3
"""Container-only no-data startup gate. Owns only its spawned process groups."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parent
ISO=Path('/workspace/source_huber_20261008')
BASE=Path('/workspace/cbs_gtsam4.3')

def main():
    out=ROOT/'startup_gate'
    out.mkdir(exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11390',ROS_IP='127.0.0.1',
        CBS_SOURCE_LOSS_AUDIT='1',KIMERA_CBS_REVISION_AUDIT='1')
    os.environ.pop('KIMERA_IMU_SAMPLE_SUPPORT',None)
    os.environ['LD_LIBRARY_PATH']=str(ISO/'devel/lib')+':'+str(ISO/'build/kimera_vio_opencv42')+':'+str(ISO/'install/lib')+':'+os.environ.get('LD_LIBRARY_PATH','')
    os.environ['CMAKE_PREFIX_PATH']=str(ISO/'devel')+':'+os.environ.get('CMAKE_PREFIX_PATH','')
    os.environ['ROS_PACKAGE_PATH']=str(ISO/'src/glim_ros1')+':'+str(ISO/'src/Kimera-VIO-ROS')+':'+os.environ.get('ROS_PACKAGE_PATH','')
    def call(command):
        return subprocess.run(command,capture_output=True,text=True,timeout=15)
    assert call(['rosparam','get','/rosversion']).returncode != 0, 'port occupied'
    # Require the corrected native library to match the original direct OpenCV ABI.
    deps={}
    for name,path in [('original',BASE/'devel/lib/libkimera_vio.so'),
                      ('corrected',ISO/'build/kimera_vio_opencv42/libkimera_vio.so')]:
        r=call(['readelf','-d',str(path)]); assert r.returncode==0
        deps[name]=[l.split('[')[1].split(']')[0] for l in r.stdout.splitlines() if 'NEEDED' in l and 'opencv' in l]
    assert sorted(deps['original'])==sorted(deps['corrected']),deps
    executable=BASE/'devel/lib/kimera_vio_ros/kimera_vio_ros_node'
    linked=call(['ldd',str(executable)])
    (out/'ldd.txt').write_text(linked.stdout+linked.stderr)
    assert linked.returncode==0 and 'not found' not in linked.stdout
    assert 'libopencv_core.so.4.2' in linked.stdout and '.so.3.4' not in linked.stdout
    profile=BASE/'runs/grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_cbs_off_full_346s_20260907'
    streams=[]; jobs=[]; cases={}
    def start(command,path):
        stream=path.open('x'); streams.append(stream)
        job=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        jobs.append(job); return job
    def stop(job):
        if job.poll() is None:
            os.killpg(job.pid,signal.SIGINT)
            try: job.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(job.pid,signal.SIGTERM); job.wait(timeout=10)
    try:
        master=start(['roscore','-p','11390'],out/'master.log')
        deadline=time.monotonic()+30
        while call(['rosparam','get','/rosversion']).returncode != 0:
            assert master.poll() is None and time.monotonic()<deadline
            time.sleep(.5)
        assert call(['rosparam','set','/use_sim_time','true']).returncode==0
        for loss in ['gaussian','huber']:
            case=out/loss; case.mkdir()
            os.environ.update(CBS_GLIM_SOURCE_LOSS=loss,CBS_KIMERA_SOURCE_LOSS=loss,
                CBSMS_RUN_NAME='startup_'+loss,ROS_LOG_DIR=str(case/'ros_logs'))
            enabled='true' if loss=='huber' else 'false'
            params=dict(glim_config_path=str(profile/'glim_config'),kimera_mode='stereo_shared_cpt7',
                kimera_params_folder=str(profile/'kimera_profile'),kimera_input_imu_topic='/boxi/cpt7/imu',
                kimera_body_frame_id='cpt7_imu',kimera_log_output_path=str(case/'kimera_logs'),
                cbs_enable=enabled,cbs_live_revision_enable='false',glim_publish_cbs_beliefs='true',
                kimera_publish_cbs_beliefs='true',glim_receive_cbs_beliefs=enabled,kimera_receive_cbs_beliefs=enabled,
                glim_cbs_mode='inject_persistent' if loss=='huber' else 'observe_only',kimera_cbs_backend_enable='true',
                enable_ekf='false',glim_factor_graph_inspector_enable='false',kimera_factor_graph_inspector_enable='false',
                cbs_odom_max_horizon_pairs_per_update='1',cbs_odom_publish_stride='1',cbs_odom_horizon_sec='.20',
                glim_export_endpoint_audit_enable='true',diagnostic_node_output='log',cbs_belief_receive_start_delay_sec='0')
            command=['roslaunch',str(ISO/'src/glim_ros1/launch/grandtour_eth1_glim_kimera_cbs.launch')]+[k+':='+v for k,v in params.items()]
            (case/'command.json').write_text(json.dumps(command,indent=2))
            launch=start(command,case/'launch.log')
            time.sleep(3)
            seed=call(['rostopic','pub','-1','/clock','rosgraph_msgs/Clock','{clock: {secs: 1727775007, nsecs: 0}}'])
            assert seed.returncode==0,seed.stderr
            deadline=time.monotonic()+70
            while True:
                assert launch.poll() is None, 'no-data estimator launch exited'
                g=call(['rosnode','info','/glim_rosnode'])
                k=call(['rosnode','info','/kimera_vio_ros/kimera_vio_ros_node'])
                if all(x in g.stdout for x in ['/boxi/hesai/points','/boxi/cpt7/imu']) and all(x in k.stdout for x in ['/grandtour/front_left/image_raw','/grandtour/front_right/image_raw','/boxi/cpt7/imu']): break
                assert time.monotonic()<deadline, 'no-data sensor subscription timeout'
                time.sleep(.5)
            maps=call(['python3',str(ROOT/'runtime_maps.py'),'startup_'+loss])
            (case/'maps.json').write_text(maps.stdout)
            assert maps.returncode==0,maps.stderr
            loaded=json.loads(maps.stdout)
            assert '--cbs_source_loss='+loss in loaded['kimera']['command']
            assert call(['rosparam','get','/glim_rosnode/glim_cbs_source_loss']).stdout.strip()==loss
            time.sleep(5)
            assert launch.poll() is None
            (case/'glim_node.txt').write_text(g.stdout)
            (case/'kimera_node.txt').write_text(k.stdout)
            cases[loss]=dict(passed=True,processes=loaded,received_sensor_data=False)
            stop(launch)
        result=dict(passed=True,data_played=False,cases=cases,opencv_dependencies=deps,
            corrected_binary_sha256=hashlib.sha256((ISO/'build/kimera_vio_opencv42/libkimera_vio.so').read_bytes()).hexdigest())
        (out/'validation.json').write_text(json.dumps(result,indent=2))
        print('NO-DATA STARTUP PASSED: Gaussian and Huber; no bag or sensor data')
    except BaseException as exc:
        (out/'failed.txt').write_text(repr(exc)+'\n'); raise
    finally:
        for job in reversed(jobs): stop(job)
        for stream in streams: stream.close()

if __name__=='__main__': main()
