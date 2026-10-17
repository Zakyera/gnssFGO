"""Bounded Kimera-only ROS replay; owned process groups, no retries."""
import hashlib,json,os,signal,socket,subprocess,sys,time,xmlrpc.client
from pathlib import Path
import rosgraph
R=Path(__file__).resolve().parent
W=Path('/workspace/cbs_gtsam4.3')
mode=sys.argv[1]; assert mode in ['remove','retain']
if mode=='retain': assert json.loads((R/'remove_capture/validation.json').read_text())['valid']
D=R/(mode+'_capture')
if D.exists():
    assert list(D.iterdir())==[D/'kimera_logs'] and not list((D/'kimera_logs').iterdir()), 'refuse existing capture'
else:
    D.mkdir(); (D/'kimera_logs').mkdir()
assert (R/'test_passed.json').exists()
with socket.socket() as s:
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    s.bind(('127.0.0.1',11436))
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11436',ROS_HOSTNAME='127.0.0.1',
    ROS_LOG_DIR=str(D/'ros_logs'),FL4M_RETIREMENT=mode,FL4M_AUDIT=str(D/'objective.csv'),
    KIMERA_CBS_REVISION_AUDIT='1')
os.environ.pop('KIMERA_IMU_SAMPLE_SUPPORT',None)
os.environ['LD_LIBRARY_PATH']=str(R/'private_build')+':'+str(W/'build/kimera_vio')+':'+os.environ.get('LD_LIBRARY_PATH','')
handles=[]; files=[]; started=time.monotonic()
def start(args,name):
    f=(D/(name+'.log')).open('x');files.append(f)
    p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    handles.append(p);return p
def wait_ready(check,timeout=40):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        try:
            if check():return
        except Exception:pass
        if any(p.poll() is not None for p in handles):raise RuntimeError('owned process exited during readiness')
        time.sleep(.2)
    raise RuntimeError('readiness timeout')
def run(args):return subprocess.check_output(args,text=True)
master=rosgraph.Master('/fl4m_preflight')
try:
    core=start(['roscore','-p','11436'],'core')
    wait_ready(lambda:master.getPid()>0)
    run(['rosparam','set','/use_sim_time','true'])
    seed=start(['rostopic','pub','-r','20','/clock','rosgraph_msgs/Clock','{clock: {secs: 1727775007, nsecs: 0}}'],'clock')
    args=['roslaunch',str(R/'grandtour_eth1_kimera_stereo.launch'),
        'params_folder:='+str(R/'kimera_profile'),'log_output:=true','log_output_path:='+str(D/'kimera_logs'),
        'node_output:=screen','imu_topic:=/boxi/cpt7/imu','base_link_frame_id:=cpt7_imu',
        'cbs_belief_bridge_enable:=true','cbs_belief_publish_enable:=true','cbs_belief_receive_enable:=true',
        'cbs_backend_enable:=true','cbs_live_revision_enable:=false','cbs_odom_factor_mode:=active_window_temporary',
        'cbs_use_temporary_cbs_linear_factors:=false','cbs_odom_max_horizon_pairs_per_update:=1',
        'cbs_covariance_exchange_audit_max_samples:=40','cbs_covariance_exchange_audit_stride:=20',
        'cbs_active_factor_diagnostic_enable:=false','rerun_visualizer_enable:=false','rerun_factor_graph_inspector_enable:=false']
    launch=start(args,'kimera')
    node='/kimera_vio_ros/kimera_vio_ros_node'
    def ready():
        info=run(['rosnode','info',node])
        return all(t in info for t in ['/grandtour/front_left/image_raw','/grandtour/front_right/image_raw','/boxi/cpt7/imu','/kimera/cbs/odom_belief_in'])
    wait_ready(ready)
    uri=master.lookupNode(node);pid=xmlrpc.client.ServerProxy(uri).getPid('/fl4m_preflight')[2]
    argv=[a.decode() for a in Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0') if a]
    flags=dict(a[2:].split('=',1) for a in argv if a.startswith('--') and '=' in a)
    old=json.loads((W/'runs/fixed_lag_same_graph_fl4l_20261015/on_source_capture/effective_mode_check.json').read_text())
    expected=dict(a[2:].split('=',1) for a in old['kimera_argv'] if a.startswith('--') and '=' in a)
    for k,v in expected.items():
        if k.startswith('cbs_') or k in ['kimera_factor_graph_audit_enable','kimera_shadow_acceptance_audit_enable']:
            assert flags[k]==('false' if k=='cbs_experimental_native_imu_mean_alignment' else v),(k,flags.get(k),v)
    maps=Path('/proc/%d/maps'%pid).read_text()
    assert str(R/'private_build/libkimera_vio.so') in maps
    assert str(R/'private_build/libcbs.so') in maps
    assert 'libe1b_profile' not in maps
    env=dict(a.decode().split('=',1) for a in Path('/proc/%d/environ'%pid).read_bytes().split(b'\0') if b'=' in a)
    assert env['FL4M_RETIREMENT']==mode and env['FL4M_AUDIT']==str(D/'objective.csv')
    (D/'preflight.json').write_text(json.dumps(dict(argv=argv,flags=flags,maps=maps,pid=pid,launch=args,mode=mode),indent=2))
    (D/'resolved_params.yaml').write_text(run(['rosparam','get',node]))
    rec=start(['rosbag','record','--buffsize=256','-O',str(D/'outputs.bag'),'/kimera_vio_ros/odometry','/kimera/cbs/odom_belief_in'],'record')
    echo=start(['rostopic','echo','-p','/kimera_vio_ros/odometry'],'native')
    time.sleep(2)
    os.killpg(seed.pid,signal.SIGINT);seed.wait(timeout=10);handles.remove(seed)
    ds=W/'src/datasets/GrandTour';mission='2024-10-01-11-29-55'
    play=start(['rosbag','play','--quiet','--clock','--delay=2','--rate=.10','--duration=94',
        str(ds/(mission+'_hesai.bag')),str(ds/(mission+'_cpt7_imu.bag')),str(ds/(mission+'_alphasense.bag')),str(R/'aligned_inputs.bag'),
        '--topics','/boxi/cpt7/imu','/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed','/kimera/cbs/odom_belief_in'],'play')
    (D/'started.json').write_text(json.dumps(dict(pid=os.getpid(),play_pid=play.pid,start=time.time(),duration=94,mode=mode)))
    print('STARTED '+mode,flush=True)
    deadline=time.monotonic()+1150
    while play.poll() is None:
        assert launch.poll() is None and rec.poll() is None,'owned process exited'
        assert time.monotonic()<deadline,'playback timeout'
        time.sleep(.5)
    assert play.returncode==0
    handles.remove(play)
    time.sleep(25)
    assert launch.poll() is None
    (D/'complete.json').write_text(json.dumps(dict(play_exit=0,wall_sec=time.monotonic()-started,mode=mode)))
except BaseException as e:
    (D/'failed.txt').write_text(repr(e)+'\n');raise
finally:
    for p in reversed(handles):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
    for f in files:f.close()
print('COMPLETE '+mode,flush=True)
