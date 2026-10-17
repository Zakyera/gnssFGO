"""One Kimera-only OFF diagnostic; unchanged solver must fail as before."""
import json,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import rosgraph,yaml
R=Path(__file__).resolve().parent;W=Path('/workspace/cbs_gtsam4.3')
T=W/'runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared'
D=R/'off_capture';assert not D.exists();D.mkdir();(D/'kimera_logs').mkdir()
assert json.loads((R/'test_passed.json').read_text())['passed']
assert json.loads((R/'sanitizer_passed.json').read_text())['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11443))
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11443',ROS_HOSTNAME='127.0.0.1',
    ROS_LOG_DIR=str(D/'ros_logs'),FL4M_RETIREMENT='remove',FL4M_AUDIT=str(D/'objective.csv'),
    KIMERA_CBS_REVISION_AUDIT='1',FL4Q_ENABLE='1',FL4Q_LOG=str(D/'acceptance.csv'),
    FL4U_PROBE=str(D/'failed_decision.txt'))
os.environ.pop('KIMERA_IMU_SAMPLE_SUPPORT',None)
os.environ['LD_LIBRARY_PATH']=str(R/'private_build')+':'+str(W/'build/kimera_vio')+':'+os.environ.get('LD_LIBRARY_PATH','')
handles=[];files=[];started=time.monotonic()
def start(args,name):
    f=(D/(name+'.log')).open('x');files.append(f)
    p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);handles.append(p);return p
def run(args):return subprocess.check_output(args,text=True)
def ready(check):
    deadline=time.monotonic()+40
    while time.monotonic()<deadline:
        try:
            if check():return
        except Exception:pass
        if any(p.poll() is not None for p in handles):raise RuntimeError('owned startup process exited')
        time.sleep(.2)
    raise RuntimeError('readiness timeout')
master=rosgraph.Master('/fl4u_preflight')
try:
    start(['roscore','-p','11443'],'core');ready(lambda:master.getPid()>0)
    run(['rosparam','set','/use_sim_time','true'])
    seed=start(['rostopic','pub','-r','20','/clock','rosgraph_msgs/Clock','{clock: {secs: 1727775007, nsecs: 0}}'],'clock')
    args=['roslaunch',str(R/'grandtour_eth1_kimera_stereo.launch'),
        'params_folder:='+str(R/'kimera_profile'),'log_output:=false','log_output_path:='+str(D/'kimera_logs'),
        'node_output:=screen','imu_topic:=/boxi/cpt7/imu','base_link_frame_id:=cpt7_imu',
        'cbs_belief_bridge_enable:=true','cbs_belief_publish_enable:=true','cbs_belief_receive_enable:=false',
        'cbs_backend_enable:=true','cbs_live_revision_enable:=false','cbs_odom_factor_mode:=active_window_temporary',
        'cbs_use_temporary_cbs_linear_factors:=false','cbs_odom_max_horizon_pairs_per_update:=1',
        'cbs_covariance_exchange_audit_max_samples:=40','cbs_covariance_exchange_audit_stride:=20',
        'cbs_active_factor_diagnostic_enable:=false','rerun_visualizer_enable:=false','rerun_factor_graph_inspector_enable:=false']
    launch=start(args,'kimera');node='/kimera_vio_ros/kimera_vio_ros_node'
    ready(lambda:all(t in run(['rosnode','info',node]) for t in ['/grandtour/front_left/image_raw','/grandtour/front_right/image_raw','/boxi/cpt7/imu']))
    pid=xmlrpc.client.ServerProxy(master.lookupNode(node)).getPid('/fl4u_preflight')[2]
    argv=[a.decode() for a in Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0') if a]
    flags=dict(a[2:].split('=',1) for a in argv if a.startswith('--') and '=' in a)
    expected=json.loads((T/'live120_off_source/effective_mode_check.json').read_text())
    old=dict(a[2:].split('=',1) for a in expected['kimera_argv'] if a.startswith('--') and '=' in a)
    for k,v in old.items():
        if k not in ['flagfile','output_path']:assert flags[k]==v,(k,flags.get(k),v)
    resolved=run(['rosparam','get',node]);params=yaml.safe_load(resolved)
    assert params['cbs_belief_receive_enable'] is False and params['cbs_belief_publish_enable'] is True
    maps=Path('/proc/%d/maps'%pid).read_text()
    assert str(R/'private_build/libkimera_vio.so') in maps and str(R/'private_build/libcbs.so') in maps
    assert 'libe1b_profile' not in maps
    (D/'preflight.json').write_text(json.dumps(dict(passed=True,argv=argv,flags=flags,maps=maps,pid=pid,launch=args),indent=2))
    (D/'resolved_params.yaml').write_text(resolved)
    rec=start(['rosbag','record','--buffsize=256','-O',str(D/'outputs.bag'),'/kimera_vio_ros/odometry'],'record')
    start(['rostopic','echo','-p','/kimera_vio_ros/odometry'],'native')
    time.sleep(2);os.killpg(seed.pid,signal.SIGINT);seed.wait(timeout=10);handles.remove(seed)
    ds=W/'src/datasets/GrandTour';m='2024-10-01-11-29-55'
    play_args=['rosbag','play','--quiet','--clock','--delay=2.0','--rate=.10','--start=0','--duration=111',
        str(ds/(m+'_hesai.bag')),str(ds/(m+'_cpt7_imu.bag')),str(ds/(m+'_alphasense.bag')),
        '--topics','/boxi/hesai/points','/boxi/cpt7/imu',
        '/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']
    play=start(play_args,'play')
    (D/'started.json').write_text(json.dumps(dict(pid=os.getpid(),play_pid=play.pid,start=time.time(),duration_max_s=111,play_args=play_args)))
    print('STARTED ONE OFF DIAGNOSTIC; max111s,0.1x,Kimera only',flush=True)
    deadline=time.monotonic()+1350;offset=0;observed=False
    while play.poll() is None:
        with (D/'kimera.log').open() as log:log.seek(offset);fresh=log.read();offset=log.tell()
        if 'FL4Q no decreasing current-chart LM step' in fresh:
            assert (D/'failed_decision.txt').exists()
            assert 'done,1' in (D/'failed_decision.txt').read_text()
            observed=True;break
        for bad in ['FL4U','FL4Q','active_window_rollback','Smoother recovery','Persistent BPSAM recovery']:
            assert bad not in fresh,'unexpected diagnostic/source failure: '+bad
        assert launch.poll() is None and rec.poll() is None,'owned process exited'
        assert Path('/proc/%d/stat'%pid).exists(),'Kimera exited before diagnostic'
        assert time.monotonic()<deadline,'diagnostic deadline'
        time.sleep(.5)
    assert observed,'target failure not reproduced within bounded prefix'
    (D/'diagnostic_captured.json').write_text(json.dumps(dict(expected_exception_observed=True,wall_sec=time.monotonic()-started)))
    print('CAPTURED EXISTING FAILURE; solver unchanged; parity validation pending',flush=True)
except BaseException as exc:
    (D/'failed.txt').write_text(repr(exc)+'\n');raise
finally:
    for p in reversed(handles):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
    for f in files:f.close()
