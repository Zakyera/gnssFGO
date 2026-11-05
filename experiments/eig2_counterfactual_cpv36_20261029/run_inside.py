"""Single owned300s original-start Kimera OFF diagnostic; no automatic retry."""
import csv,hashlib,json,os,signal,socket,subprocess,sys,time,xmlrpc.client,shutil
from pathlib import Path
import rosgraph
P=Path(__file__).resolve().parent;C=P/'capture';W=Path('/workspace/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
NODE='/kimera_vio_ros/kimera_vio_ros_node';PORT=11536
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
assert not C.exists(),'refuse duplicate capture'
for name,s in json.loads((P/'prepared_manifest.json').read_text()).items():assert sha(P/name)==s,name
assert (P/'build_backend/passed.json').exists()
assert shutil.disk_usage(P).free>8*1024**3
with socket.socket() as s:s.bind(('127.0.0.1',PORT))
C.mkdir();(C/'ros_logs').mkdir();(C/'kimera_logs').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11536',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_LOG_DIR=str(C/'ros_logs'),KIMERA_CBS_REVISION_AUDIT='1',KIMERA_FIXED_LAG_SEMANTICS_AUDIT='1',CPV36_AUDIT_DIR=str(C/'snapshots'),LD_LIBRARY_PATH=str(P/'private_build')+':'+str(W/'build/kimera_vio')+':'+os.environ.get('LD_LIBRARY_PATH',''))
for k in ('KIMERA_IMU_SAMPLE_SUPPORT','FL4N_CAPTURE','CPK1_PROBE','FL4U_PROBE'):os.environ.pop(k,None)
jobs={};streams=[];started=time.monotonic();pid=None
def save(name,data):
    with (C/name).open('x') as f:json.dump(data,f,indent=2)
def start(name,args):
    f=(C/(name+'.log')).open('x');streams.append(f)
    jobs[name]=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    return jobs[name]
def call(args):return subprocess.check_output(args,text=True)
def stop(name):
    job=jobs.get(name)
    if job is None or job.poll() is not None:return
    os.killpg(job.pid,signal.SIGINT)
    try:job.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(job.pid,signal.SIGTERM);job.wait(timeout=10)
        raise RuntimeError('shutdown timeout: '+name)
def ready(test):
    deadline=time.monotonic()+80
    while True:
        assert all(j.poll() is None for j in jobs.values()),'startup process exited'
        try:
            if test():return
        except Exception:pass
        assert time.monotonic()<deadline,'startup deadline'
        time.sleep(.2)
try:
    save('start.json',dict(controller_pid=os.getpid(),wall_ns=time.time_ns(),duration=300,rate=.2))
    start('master',['roscore','-p',str(PORT)])
    master=rosgraph.Master('/cpv36_check');ready(lambda:master.getPid()>0)
    call(['rosparam','set','/use_sim_time','true'])
    args=['roslaunch',str(P/'kimera_only.launch'),'kimera_mode:=stereo_shared_cpt7','kimera_params_folder:='+str(P/'kimera_profile'),'kimera_log_output_path:='+str(C/'kimera_logs'),'kimera_input_imu_topic:=/boxi/cpt7/imu','kimera_body_frame_id:=cpt7_imu','cbs_enable:=false','kimera_receive_cbs_beliefs:=false','kimera_publish_cbs_beliefs:=true','kimera_cbs_backend_enable:=true','cbs_live_revision_enable:=false','kimera_factor_graph_inspector_enable:=false','cbs_odom_max_horizon_pairs_per_update:=1','cbs_odom_publish_stride:=1','cbs_odom_horizon_sec:=0.20','diagnostic_node_output:=log','kimera_receiver_local_competition_audit_max_samples:=40','kimera_receiver_local_competition_audit_stride:=20']
    save('launch_args.json',args);start('estimators',args)
    call(['rostopic','pub','-1','/clock','rosgraph_msgs/Clock','{clock: {secs: 1730638815, nsecs: 0}}'])
    ready(lambda:all(t in call(['rosnode','info',NODE]) for t in ('/grandtour/front_left/image_raw','/grandtour/front_right/image_raw','/boxi/cpt7/imu')))
    code,_,pid=xmlrpc.client.ServerProxy(master.lookupNode(NODE)).getPid('/cpv36_check');assert code==1
    argv=[x.decode() for x in Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0') if x]
    ref=json.loads((P/'reference_effective.json').read_text())
    def normalized(a):return [x for x in a if x.startswith('--') and not x.startswith(('--flagfile=','--output_path='))]
    assert normalized(argv)==normalized(ref['kimera_argv']),'effective argv differs'
    flags=dict(x[2:].split('=',1) for x in argv if x.startswith('--') and '=' in x)
    env=dict(x.decode().split('=',1) for x in Path('/proc/%d/environ'%pid).read_bytes().split(b'\0') if b'=' in x)
    for k,v in ref['kimera_candidate_env'].items():assert env[k]==v
    assert 'KIMERA_IMU_SAMPLE_SUPPORT' not in env
    assert env['CPV36_AUDIT_DIR']==str(C/'snapshots')
    maps=Path('/proc/%d/maps'%pid).read_text()
    for name in ('libkimera_vio.so','libcbs.so'):assert str(P/'private_build'/name) in maps,name
    params=master.getParam(NODE)
    assert not params['cbs_belief_receive_enable'] and params['cbs_belief_publish_enable']
    assert not params['rerun_factor_graph_inspector_enable']
    nodes=call(['rosnode','list']).splitlines();assert not any(any(x in n.lower() for x in ('glim','gnss','ekf')) for n in nodes)
    assert not dict(master.getSystemState()[0]).get('/kimera/cbs/odom_belief_in')
    save('effective.json',dict(passed=True,argv=argv,params=params,maps=maps,environment={k:env[k] for k in ('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT','CPV36_AUDIT_DIR','KIMERA_CBS_REVISION_AUDIT','KIMERA_FIXED_LAG_SEMANTICS_AUDIT','LD_LIBRARY_PATH')},nodes=nodes,checked_before_playback=True))
    start('observer',['python3',str(P/'observe.py'),str(C)]);ready(lambda:(C/'observer_ready.marker').exists())
    start('odometry',['rostopic','echo','-p','/kimera_vio_ros/odometry'])
    start('recorder',['rosbag','record','--buffsize=64','-O',str(C/'diagnostic.bag'),'/kimera_vio_ros/odometry','/kimera/cbs/odom_belief_out'])
    time.sleep(2)
    base=W/'src/datasets/GrandTour/EIG-2/2024-11-03-13-59-54_'
    bag=['rosbag','play','--quiet','--clock','--delay=2.0','--rate=0.20','--start=5.7','--duration=300',str(base)+'cpt7_imu.bag',str(base)+'alphasense.bag','--topics','/boxi/cpt7/imu','/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']
    save('bag_args.json',bag);start('bag',bag)
    save('owned_jobs.json',{n:j.pid for n,j in jobs.items()})
    print('ONE CPV36 OFF replay STARTED,300s at0.20x; actual flag parity passed',flush=True)
    offset=0
    while jobs['bag'].poll() is None:
        for n in ('master','estimators','observer','odometry','recorder'):assert jobs[n].poll() is None,n+' exited'
        assert time.monotonic()-started<2000,'capture watchdog'
        assert shutil.disk_usage(P).free>5*1024**3,'runtime storage reserve'
        with (C/'estimators.log').open() as f:f.seek(offset);fresh=f.read();offset=f.tell()
        for bad in ('active_window_rollback','Smoother recovery','VioBackend::optimize failed','terminate called','process has died','Segmentation fault','CPV36 duplicate','CPV36 snapshot write failed'):assert bad not in fresh,bad
        status=dict(wall_seconds=time.monotonic()-started,bag_running=True,kimera_pid=pid,free_bytes=shutil.disk_usage(P).free)
        tmp=C/'status.tmp';tmp.write_text(json.dumps(status));os.replace(tmp,C/'status.json')
        time.sleep(2)
    assert jobs['bag'].returncode==0,'bag failed'
    save('playback_complete.json',dict(wall_ns=time.time_ns(),returncode=0))
    time.sleep(25)
    for name in ('recorder','odometry','observer','estimators'):stop(name)
    save('capture_complete.json',dict(wall_seconds=time.monotonic()-started,duration=300,rate=.2,coverage_and_parity_pending=True))
    print('CPV36 capture complete; parity and diagnostic analysis pending',flush=True)
except BaseException as exc:
    (C/'failed.txt').write_text(repr(exc)+'\n');raise
finally:
    errors=[]
    for name in ('bag','recorder','odometry','observer','estimators','master'):
        try:stop(name)
        except Exception as exc:errors.append([name,repr(exc)])
    for f in streams:f.close()
    save('cleanup.json',dict(returncodes={n:j.poll() for n,j in jobs.items()},errors=errors,kimera_process_gone=pid is None or not Path('/proc/%d/stat'%pid).exists()))

