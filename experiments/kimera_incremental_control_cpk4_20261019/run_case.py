"""One bounded native-incremental OFF control; no retries, no source edits."""
import hashlib,json,os,signal,socket,subprocess,time,xmlrpc.client,shutil
from pathlib import Path
import rosgraph,yaml
R=Path(__file__).resolve().parent;W=Path('/workspace/cbs_gtsam4.3')
K=W/'runs/kimera_failed_factor_cpk1_20261019';D=R/'off_capture'
assert not D.exists(),'Refuse duplicate capture'
for name,h in json.loads((R/'pre_capture_receipt.json').read_text())['manifest'].items():
    p=Path(name.replace('/home/yeranis/repos/V4RL','/workspace'))
    digest=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):digest.update(block)
    assert digest.hexdigest()==h,str(p)
assert json.loads((R/'preflight_tests.json').read_text())['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11455))
assert shutil.disk_usage(R).free>8*1024**3
D.mkdir();(D/'kimera_logs').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11455',ROS_HOSTNAME='127.0.0.1',
    ROS_LOG_DIR=str(D/'ros_logs'),FL4M_RETIREMENT='remove',FL4M_AUDIT=str(D/'objective.csv'),
    KIMERA_CBS_REVISION_AUDIT='1',FL4Q_ENABLE='0',FL4Q_LOG=str(D/'acceptance.csv'))
for key in ('KIMERA_IMU_SAMPLE_SUPPORT','CPK1_PROBE','FL4U_PROBE','FL4N_CAPTURE'):os.environ.pop(key,None)
os.environ['LD_LIBRARY_PATH']=str(K/'private_build')+':'+str(W/'build/kimera_vio')+':'+os.environ.get('LD_LIBRARY_PATH','')
handles=[];files=[];started=time.monotonic();pid=None
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
master=rosgraph.Master('/cpk4_preflight')
try:
    start(['roscore','-p','11455'],'core');ready(lambda:master.getPid()>0)
    run(['rosparam','set','/use_sim_time','true'])
    seed=start(['rostopic','pub','-r','20','/clock','rosgraph_msgs/Clock','{clock: {secs: 1727775007, nsecs: 0}}'],'clock')
    ref=json.loads((K/'off_capture/preflight.json').read_text())
    args=[s.replace(str(K),str(R)) for s in ref['launch']]
    launch=start(args,'kimera');node='/kimera_vio_ros/kimera_vio_ros_node'
    ready(lambda:all(t in run(['rosnode','info',node]) for t in ['/grandtour/front_left/image_raw','/grandtour/front_right/image_raw','/boxi/cpt7/imu']))
    pid=xmlrpc.client.ServerProxy(master.lookupNode(node)).getPid('/cpk4_preflight')[2]
    argv=[a.decode() for a in Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0') if a]
    flags=dict(a[2:].split('=',1) for a in argv if a.startswith('--') and '=' in a)
    for k,v in ref['flags'].items():
        if k not in ('flagfile','output_path'):assert flags[k]==v,(k,flags.get(k),v)
    resolved=run(['rosparam','get',node]);params=yaml.safe_load(resolved)
    assert params['cbs_belief_receive_enable'] is False and params['cbs_belief_publish_enable'] is True
    assert flags['cbs_odom_factor_mode']=='active_window_temporary' and flags['cbs_live_revision_enable']=='false'
    env=dict(a.decode().split('=',1) for a in Path('/proc/%d/environ'%pid).read_bytes().split(b'\0') if a)
    assert env['FL4Q_ENABLE']=='0' and env['FL4M_RETIREMENT']=='remove'
    assert 'KIMERA_IMU_SAMPLE_SUPPORT' not in env
    assert Path(env['FL4M_AUDIT']).resolve()==(D/'objective.csv').resolve(), ('objective path mismatch',env['FL4M_AUDIT'])
    assert env['KIMERA_CBS_REVISION_AUDIT']=='1', 'revision audit disabled'
    maps=Path('/proc/%d/maps'%pid).read_text()
    for lib in ('libkimera_vio.so','libcbs.so'):assert str(K/'private_build'/lib) in maps
    publishers=dict(master.getSystemState()[0])
    assert not publishers.get('/kimera/cbs/odom_belief_in')
    nodes=run(['rosnode','list']).splitlines()
    assert not any(('glim' in n.lower() or 'gnssfgo' in n.lower()) for n in nodes)
    selected_env={k:env[k] for k in ('FL4Q_ENABLE','FL4Q_LOG','FL4M_RETIREMENT','FL4M_AUDIT','KIMERA_CBS_REVISION_AUDIT','LD_LIBRARY_PATH')}
    (D/'preflight.json').write_text(json.dumps(dict(passed=True,argv=argv,flags=flags,maps=maps,pid=pid,launch=args,selected_env=selected_env,nodes=nodes),indent=2))
    (D/'resolved_params.yaml').write_text(resolved)
    rec=start(['rosbag','record','--buffsize=256','-O',str(D/'outputs.bag'),'/kimera_vio_ros/odometry'],'record')
    start(['rostopic','echo','-p','/kimera_vio_ros/odometry'],'native')
    time.sleep(2);os.killpg(seed.pid,signal.SIGINT);seed.wait(timeout=10);handles.remove(seed)
    old_play=json.loads((K/'off_capture/started.json').read_text())['play_args']
    assert '--duration=111' in old_play and '--rate=.10' in old_play
    play=start(old_play,'play')
    (D/'started.json').write_text(json.dumps(dict(pid=os.getpid(),play_pid=play.pid,start=time.time(),duration_max_s=111,play_args=old_play)))
    print('STARTED ONE CP-K4 OFF CONTROL: FL4Q=0, max111s,0.1x,Kimera only',flush=True)
    deadline=time.monotonic()+1350;offset=0;played_at=None;last_status=0
    while True:
        with (D/'kimera.log').open() as log:log.seek(offset);fresh=log.read();offset=log.tell()
        for bad in ('FL4Q no decreasing','CPK1','FL4U','active_window_rollback','Smoother recovery','Persistent BPSAM recovery','VioBackend::optimize failed','terminate called','process has died','Segmentation fault'):
            assert bad not in fresh,'source failure: '+bad
        assert launch.poll() is None and rec.poll() is None,'owned process exited'
        assert Path('/proc/%d/stat'%pid).exists(),'Kimera process exited'
        assert not (D/'acceptance.csv').exists(),'FL4q ran despite disabled selector'
        assert time.monotonic()<deadline,'control wall deadline'
        assert shutil.disk_usage(R).free>5*1024**3,'runtime disk reserve'
        if play.poll() is not None:
            assert play.returncode==0,('bag player failed',play.returncode)
            if played_at is None:played_at=time.monotonic()
            if time.monotonic()-played_at>=25:break
        if time.monotonic()-last_status>=10:
            rows=(D/'objective.csv').read_text().splitlines() if (D/'objective.csv').exists() else []
            status=dict(wall_sec=time.monotonic()-started,objective_rows=len(rows),last_update=float(rows[-1].split(',')[0]) if rows else None,play_done=played_at is not None,fl4q_enabled=False)
            (D/'status.json').write_text(json.dumps(status));last_status=time.monotonic()
        time.sleep(.5)
    (D/'capture_complete.json').write_text(json.dumps(dict(passed=True,wall_sec=time.monotonic()-started,bag_duration_s=111,fl4q_enabled=False)))
    print('CP-K4 CAPTURE COMPLETE; validation/scoring pending',flush=True)
except BaseException as exc:
    (D/'failed.txt').write_text(repr(exc)+'\n');raise
finally:
    for p in reversed(handles):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
    for f in files:f.close()
    (D/'cleanup.json').write_text(json.dumps({'owned_returncodes':[p.returncode for p in handles],'kimera_process_gone':pid is None or not Path('/proc/%d/stat'%pid).exists()}))
