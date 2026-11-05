"""One owned Kimera-only capture inside existing Noetic; no automatic retry."""
import csv,hashlib,json,os,signal,socket,subprocess,sys,time,xmlrpc.client
from pathlib import Path
import rosgraph

P=Path(__file__).resolve().parent
C=P/'capture';W=Path('/workspace/cbs_gtsam4.3')
PORT=11471;NODE='/kimera_vio_ros/kimera_vio_ros_node'
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
assert not C.exists(),'refuse duplicate capture'
for rel,want in json.loads((P/'prepared_manifest.json').read_text()).items():assert sha(P/rel)==want,rel
with socket.socket() as s:s.bind(('127.0.0.1',PORT))
C.mkdir();(C/'ros_logs').mkdir();(C/'kimera_logs').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11471',ROS_IP='127.0.0.1',
 ROS_LOG_DIR=str(C/'ros_logs'),KIMERA_CBS_REVISION_AUDIT='1',
 LD_LIBRARY_PATH=str(P/'private_build')+':'+str(W/'build/kimera_vio')+':'+os.environ.get('LD_LIBRARY_PATH',''))
os.environ.pop('KIMERA_IMU_SAMPLE_SUPPORT',None)
jobs={};streams=[];started=time.time()
def start(name,args):
 f=(C/(name+'.log')).open('x');streams.append(f)
 jobs[name]=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 return jobs[name]
def call(args):return subprocess.check_output(args,text=True)
def stop(name):
 job=jobs.get(name)
 if job is None or job.poll() is not None:return
 os.killpg(job.pid,signal.SIGINT)
 try:job.wait(timeout=45)
 except subprocess.TimeoutExpired:
  os.killpg(job.pid,signal.SIGTERM);job.wait(timeout=10)
  raise RuntimeError('shutdown timeout: '+name)
def wait_ready(test,timeout=80):
 deadline=time.monotonic()+timeout
 while True:
  assert all(jobs[n].poll() is None for n in jobs),'owned process exited at startup'
  try:
   if test():return
  except Exception:pass
  assert time.monotonic()<deadline,'startup deadline'
  time.sleep(.2)
try:
 start('master',['roscore','-p',str(PORT)])
 wait_ready(lambda:rosgraph.Master('/cpk21_check').getParam('/rosversion'))
 call(['rosparam','set','/use_sim_time','true'])
 args=['roslaunch',str(P/'kimera_only.launch'),
  'kimera_mode:=stereo_shared_cpt7','kimera_params_folder:='+str(P/'kimera_profile'),
  'kimera_log_output_path:='+str(C/'kimera_logs'),'kimera_input_imu_topic:=/boxi/cpt7/imu',
  'kimera_body_frame_id:=cpt7_imu','cbs_enable:=false','kimera_receive_cbs_beliefs:=false',
  'kimera_publish_cbs_beliefs:=true','kimera_cbs_backend_enable:=true','cbs_live_revision_enable:=false',
  'kimera_factor_graph_inspector_enable:=false','cbs_odom_max_horizon_pairs_per_update:=1',
  'cbs_odom_publish_stride:=1','cbs_odom_horizon_sec:=0.20','diagnostic_node_output:=log',
  'kimera_receiver_local_competition_audit_max_samples:=40','kimera_receiver_local_competition_audit_stride:=20']
 (C/'launch_args.json').write_text(json.dumps(args,indent=2))
 start('estimators',args)
 call(['rostopic','pub','-1','/clock','rosgraph_msgs/Clock','{clock: {secs: 1727775007, nsecs: 0}}'])
 master=rosgraph.Master('/cpk21_check')
 def sensor_ready():
  info=call(['rosnode','info',NODE])
  return all(t in info for t in ('/grandtour/front_left/image_raw','/grandtour/front_right/image_raw','/boxi/cpt7/imu'))
 wait_ready(sensor_ready)
 uri=master.lookupNode(NODE);code,_,pid=xmlrpc.client.ServerProxy(uri).getPid('/cpk21_check');assert code==1
 argv=[v.decode() for v in Path('/proc/%d/cmdline'%pid).read_bytes().split(b'\0') if v]
 flags=dict(v[2:].split('=',1) for v in argv if v.startswith('--') and '=' in v)
 original=json.loads((P/'reference_effective.json').read_text())
 for key,want in original['kimera'].items():assert flags.get(key)==want,(key,flags.get(key),want)
 maps=Path('/proc/%d/maps'%pid).read_text()
 assert str(P/'private_build/libkimera_vio.so') in maps
 assert sha(P/'private_build/libkimera_vio.so')=='be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9'
 env=dict(v.decode().split('=',1) for v in Path('/proc/%d/environ'%pid).read_bytes().split(b'\0') if b'=' in v)
 for key,want in original['kimera_candidate_env'].items():assert env[key]==want
 assert 'KIMERA_IMU_SAMPLE_SUPPORT' not in env
 params=master.getParam(NODE)
 assert params['cbs_belief_receive_enable'] is False and params['cbs_belief_publish_enable'] is True
 assert params['rerun_factor_graph_inspector_enable'] is False
 names=call(['rosnode','list']).splitlines()
 assert not any('glim' in n.lower() or 'gnss' in n.lower() or 'ekf' in n.lower() for n in names)
 (C/'effective.json').write_text(json.dumps(dict(passed=True,argv=argv,flags=flags,params=params,
   maps=maps,environment={k:env[k] for k in ('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT','LD_LIBRARY_PATH','KIMERA_CBS_REVISION_AUDIT')},
   nodes=names,checked_before_playback=True),indent=2))
 start('observer',['python3',str(P/'observe.py'),str(C)])
 wait_ready(lambda:(C/'observer_ready.marker').exists())
 start('odometry',['rostopic','echo','-p','/kimera_vio_ros/odometry'])
 start('recorder',['rosbag','record','--buffsize=64','-O',str(C/'diagnostic.bag'),
   '/kimera_vio_ros/odometry','/kimera_vio_ros/frontend_stats','/kimera/cbs/odom_belief_out'])
 time.sleep(2)
 base=W/'src/datasets/GrandTour';mission='2024-10-01-11-29-55'
 bag=['rosbag','play','--quiet','--clock','--delay=2.0','--rate=0.50','--start=0.0','--duration=346.32',
   str(base/(mission+'_cpt7_imu.bag')),str(base/(mission+'_alphasense.bag')),'--topics',
   '/boxi/cpt7/imu','/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed']
 (C/'bag_args.json').write_text(json.dumps(bag,indent=2))
 start('bag',bag)
 print('CP-K21 KIMERA ONLY STARTED: full346.32s at0.50x; effective settings passed',flush=True)
 with (C/'process_monitor.jsonl').open('x',buffering=1) as f:
  while jobs['bag'].poll() is None:
   for name in ('master','estimators','observer','odometry','recorder'):assert jobs[name].poll() is None,name+' stopped'
   assert time.time()-started<1100,'capture watchdog'
   f.write(json.dumps(dict(wall_ns=time.time_ns(),kimera_pid=pid,
     cpu_stat=Path('/proc/%d/stat'%pid).read_text()))+'\n')
   time.sleep(2)
 assert jobs['bag'].returncode==0,'bag failed'
 (C/'playback_complete.json').write_text(json.dumps(dict(wall_ns=time.time_ns(),returncode=0)))
 time.sleep(25)
 for name in ('recorder','odometry','observer','estimators'):stop(name)
 (C/'capture_complete.json').write_text(json.dumps(dict(seconds=time.time()-started,playback_s=346.32,
   playback_rate=.5,native_pose_coverage_not_assumed=True),indent=2))
 print('CAPTURE COMPLETE; separate gap analysis required',flush=True)
except BaseException as exc:
 (C/'failed.txt').write_text(repr(exc)+'\n');raise
finally:
 for name in ('bag','recorder','odometry','observer','estimators','master'):
  try:stop(name)
  except Exception as exc:print('cleanup:',repr(exc),flush=True)
 for f in streams:f.close()
