"""One finite baseline then conditional covariance-only variant; no retry."""
import csv,fcntl,os,shutil,subprocess,time,traceback
from common import *
lock=(H/'replay.lock').open('x');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
assert read(H/'prepared.json')['passed'] and not (H/'replay_start.json').exists()
stage='admission'
C='ubuntu-jammy-humble-nvidia'
def guest_probe(out):
    code='''import hashlib,json,pathlib,sys
binary,out=sys.argv[1:];found=[]
for p in pathlib.Path('/proc').glob('[0-9]*'):
 try:
  args=(p/'cmdline').read_bytes().split(b'\\0')
  if args and args[0].decode()==binary and out.encode() in args:
   maps=(p/'maps').read_text();paths=sorted({x.split()[-1] for x in maps.splitlines() if len(x.split())>=6 and x.split()[-1].startswith('/')})
   found.append(dict(pid=int(p.name),proc_start=int((p/'stat').read_text().rsplit(')',1)[1].split()[19]),args=[x.decode() for x in args if x],mapped_sha256={x:hashlib.sha256(pathlib.Path(x).read_bytes()).hexdigest() for x in paths if pathlib.Path(x).is_file()}))
 except (FileNotFoundError,ProcessLookupError):pass
print(json.dumps(found))'''
    return json.loads(subprocess.check_output(['docker','exec',C,'python3','-c',code,str(B),str(out)],text=True))
def stop_guest(rec):
    code='''import os,pathlib,signal,sys
p=pathlib.Path('/proc')/sys.argv[1]
if p.exists():
 start=int((p/'stat').read_text().rsplit(')',1)[1].split()[19])
 assert start==int(sys.argv[2])
 os.kill(int(sys.argv[1]),signal.SIGTERM)'''
    subprocess.run(['docker','exec',C,'python3','-c',code,str(rec['pid']),str(rec['proc_start'])],check=True)
def identity(pid):
    p=Path('/proc')/str(pid)
    return dict(pid=pid,proc_start=int((p/'stat').read_text().rsplit(')',1)[1].split()[19]),
                command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode())
def containers():
    return subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
                                   'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
def status(**kw):
    v=dict(stage=stage,at=now(),**kw)
    temp=H/'status.pending';temp.write_text(json.dumps(v,indent=2));temp.replace(H/'status.json')
    print(json.dumps(v),flush=True)
def run(mode,events):
    global stage
    stage=mode;status(state='running')
    out=H/mode;assert not out.exists()
    cfg=read(R/'resolved_config.json');env=dict(os.environ,**cfg['receiver_environment'])
    args=['docker','exec','-e','LD_LIBRARY_PATH='+cfg['receiver_environment']['LD_LIBRARY_PATH'],
          '-e','OMP_NUM_THREADS=1',C,'bash',str(P/'receiver_cpk19.sh'),'--mode','sources','--timeline','record',
          '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
          '--receiver-startup-samples','59','--receiver-model','source_wnoa',
          '--event-stream',str(events),'--out',str(out)]
    with (H/(mode+'.log')).open('x') as log:
        job=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        save(mode+'_process.json',dict(started=now(),args=args,identity=identity(job.pid)))
        start=time.monotonic();maps_saved=False;guest=None
        while job.poll() is None:
            if not maps_saved:
                found=guest_probe(out)
                if found:
                    assert len(found)==1
                    guest=found[0]
                    top=subprocess.check_output(['docker','top',C,'-eo','pid,comm,args'],text=True)
                    host=[x for x in top.splitlines() if str(B) in x and str(out) in x and ' live ' in x]
                    assert len(host)==1,top
                    save(mode+'_loaded.json',dict(guest=guest,host_identity=identity(int(host[0].split()[0])),mapped_sha256=guest['mapped_sha256']))
                    maps_saved=True
            if time.monotonic()-start>1200 or shutil.disk_usage(H).free<5*2**30:
                if guest:stop_guest(guest)
                else:job.terminate()
                try:job.wait(timeout=20)
                except subprocess.TimeoutExpired:job.kill();job.wait()
                raise RuntimeError(mode+' timeout/storage guard; owned receiver stopped')
            time.sleep(1)
    save(mode+'_exit.json',dict(exit=job.returncode,finished=now(),seconds=time.monotonic()-start,
                               identity_exited=not (Path('/proc')/str(job.pid)).exists()))
    assert job.returncode==0 and maps_saved,(mode,job.returncode)
    assert not guest_probe(out)
    assert read(out/'summary.json')==read(R/'receiver/summary.json')
    return out

before=containers();assert before==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
active=subprocess.check_output(['ps','-eo','pid,comm,args'],text=True)
bad=[x for x in active.splitlines()[1:] if len(x.split())>=3 and
     (x.split()[1] in {'live','roscore','rosmaster','glim_rosnode','evo_ape','evo_rpe'} or
      (x.split()[1].startswith('python') and any(q in x for q in ('controller.py','evaluate.py','validate_case.py'))))]
assert not bad,bad
save('replay_start.json',dict(started=now(),identity=identity(os.getpid()),containers=before,no_competing_jobs=True))
started=False
try:
    check(read(H/'input_sha256.json'))
    prep=read(H/'prepared.json');assert sha(H/'variant_events.csv')==prep['variant_events_sha256']
    started=True;subprocess.run(['docker','start',C],check=True,stdout=subprocess.DEVNULL)
    allowed={'tail','bash','zsh','ros2','rmw_zenohd'};deadline=time.monotonic()+30
    while True:
        top=subprocess.check_output(['docker','top',C,'-eo','pid,comm,args'],text=True)
        extra=[x for x in top.splitlines()[1:] if x.split()[1] not in allowed]
        if not extra:break
        assert time.monotonic()<deadline,top
        time.sleep(1)
    save('container_admission.json',dict(passed=True,top=top,at=now()))
    cfg=read(R/'resolved_config.json')
    ldd=subprocess.check_output(['docker','exec','-e','LD_LIBRARY_PATH='+cfg['receiver_environment']['LD_LIBRARY_PATH'],C,'ldd',str(B)],text=True)
    assert 'not found' not in ldd
    with (H/'container_ldd.txt').open('x') as f:f.write(ldd)
    baseline=run('baseline',R/'events.csv')
    stage='baseline_parity';status(state='checking')
    same=[]
    for name in ('online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum','summary.json'):
        assert sha(baseline/name)==sha(R/'receiver'/name),name
        same.append(name)
    archive=tree(baseline/'archive');old=read(H/'original_archive_sha256.json')
    assert archive==old,'baseline archive differs'
    ignored={'processed_wall_ns','service_seconds','rss_kib'}
    with (baseline/'stream_progress.csv').open() as f,(R/'receiver/stream_progress.csv').open() as g:
        n=0
        for a,b in zip(csv.DictReader(f),csv.DictReader(g),strict=True):
            assert {k:v for k,v in a.items() if k not in ignored}=={k:v for k,v in b.items() if k not in ignored}
            n+=1
    assert n==237614
    save('baseline_parity.json',dict(passed=True,finished=now(),byte_identical_products=same,
        byte_identical_archive_files=len(archive),archive_digest=tree_digest(archive),
        identical_logical_progress_rows=n,ignored_progress_fields=sorted(ignored)))
    variant=run('frozen_kimera',H/'variant_events.csv')
    assert read(H/'baseline_loaded.json')['mapped_sha256']==read(H/'frozen_kimera_loaded.json')['mapped_sha256']
    check(read(H/'input_sha256.json'))
    stage='replays_exited';status(state='complete')
except BaseException as exc:
    save('failed.json',dict(stage=stage,error=repr(exc),traceback=traceback.format_exc(),at=now()))
    status(state='failed',error=repr(exc));raise
finally:
    if started:
        for mode in ('baseline','frozen_kimera'):
            for owned in guest_probe(H/mode):stop_guest(owned)
        deadline=time.monotonic()+30
        while True:
            top=subprocess.check_output(['docker','top',C,'-eo','pid,comm,args'],text=True)
            extra=[x for x in top.splitlines()[1:] if x.split()[1] not in {'tail','bash','zsh','ros2','rmw_zenohd'}]
            if not extra:break
            assert time.monotonic()<deadline,('jobs remain; refusing container stop',top)
            time.sleep(1)
        subprocess.run(['docker','stop',C],check=True,stdout=subprocess.DEVNULL)
    assert containers()==before
    save('container_restore.json',dict(passed=True,at=now(),containers=before))
save('replay_complete.json',dict(passed=True,finished=now(),receiver_replays=2,source_runs=0,
                               containers=before,variant_only_after_exact_baseline_parity=True))
