import hashlib,json,os,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
BASE=HERE.parent/'fixed_lag_slots_fl3i_20261014'
sys.path.insert(0,str(BASE));import driver
ROOT=Path('/home/yeranis/results/gnssfgo_fixed_lag_live_fl4a_20261014')
PREP=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4a_20261014/prepared')
S5=PREP.parents[1]/'gnssfgo_shared_linearization_s5_live160_20261012/prepared'
def core():return {str(p):driver.sha(p) for p in BASE.iterdir() if p.suffix in ('.h','.cpp')}
def command(events,out):return [str(ROOT/'live'),'--event-stream',str(events),'--out',str(out),'--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59','--receiver-model','source_wnoa']
action=sys.argv[1]
if action=='prepare':
    # Read-only verification of the pinned source/runtime/profile identities.
    protected=json.loads((S5/'protected_manifest.json').read_text())
    for p,h in protected.items():assert driver.sha(p)==h,('historical input changed',p)
    protected.update(core())
    for p in [*PREP.rglob('*'),HERE/'Live.cpp',HERE/'PROTOCOL.txt']:
        if p.is_file() and '__pycache__' not in p.parts:protected[str(p)]=driver.sha(p)
    ROOT.mkdir(exist_ok=False)
    with (ROOT/'protected.json').open('x') as f:json.dump(protected,f,indent=2)
    with (PREP/'protected_manifest.json').open('x') as f:json.dump(protected,f,indent=2)
    print('protected',len(protected));sys.exit(0)
protected=json.loads((PREP/'protected_manifest_v2.json' if action=='repeat_live' else ROOT/'protected.json').read_text())
for p,h in protected.items():assert driver.sha(p)==h,('protected changed',p)
if action=='verify_prefix':
    count=0
    for p in (ROOT/'prefix/archive').rglob('*'):
        if p.is_file():
            q=driver.ROOT/'prefix_outputs'/p.relative_to(ROOT/'prefix/archive')
            assert q.is_file() and driver.sha(p)==driver.sha(q),str(p);count+=1
    assert count==sum(p.is_file() for p in (driver.ROOT/'prefix_outputs').rglob('*'))
    import csv
    sys.path.insert(0,'/home/yeranis/repos/gnssFGO/tools')
    from live_receiver_protocol import parse_online_pose
    with (ROOT/'prefix/online_states.csv').open() as f:
        header=next(f).strip().split(',');poses=[parse_online_pose(header,line.strip()) for line in f]
    summary=json.loads((ROOT/'prefix/summary.json').read_text());assert len(poses)==summary['publications']
    result=dict(passed=True,archive_files_byte_identical=count,published_rows=len(poses),core_hashes=core(),binary_sha256=driver.sha(ROOT/'live'),protected_count=len(protected))
    (PREP/'preflight').mkdir(exist_ok=False)
    with (PREP/'preflight/validation.json').open('x') as f:json.dump(result,f,indent=2)
    print(result);sys.exit(0)
with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
if action=='build':
    cmd=json.loads((driver.ROOT/'build_replay.json').read_text())['command'];cmd=[x for x in cmd if not x.endswith('.cpp')]
    cmd[6:6]=[str(HERE/'Live.cpp'),str(BASE/'RecordedFixedLagSmoother.cpp'),str(BASE/'SlotISAM2.cpp')];cmd[-1]=str(ROOT/'live')
elif action=='prefix':
    assert json.loads((ROOT/'build.json').read_text())['returncode']==0
    cmd=command(driver.FL3/'prefix20.events',ROOT/'prefix')
elif action=='repeat_live':
    assert (PREP/'pilot30_off_control_preflight2/complete.json').is_file()
    cmd=command(PREP/'pilot30_off_receiver/events.csv',ROOT/'repeat_live')
else:raise ValueError(action)
env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
env.update(LD_LIBRARY_PATH=str(driver.RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
os.sched_setaffinity(0,{2});started=time.monotonic()
with (ROOT/(action+'.log')).open('x') as f:rc=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=300).returncode
for p,h in protected.items():assert driver.sha(p)==h,p
result=dict(returncode=rc,seconds=time.monotonic()-started,command=cmd,core_hashes=core(),protected_unchanged=True)
with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
print(action,rc,result['seconds']);sys.exit(rc)
