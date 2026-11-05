"""Private CP-K19 build/tests and saved-journal replay only; never starts sources."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

H = Path(__file__).resolve().parent
G = H.parent.parent
R = Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021')
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_batched_full_cpk18_20261021/prepared')
BASE = G/'experiments/fixed_lag_slots_fl3i_20261014'
RT = Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def save(p, value):
    with p.open('x') as f:
        json.dump(value, f, indent=2)

def verify():
    manifest=json.loads((R/'protected.json').read_text())
    for p,h in manifest.items():
        assert sha(p)==h, ('protected file changed',p)
    return len(manifest)

def env():
    e={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_', 'E2_', 'E2B_', 'S5B_', 'S5C_', 'E4_', 'CACHE_FAILURE')) and k!='LD_PRELOAD'}
    e.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0',
             ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    return e

def main():
    action=sys.argv[1]
    if action=='prepare':
        R.mkdir(exist_ok=False)
        protected=json.loads((P/'protected_manifest.json').read_text())
        for p,h in protected.items():
            assert sha(p)==h, ('existing protected mismatch',p)
        for p in [*BASE.glob('*.h'),*BASE.glob('*.cpp'),G/'experiments/fixed_lag_failure_fl4e_20261014/RecordedFixedLagSmoother.cpp',
                  G/'experiments/fixed_lag_live_fl4a_20261014/Live.cpp',P/'full_off_receiver/events.csv',
                  Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')]:
            protected[str(p)]=sha(p)
        save(R/'protected.json',protected)
        count=0;last=0
        # Preserve ALL retained event bytes/order; only a clearly labeled
        # diagnostic EOF is appended to this fresh copy, never to original.
        with (P/'full_off_receiver/events.csv').open('rb') as src,(R/'retained_prefix.events').open('xb') as dst:
            for line in src:
                assert line.endswith(b'\n') and line[:1] in (b'I',b'B')
                last=int(line.split(b',',2)[1]);dst.write(line);count+=1
            dst.write(f'E,{last}\n'.encode())
        save(R/'input.json',dict(original_events=count,synthetic_eof=True,last_arrival_ns=last,
             source_sha256=sha(P/'full_off_receiver/events.csv'),replay_sha256=sha(R/'retained_prefix.events')))
        print('prepared',count,'events',len(protected),'protected files');return
    with (R/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        n=verify();e=env();timeout=1200
        if action.startswith('build_'):
            target=action.removeprefix('build_')
            source={'live':'Live.cpp','legacy':'legacy_test.cpp','focused':'test_finalization.cpp','sanitize':'test_finalization.cpp'}[target]
            cmd=json.loads(Path('/home/yeranis/results/gnssfgo_fixed_lag_live_fl4a_20261014/build.json').read_text())['command']
            cmd=[x for x in cmd if not x.endswith('.cpp')]
            cmd[6:6]=['-I'+str(H),str(H/source),str(G/'experiments/fixed_lag_failure_fl4e_20261014/RecordedFixedLagSmoother.cpp'),str(BASE/'SlotISAM2.cpp')]
            # Legacy fixture includes comparisons using the separate FL3h class.
            if target!='live':cmd.insert(6,str(G/'experiments/fixed_lag_native_fl3h_20261014/RecordedFixedLagSmoother.cpp'))
            if target=='sanitize':
                cmd=[x for x in cmd if x not in ('-O3','-DNDEBUG')]
                cmd[1:1]=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
            cmd[-1]=str(R/target)
        elif action in ('test_legacy','test_focused','test_sanitize'):
            target=action.removeprefix('test_');cmd=[str(R/target)]
            build=json.loads((R/('build_'+target+'.json')).read_text())
            assert build['returncode']==0 and build['binary_sha256']==sha(R/target)
            assert build['sources']=={p.name:sha(p) for p in H.iterdir() if p.suffix in ('.h','.cpp')}
            e['FL3I_TEST_ROOT']=str(R/(action+'_outputs'))
        elif action=='replay':
            for x in ('test_legacy','test_focused','test_sanitize','build_live'):
                gate=json.loads((R/(x+'.json')).read_text())
                assert gate['returncode']==0
                assert gate['sources']=={p.name:sha(p) for p in H.iterdir() if p.suffix in ('.h','.cpp')}
            assert gate['binary_sha256']==sha(R/'live')
            assert json.loads((R/'input.json').read_text())['replay_sha256']==sha(R/'retained_prefix.events')
            cmd=[str(R/'live'),'--event-stream',str(R/'retained_prefix.events'),'--out',str(R/'replay'),
                 '--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu',
                 '--support-period-ns','100000000','--receiver-startup-samples','59','--receiver-model','source_wnoa',
                 '--absorbed-revision-policy','seal_at_absorption']
            timeout=1800
        else:raise ValueError(action)
        with (R/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        os.sched_setaffinity(0,{2});start=time.monotonic();error=None
        with (R/(action+'.log')).open('x') as log:
            try:rc=subprocess.run(cmd,env=e,stdout=log,stderr=subprocess.STDOUT,timeout=timeout).returncode
            except Exception as ex:rc=-1;error=repr(ex)
        result=dict(returncode=rc,error=error,seconds=time.monotonic()-start,command=cmd,protected_unchanged=verify(),
                    sources={p.name:sha(p) for p in H.iterdir() if p.suffix in ('.h','.cpp')})
        if rc==0:result['binary_sha256']=sha(cmd[-1] if action.startswith('build_') else cmd[0])
        save(R/(action+'.json'),result)
        print(action,rc,result['seconds'],flush=True);raise SystemExit(0 if rc==0 else 1)

if __name__=='__main__':main()
