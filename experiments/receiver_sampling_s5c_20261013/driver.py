"""Create-once, UID1000, unchanged-executable sampling in pinned container."""
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
import fcntl

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
OLD=Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
EXE=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay')
CAPTURE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver')
LIBS=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib')
COLLECTOR=ROOT/'profiler/libgp-collector.so'
SAMPLER=ROOT/'profiler/libreceiver_sampler_deep.so'

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''): h.update(b)
    return h.hexdigest()

def env_for(profile):
    env=os.environ.copy()
    for k in list(env):
        if k.startswith(('SP_COLLECTOR','GPROFNG','S5B_','WNOJ_','E2_','E4_','CACHE_FAILURE_')) or k=='LD_PRELOAD': env.pop(k)
    env.update(LD_LIBRARY_PATH=str(LIBS),OMP_NUM_THREADS='1',WNOJ_STRICT_PUBLICATION='1',WNOJ_INCREMENTAL_PROFILE='nominal',E2_EXACT_LINEARIZATION_CACHE='1')
    if profile:
        if profile in ['fixture','fixture_founder']:
            directory=ROOT/(profile+'.er'); directory.mkdir()
            env.update(LD_PRELOAD=str(COLLECTOR),SP_COLLECTOR_EXPNAME=str(directory),
              SP_COLLECTOR_PARAMS='P:/usr;p:10000;S:1;a:off;',
              SP_COLLECTOR_FOUNDER='0',SP_COLLECTOR_ORIGIN_COLLECT='1',
              SP_COLLECTOR_LIBRARY_PATH=str(COLLECTOR.parent),SP_COLLECTOR_PRELOAD=str(COLLECTOR))
        else:
            directory=ROOT/(profile+'_profile');directory.mkdir()
            env.update(LD_PRELOAD=str(SAMPLER),S5C_SAMPLER_OUTPUT=str(directory),
                S5C_INTERVAL_NS='20000000' if profile in ['sample20','replay_sample20','fixture_deep8','prefix_deep8','replay_sample20_deep8'] else '10000000')
    return env

def main():
    global SAMPLER
    assert os.getuid()==1000
    tag=sys.argv[1]
    assert tag in ['fixture','fixture_founder','fixture_unwind','fixture_deep','prefix_reference','prefix_sample','prefix_deep','reference','sample','sample20','replay_sample','replay_sample20','fixture_deep8','prefix_deep8','replay_sample20_deep8']
    if tag.endswith('deep8'):SAMPLER=ROOT/'profiler/libreceiver_sampler_deep8.so'
    lock=(ROOT/'controller.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    assert not (ROOT/(tag+'.json')).exists() and not (ROOT/(tag+'.log')).exists(),'Refuse duplicate'
    assert sha(EXE)=='9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
    os.sched_setaffinity(0,{2})
    sampled='sample' in tag or tag.startswith('fixture') or tag in ['prefix_deep','prefix_deep8']
    if tag in ['reference','sample','sample20','replay_sample','replay_sample20']:
        assert sha(SAMPLER)==json.loads((ROOT/'fixture_deep.json').read_text())['collector_sha256']
        assert json.loads((ROOT/'prefix_deep_validation.json').read_text())['passed']
        assert json.loads((ROOT/'fixture_deep_sampling_validation.json').read_text())['passed']
        assert json.loads((ROOT/'prefix_deep_sampling_validation.json').read_text())['passed']
    if tag in ['sample','sample20']:
        assert json.loads((ROOT/'reference_validation.json').read_text())['passed']
    if tag=='sample20':
        assert json.loads((ROOT/'sample_validation.json').read_text())['passed']
    if tag in ['replay_sample','replay_sample20']:
        assert json.loads((ROOT/'offline_baseline_identity.json').read_text())['passed']
        assert not json.loads((ROOT/'reference_validation.json').read_text())['passed']
    if tag=='replay_sample20':
        assert json.loads((ROOT/'replay_sample_validation.json').read_text())['passed']
    if tag=='replay_sample20_deep8':
        assert json.loads((ROOT/'offline_baseline_identity.json').read_text())['passed']
        assert json.loads((ROOT/'replay_sample_validation.json').read_text())['passed']
        assert json.loads((ROOT/'fixture_deep8_sampling_validation.json').read_text())['passed']
        assert json.loads((ROOT/'prefix_deep8_sampling_validation.json').read_text())['passed']
        assert json.loads((ROOT/'prefix_deep8_validation.json').read_text())['passed']
        assert sha(SAMPLER)==json.loads((ROOT/'fixture_deep8.json').read_text())['collector_sha256']
    commands=[]
    if tag.startswith('fixture'):
        if tag=='fixture': commands.append(['c++','-O2','-g','-fno-omit-frame-pointer',str(HERE/'cpu_fixture.cpp'),'-o',str(ROOT/'cpu_fixture')])
        if tag in ['fixture_unwind','fixture_deep','fixture_deep8']:
            assert not SAMPLER.exists()
            commands.append(['c++','-std=c++17','-O2','-g','-shared','-fPIC',
                '-DS5C_MAX_FRAMES='+('8192' if tag=='fixture_deep8' else '4096'),
                str(HERE/'sampler.cpp'),'-o',str(SAMPLER),'-lunwind','-lrt'])
        cmd=[str(ROOT/'cpu_fixture')]
    else:
        output=ROOT/tag
        assert not output.exists()
        journal=OLD/'prefix5_valid.csv' if tag.startswith('prefix') else CAPTURE/'events.csv'
        cmd=[str(EXE),'--receiver-model','source_wnoa','--mode','sources','--timeline','record',
          '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
          '--receiver-startup-samples','59','--event-stream',str(journal),'--out',str(output)]
    commands.append(cmd)
    env=env_for(tag if sampled else None)
    start=time.monotonic()
    with (ROOT/(tag+'.log')).open('x') as log:
        code=0
        for command in commands:
            ce=env if command is cmd else env_for(None)
            actual=command
            if command is cmd and tag=='fixture_founder':
                base=env_for(None)
                extra={k:v for k,v in ce.items() if base.get(k)!=v}
                actual=[sys.executable,str(HERE/'collector_exec.py'),json.dumps(extra),*command]
                ce=base
            p=subprocess.Popen(actual,stdout=log,stderr=subprocess.STDOUT,env=ce)
            if command is cmd:
                with (ROOT/(tag+'_pid.json')).open('x') as f: json.dump(dict(pid=p.pid,controller=os.getpid(),command=command,start_unix_ns=time.time_ns()),f)
            try: code=p.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:p.kill();p.wait()
                code=124
            if code:break
    usage=resource.getrusage(resource.RUSAGE_CHILDREN)
    result=dict(passed=code==0,returncode=code,seconds=time.monotonic()-start,
      user_seconds=usage.ru_utime,system_seconds=usage.ru_stime,max_rss_kib=usage.ru_maxrss,
      commands=commands,affinity=sorted(os.sched_getaffinity(0)),binary_sha256=sha(EXE),
      collector_sha256=sha(SAMPLER if SAMPLER.exists() and tag not in ['fixture','fixture_founder'] else COLLECTOR),
      environment={k:v for k,v in env.items() if k.startswith(('SP_COLLECTOR','S5C_','WNOJ_','E2_','OMP_','LD_'))})
    with (ROOT/(tag+'.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    if code:raise SystemExit(code)

if __name__=='__main__':main()
