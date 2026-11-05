#!/usr/bin/env python3
"""Finite private build + one full saved-journal diagnostic replay per mode."""
import csv, hashlib, json, os, shutil, subprocess, time, traceback
from pathlib import Path

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/results/gnssfgo_leica2_receiver_trace_cpv13_20261026')
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
OLD = H.parent / 'receiver_finalization_cpk19_20261021'
BUILD = Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/build_live.json')
LIB = '/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib'

def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(2**20), b''): h.update(b)
    return h.hexdigest()

def save(p, data):
    with open(p, 'x') as f: json.dump(data, f, indent=2); f.write('\n')

def status(stage):
    print(time.strftime('%Y-%m-%dT%H:%M:%S'), stage, flush=True)
    with open(H/'progress.jsonl', 'a') as f:
        f.write(json.dumps(dict(stage=stage, time=time.time(), pid=os.getpid()))+'\n')

def disk():
    assert shutil.disk_usage(R.parent).free > 8*2**30, 'less than 8GiB headroom'

def check_mode(mode):
    orig = P/f'full_{mode}_receiver'/'receiver'
    replay = R/mode
    pairs = {}
    for name in ['online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum']:
        a,b=sha(orig/name),sha(replay/name)
        assert a==b, (mode,name,'output parity failed',a,b)
        pairs[name]=a
    a=json.loads((orig/'summary.json').read_text())
    b=json.loads((replay/'summary.json').read_text())
    assert a==b, (mode,'summary parity')
    n=0
    with open(orig/'stream_progress.csv') as fa, open(replay/'stream_progress.csv') as fb:
        aa,bb=csv.DictReader(fa),csv.DictReader(fb)
        for x,y in zip(aa,bb):
            for k in x:
                if k not in ['processed_wall_ns','service_seconds','rss_kib']:
                    assert x[k]==y[k], (mode,n,k,x[k],y[k])
            n+=1
        assert next(aa,None) is None and next(bb,None) is None
    assert n==a['events']
    events=set()
    with open(R/f'{mode}_trace'/'trace_meta.csv') as f:
        for row in csv.DictReader(f): events.add(int(row['event']))
    assert events
    commits=0
    for event in sorted(events):
        path=Path('archive/commits')/str(event)/'records.txt'
        assert sha(orig/path)==sha(replay/path), (mode,event,'commit parity')
        commits+=1
    result=dict(pass_=True,byte_identical_outputs=pairs,summary=a,
                all_event_metadata_equal=n,trace_commits_equal=commits,
                trace_event_range=[min(events),max(events)])
    save(H/f'{mode}_parity.json',result)

def main():
    disk()
    assert not R.exists(), 'fresh output root required'
    R.mkdir()
    save(H/'start.json',dict(pid=os.getpid(),time=time.time(),output=str(R)))
    status('pin_inputs')
    pins=json.loads((P/'protected_manifest.json').read_text())
    assert isinstance(pins,dict)
    for path,expected in pins.items():
        assert sha(path)==expected, ('protected input changed',path)
    for mode in ['off','on']:
        src=P/f'full_{mode}_receiver'
        for p in [src/'events.csv', src/'resolved_config.json', *[src/'receiver'/n for n in
                  ['online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum','summary.json','stream_progress.csv']]]:
            pins[str(p)]=sha(p)
    for p in [BUILD,OLD/'Receiver.h',OLD/'Live.cpp',H/'Receiver.h',H/'Live.cpp',H/'Trace.h',H/'run.py',H/'PROTOCOL.txt']:
        pins[str(p)]=sha(p)
    save(H/'input_pins.json',pins)
    env=dict(os.environ)
    for key in list(env):
        if key.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) or key=='LD_PRELOAD':
            env.pop(key)
    env.update(LD_LIBRARY_PATH=LIB,OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
    old=json.loads(BUILD.read_text())
    cmd=[x.replace(str(OLD),str(H)) for x in old['command']]
    assert cmd[-2]=='-o'
    cmd[-1]=str(R/'live_trace')
    cmd += ['-MD','-MF',str(R/'dependencies.d')]
    status('build')
    start=time.time()
    with open(H/'build.log','xb') as log:
        p=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    save(H/'build.json',dict(command=cmd,returncode=p.returncode,seconds=time.time()-start))
    assert p.returncode==0,'private observer build failed'
    deps=(R/'dependencies.d').read_text().replace('\\\n',' ').split(':',1)[1].split()
    dep_pins={str(Path(x).resolve()):sha(x) for x in deps}
    save(H/'dependency_pins.json',dep_pins)
    save(H/'binary.json',dict(path=str(R/'live_trace'),sha256=sha(R/'live_trace')))
    for mode in ['off','on']:
        disk()
        status('replay_'+mode)
        trace=R/f'{mode}_trace';trace.mkdir()
        env['CPV13_TRACE_ROOT']=str(trace)
        cmd=['taskset','-c','2',str(R/'live_trace'), '--absorbed-revision-policy','seal_at_absorption',
             '--mode','sources','--timeline','record','--solver','isam2',
             '--support-policy','uniform_imu','--support-period-ns','100000000',
             '--receiver-startup-samples','59','--receiver-model','source_wnoa',
             '--event-stream',str(P/f'full_{mode}_receiver'/'events.csv'),'--out',str(R/mode)]
        start=time.time()
        with open(H/f'{mode}_replay.log','xb') as log:
            p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            save(H/f'{mode}_launch.json',dict(command=cmd,pid=p.pid,time=start))
            rc=p.wait()
        save(H/f'{mode}_finish.json',dict(returncode=rc,seconds=time.time()-start))
        assert rc==0,mode+' receiver replay failed'
        status('parity_'+mode)
        check_mode(mode)
    status('verify_pins')
    for path,expected in {**pins,**dep_pins}.items():
        assert sha(path)==expected, ('input changed',path)
    save(H/'replay_complete.json',dict(pass_=True,time=time.time(),input_pins=len(pins),dependency_pins=len(dep_pins)))
    status('replays_complete')

if __name__=='__main__':
    try: main()
    except Exception:
        with open(H/'failed.txt','x') as f: f.write(traceback.format_exc())
        raise
