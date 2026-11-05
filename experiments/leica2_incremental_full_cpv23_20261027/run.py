"""Finite private observer build and exactly one saved OFF/ON replay each."""
import csv, hashlib, itertools, json, os, shutil, subprocess, time, traceback
from pathlib import Path

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
OLD=H.parent/'receiver_finalization_cpk19_20261021'
BUILD=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/build_live.json')
LIB='/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib'

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for chunk in iter(lambda:f.read(2**20),b''):h.update(chunk)
    return h.hexdigest()

def save(p,x):
    with open(p,'x') as f:json.dump(x,f,indent=2);f.write('\n')

def status(stage):
    row=dict(stage=stage,time=time.time(),pid=os.getpid())
    with (H/'progress.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    print(stage,flush=True)

def disk():
    assert shutil.disk_usage(R.parent).free>8*2**30,'less than8GiB free'

def parity(mode):
    original=P/f'full_{mode}_receiver/receiver';replay=R/mode
    checks={}
    for n in ('online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum'):
        assert sha(original/n)==sha(replay/n),(mode,n)
        checks[n]=sha(replay/n)
    summary=json.loads((original/'summary.json').read_text())
    assert summary==json.loads((replay/'summary.json').read_text())
    assert summary['complete'] and summary['publications']==summary['finalized']==4282
    with (original/'stream_progress.csv').open() as fa,(replay/'stream_progress.csv').open() as fb:
        n=0
        for a,b in itertools.zip_longest(csv.DictReader(fa),csv.DictReader(fb)):
            assert a is not None and b is not None
            assert a.keys()==b.keys()
            for k in a:
                if k not in ('processed_wall_ns','service_seconds','rss_kib'):assert a[k]==b[k],(mode,n,k)
            n+=1
    assert n==summary['events']
    with (R/f'{mode}_publications.csv').open() as f:rows=list(csv.DictReader(f))
    assert len(rows)==8563
    events={r['event'] for r in rows};assert len(events)==4282
    for event in events:
        rel=Path('archive/commits')/event/'records.txt'
        assert sha(original/rel)==sha(replay/rel),(mode,event,'publication payload')
    save(H/f'{mode}_parity.json',dict(pass_=True,byte_identical_outputs=checks,summary=summary,
         deterministic_events=n,publication_commits=len(events),snapshot_pose_rows=len(rows)))

def main():
    disk();assert not R.exists(),'fresh root only';R.mkdir()
    save(H/'start.json',dict(pid=os.getpid(),time=time.time(),output=str(R)))
    before=(OLD/'Receiver.h').read_text();after=(H/'Receiver.h').read_text()
    expected=before.replace('#include <set>\n','#include <set>\n#include "PublicationObserver.h"\n')
    anchor='    const auto value=s.solver.estimate();rowState(out,s,value,s.newest,s.lastArrival,"online_incremental");\n'
    expected=expected.replace(anchor,anchor+'    cpv23::ObservePublication(value,s.event,s.newest,s.origin,s.lastArrival);\n')
    assert expected==after and before!=after
    assert (OLD/'Live.cpp').read_bytes()==(H/'Live.cpp').read_bytes()
    save(H/'source_delta.json',dict(exact_observer_only_delta=True,live_byte_identical=True,
         original_receiver_sha256=sha(OLD/'Receiver.h'),observer_receiver_sha256=sha(H/'Receiver.h')))
    status('pin_inputs')
    pins=json.loads((P/'protected_manifest.json').read_text())
    for path,expected_hash in pins.items():assert sha(path)==expected_hash,('protected input changed',path)
    for mode in ('off','on'):
        root=P/f'full_{mode}_receiver'
        for p in [root/'events.csv',root/'resolved_config.json',*[root/'receiver'/n for n in
                  ('online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum','summary.json','stream_progress.csv')]]:
            pins[str(p)]=sha(p)
    for p in [P/'protected_manifest.json',BUILD,OLD/'Receiver.h',OLD/'Live.cpp',H/'Receiver.h',H/'Live.cpp',H/'PublicationObserver.h',H/'run.py',H/'PROTOCOL.txt']:
        pins[str(p)]=sha(p)
    save(H/'input_pins.json',pins)
    env=dict(os.environ)
    for k in list(env):
        if k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) or k=='LD_PRELOAD':env.pop(k)
    env.update(LD_LIBRARY_PATH=LIB,OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
    cmd=[x.replace(str(OLD),str(H)) for x in json.loads(BUILD.read_text())['command']]
    assert cmd[-2]=='-o';cmd[-1]=str(R/'live_publications')
    cmd+=['-MD','-MF',str(R/'dependencies.d')]
    status('build');t=time.time()
    with (H/'build.log').open('xb') as log:p=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    save(H/'build.json',dict(command=cmd,returncode=p.returncode,seconds=time.time()-t))
    assert p.returncode==0,'private observer build failed'
    deps=(R/'dependencies.d').read_text().replace('\\\n',' ').split(':',1)[1].split()
    dep_pins={str(Path(p).resolve()):sha(p) for p in deps}
    save(H/'dependency_pins.json',dep_pins)
    save(H/'binary.json',dict(path=str(R/'live_publications'),sha256=sha(R/'live_publications')))
    for mode in ('off','on'):
        disk();status('replay_'+mode)
        env['CPV23_PUBLICATION_FILE']=str(R/f'{mode}_publications.csv')
        cmd=['taskset','-c','2',str(R/'live_publications'),'--absorbed-revision-policy','seal_at_absorption',
             '--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu',
             '--support-period-ns','100000000','--receiver-startup-samples','59','--receiver-model','source_wnoa',
             '--event-stream',str(P/f'full_{mode}_receiver/events.csv'),'--out',str(R/mode)]
        t=time.time()
        with (H/f'{mode}_replay.log').open('xb') as log:
            p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            save(H/f'{mode}_launch.json',dict(pid=p.pid,time=t,command=cmd))
            code=p.wait()
        save(H/f'{mode}_finish.json',dict(returncode=code,seconds=time.time()-t))
        assert code==0,mode+' replay failed'
        status('parity_'+mode);parity(mode)
    status('verify_inputs')
    for path,expected_hash in {**pins,**dep_pins}.items():assert sha(path)==expected_hash,('input changed',path)
    save(H/'replay_complete.json',dict(pass_=True,time=time.time(),input_pins=len(pins),dependency_pins=len(dep_pins)))
    status('complete')

if __name__=='__main__':
    try:main()
    except Exception:
        with (H/'failed.txt').open('x') as f:f.write(traceback.format_exc())
        raise
