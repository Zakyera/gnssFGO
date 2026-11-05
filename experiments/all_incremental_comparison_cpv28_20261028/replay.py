"""Finite, exclusive saved-data CP-V28 observer replays; no estimator builds."""
import csv, hashlib, itertools, json, os, shutil, subprocess, tarfile, tempfile, time, traceback
from pathlib import Path

H=Path(__file__).resolve().parent
E=H.parent
H10=E/'first_published_comparison_cpv10_20261026/outputs'
H23=E/'leica2_incremental_full_cpv23_20261027'
EXE=Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027/live_publications')
LIB='/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib'
CASES=('hesai_eth1','hesai_eig2','hesai_spx3','hesai_leica2','velodyne_spx3','velodyne_eig2')
FILES=('online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum','summary.json','stream_progress.csv')
PINS={}

def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(2**20),b''):h.update(b)
    return h.hexdigest()
def pin(p):
    p=Path(p);PINS[str(p)]=sha(p);return p
def save(p,x):
    with Path(p).open('x') as f:json.dump(x,f,indent=2);f.write('\n')
def status(stage,**kw):
    d=dict(stage=stage,time=time.time(),pid=os.getpid(),**kw)
    with (H/'replay_progress.jsonl').open('a') as f:f.write(json.dumps(d)+'\n')
    print(json.dumps(d),flush=True)
def parity(original,replay,pub):
    hashes={n:sha(replay/n) for n in FILES[:4]}
    assert all(sha(original/n)==v for n,v in hashes.items()),'output parity'
    summary=json.loads((original/'summary.json').read_text())
    assert summary==json.loads((replay/'summary.json').read_text()),'summary parity'
    assert summary['complete'] and summary['publications']==summary['finalized']
    with (original/'stream_progress.csv').open() as fa,(replay/'stream_progress.csv').open() as fb:
        n=0
        for a,b in itertools.zip_longest(csv.DictReader(fa),csv.DictReader(fb)):
            assert a is not None and b is not None and a.keys()==b.keys()
            assert all(a[k]==b[k] for k in a if k not in ('processed_wall_ns','service_seconds','rss_kib')),(n,'event parity')
            n+=1
    assert n==summary['events']
    with pub.open() as f:rows=list(csv.DictReader(f))
    N=summary['publications'];assert len(rows)==2*N-1
    events={r['event'] for r in rows};assert len(events)==N
    for event in events:
        rel=Path('archive/commits')/event/'records.txt'
        assert sha(original/rel)==sha(replay/rel),(event,'publication commit parity')
    return dict(pass_=True,byte_identical_outputs=hashes,summary=summary,deterministic_events=n,
                publication_commits=len(events),snapshot_pose_rows=len(rows))
def archive(replay,target):
    manifest={str(p.relative_to(replay)):sha(p) for p in replay.rglob('*') if p.is_file()}
    assert manifest and not target.exists()
    with tarfile.open(target,'w:gz',compresslevel=1) as tf:tf.add(replay,arcname=replay.name)
    seen=set()
    with tarfile.open(target,'r:gz') as tf:
        for member in tf:
            if not member.isfile():continue
            rel=str(Path(member.name).relative_to(replay.name));assert rel not in seen
            h=hashlib.sha256()
            with tf.extractfile(member) as f:
                for b in iter(lambda:f.read(2**20),b''):h.update(b)
            assert h.hexdigest()==manifest[rel],rel
            seen.add(rel)
    assert seen==set(manifest)
    save(target.with_suffix('.manifest.json'),manifest)
    return dict(files=len(manifest),bytes=target.stat().st_size,sha256=sha(target),all_members_verified=True)
def main():
    save(H/'replay_start.json',dict(pid=os.getpid(),time=time.time(),cases=CASES))
    pin(__file__);pin(H/'PROTOCOL.txt');status('preflight')
    binary=json.loads(pin(H23/'binary.json').read_text());assert sha(pin(EXE))==binary['sha256']
    deps=json.loads(pin(H23/'dependency_pins.json').read_text())
    for path,digest in deps.items():assert sha(path)==digest,('compiled dependency changed',path)
    PINS.update(deps)
    # Pin runtime libraries against the observer's already verified provenance.
    oldpins=json.loads(pin(H23/'input_pins.json').read_text())
    for path,digest in oldpins.items():
        if path.startswith(LIB+'/'):
            assert sha(path)==digest,('runtime library changed',path);PINS[path]=digest
    jobs=[]
    for case in CASES:
        old=json.loads(pin(H10/(case+'.json')).read_text())
        for mode in ('off','on'):
            root=Path(old['input_products'][mode+'_fusion_online']['path']).parent.parent
            cfg=json.loads(pin(root/'resolved_config.json').read_text())
            assert cfg['support_period_ns']==100000000 and cfg['startup_samples']==59
            assert cfg['receiver_model']=='source_wnoa' and cfg['receiver_fixed_lag_seconds']==7
            assert cfg['receiver_environment']=={'LD_LIBRARY_PATH':LIB,'OMP_NUM_THREADS':'1'}
            for n in FILES:pin(root/'receiver'/n)
            pin(root/'events.csv')
            assert PINS[str(root/'receiver/online_trajectory.tum')]==old['input_products'][mode+'_fusion_online']['sha256']
            jobs.append((case,mode,root))
    save(H/'replay_input_pins.json',PINS)
    scratch=Path(tempfile.mkdtemp(prefix='cpv28-method2-',dir='/run/user/1000'))
    bundle=scratch/'evidence';bundle.mkdir()
    save(H/'scratch.json',dict(path=str(scratch),initial_free=shutil.disk_usage(scratch).free))
    env=dict(os.environ)
    for k in list(env):
        if k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) or k=='LD_PRELOAD':env.pop(k)
    env.update(LD_LIBRARY_PATH=LIB,OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
    records=[]
    for case,mode,root in jobs:
        assert shutil.disk_usage(H).free>8*2**30,'SSD free <8GiB'
        assert shutil.disk_usage(scratch).free>2.75*2**30,'tmpfs free <2.75GiB'
        name=case+'_'+mode;replay=scratch/name;pub=bundle/(name+'_publications.csv')
        env['CPV23_PUBLICATION_FILE']=str(pub)
        cmd=['taskset','-c','2',str(EXE),'--absorbed-revision-policy','seal_at_absorption','--mode','sources',
             '--timeline','record','--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
             '--receiver-startup-samples','59','--receiver-model','source_wnoa','--event-stream',str(root/'events.csv'),'--out',str(replay)]
        status('replay',case=case,mode=mode);t=time.time()
        with (H/(name+'_replay.log')).open('xb') as log:
            p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            save(H/(name+'_launch.json'),dict(pid=p.pid,command=cmd,time=t));code=p.wait()
        save(H/(name+'_finish.json'),dict(returncode=code,seconds=time.time()-t));assert code==0,name
        status('parity',case=case,mode=mode)
        checks=parity(root/'receiver',replay,pub);save(H/(name+'_parity.json'),checks)
        status('archive',case=case,mode=mode)
        receipt=archive(replay,bundle/(name+'.tar.gz'));save(H/(name+'_archive.json'),receipt)
        # Only this freshly created replay, after complete archive verification.
        assert replay.parent==scratch and replay.name==name and not replay.is_symlink()
        shutil.rmtree(replay)
        records.append(dict(case=case,mode=mode,archive=receipt,publication_sha256=sha(pub)))
        status('case_complete',case=case,mode=mode,archive_bytes=receipt['bytes'])
    status('verify_inputs')
    for path,digest in PINS.items():assert sha(path)==digest,('input changed',path)
    needed=sum(p.stat().st_size for p in bundle.iterdir())
    assert shutil.disk_usage(H).free-needed>4*2**30,'insufficient persistent storage'
    dest=H/'replay_evidence';dest.mkdir()
    for src in bundle.iterdir():
        target=dest/src.name
        with src.open('rb') as fi,target.open('xb') as fo:shutil.copyfileobj(fi,fo)
        assert sha(target)==sha(src)
    save(H/'replay_complete.json',dict(pass_=True,time=time.time(),records=records,pins=len(PINS),persistent_bytes=needed,
         scratch=str(scratch),all_archive_members_verified=True))
    # Verified copies of all RAM evidence now exist in the permanent task root.
    assert scratch.parent==Path('/run/user/1000') and scratch.name.startswith('cpv28-method2-')
    shutil.rmtree(scratch)
    status('complete')

if __name__=='__main__':
    try:main()
    except Exception:
        with (H/'replay_failed.txt').open('x') as f:f.write(traceback.format_exc())
        raise
