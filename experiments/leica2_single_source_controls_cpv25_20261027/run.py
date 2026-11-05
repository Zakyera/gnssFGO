"""Exactly two single-source OFF receiver controls, unchanged original executable."""
import csv, hashlib, itertools, json, math, os, shutil, subprocess, time, traceback
from collections import Counter
from pathlib import Path

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_single_source_controls_cpv25_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
B=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')
LIB='/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2)
def status(stage):
    x=dict(stage=stage,time=time.time(),pid=os.getpid())
    with (H/'progress.jsonl').open('a') as f:f.write(json.dumps(x)+'\n')
    print(stage,flush=True)
def verify(pins):
    for p,digest in pins.items():assert sha(p)==digest,('input changed',p)
def normalized(path,source):
    rows=[]
    if not path.exists():return rows
    for line in path.read_text().splitlines():
        r=line.split(',')
        if r[0]=='belief' and r[2]==source:
            assert len(r)==55
            r.pop(12)
        elif r[0]=='absorbed' and r[1]==source:
            assert len(r)==8
            r.pop(6)
        else:continue
        rows.append(','.join(r))
    return rows

def validate(case,info):
    root=R/case;orig=P/'full_off_receiver/receiver'
    summary=json.loads((root/'summary.json').read_text())
    assert summary['complete'] and summary['publications']==summary['finalized']==4282
    assert summary['events']==info['events'] and summary['received']==info['counts']['B']
    assert summary['pending_at_end']==summary['unsupported_end']==0
    assert summary['model']=='source_wnoa_fixed_lag' and summary['lag_seconds']==7
    assert summary['absorbed_revision_policy']=='seal_at_absorption'
    for name in ('online_states.csv','finalized_states.csv'):
        with (orig/name).open() as fa,(root/name).open() as fb:
            n=0
            for x,y in itertools.zip_longest(csv.DictReader(fa),csv.DictReader(fb)):
                assert x is not None and y is not None
                for k in ('state_index','timestamp_ns','availability_ns'):assert x[k]==y[k],(case,name,k)
                assert all(math.isfinite(float(v)) for v in y.values())
                n+=1
            assert n==4282
    mapping=json.loads((R/(case+'_original_lines.json')).read_text())
    # Both archives are immutable. Map filtered event IDs to original events,
    # drop graph slot IDs only; retain all means/C/actions/physical/arrival times.
    counts=Counter();hashes=[hashlib.sha256(),hashlib.sha256()];commits=0
    for entry in os.scandir(root/'archive/commits'):
        event=int(entry.name);original=mapping[event-1]
        new=normalized(Path(entry.path)/'records.txt',info['source'])
        old=normalized(orig/'archive/commits'/str(original)/'records.txt',info['source'])
        assert new==old,(case,event,original,'retained lifecycle/payload mismatch')
        for a,b in zip(new,old):
            counts[a.split(',')[0]+':'+a.split(',')[1]]+=1
            hashes[0].update((str(original)+':'+a+'\n').encode())
            hashes[1].update((str(original)+':'+b+'\n').encode())
        commits+=1
    assert hashes[0].hexdigest()==hashes[1].hexdigest()
    # Buffered/pending B events may emit no commit until a later support.
    # Audit ALL input versions via eventual actions, not commit-directory count.
    accounted=sum(n for key,n in counts.items() if key.startswith('belief:') and key!='belief:sealed_reference')
    assert accounted==info['counts']['B'],(case,accounted,info['counts']['B'])
    save(H/(case+'_validation.json'),dict(pass_=True,summary=summary,commits=commits,
        retained_source_payload_action_sha256=hashes[0].hexdigest(),action_counts=dict(counts),
        exact_output_time_rows=8564,files_sha256={n:sha(root/n) for n in
        ('summary.json','online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum')}))

def main():
    save(H/'start.json',dict(time=time.time(),pid=os.getpid()))
    pins=json.loads((H/'input_pins.json').read_text());verify(pins)
    prepared=json.loads((H/'prepared.json').read_text())
    env=dict(os.environ)
    for k in list(env):
        if k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE','CPV')) or k=='LD_PRELOAD':env.pop(k)
    env.update(LD_LIBRARY_PATH=LIB,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',
               E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
    loaded=subprocess.run(['ldd',str(B)],env=env,text=True,capture_output=True,check=True)
    assert 'not found' not in loaded.stdout
    runtime={}
    for line in loaded.stdout.splitlines():
        for tok in line.split():
            if tok.startswith('/') and Path(tok).is_file():runtime[tok]=sha(tok)
    save(H/'runtime_pins.json',runtime)
    outcomes={}
    for case,info in prepared['cases'].items():
        assert shutil.disk_usage(R).free>8*2**30,'less than8GiB headroom'
        assert not (R/case).exists()
        assert sha(R/(case+'_events.csv'))==info['filtered_sha256']
        assert sha(R/(case+'_original_lines.json'))==info['mapping_sha256']
        status('replay_'+case)
        cmd=['taskset','-c','2',str(B),'--absorbed-revision-policy','seal_at_absorption',
             '--mode','sources','--timeline','record','--solver','isam2','--support-policy','uniform_imu',
             '--support-period-ns','100000000','--receiver-startup-samples','59','--receiver-model','source_wnoa',
             '--event-stream',str(R/(case+'_events.csv')),'--out',str(R/case)]
        began=time.time()
        with (H/(case+'_replay.log')).open('xb') as log:
            p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
            save(H/(case+'_launch.json'),dict(pid=p.pid,time=began,command=cmd,environment_overrides={
                k:env[k] for k in ('LD_LIBRARY_PATH','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','E2B_ENDPOINT_CACHE','E2_EXACT_LINEARIZATION_CACHE')}))
            # Read-only provenance snapshot; do not attach debugging or pause.
            time.sleep(.1)
            try:maps=Path(f'/proc/{p.pid}/maps').read_text()
            except FileNotFoundError:maps='process exited before maps snapshot'
            with (H/(case+'_maps.txt')).open('x') as f:f.write(maps)
            code=p.wait()
        save(H/(case+'_finish.json'),dict(returncode=code,seconds=time.time()-began))
        if code:
            outcomes[case]='failed_no_retry';status('failed_'+case)
            verify(pins);verify(runtime)
            continue
        status('validate_'+case);validate(case,info);outcomes[case]='complete_validated'
    verify(pins);verify(runtime)
    save(H/'replay_complete.json',dict(all_pass=all(x=='complete_validated' for x in outcomes.values()),
        outcomes=outcomes,input_pins=len(pins),runtime_pins=len(runtime),time=time.time()))
    status('complete')

if __name__=='__main__':
    try:main()
    except Exception:
        with (H/'failed.txt').open('x') as f:f.write(traceback.format_exc())
        raise
