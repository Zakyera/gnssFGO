#!/usr/bin/env python3
"""CP-V16 finite Qc-tenth ablation; one full receiver replay per mode."""
import csv, hashlib, json, os, shutil, subprocess, time, traceback
from pathlib import Path

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/results/gnssfgo_leica2_motion_prior_tenth_cpv16_20261026')
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

def payloads(root):
    result={};counts={}
    for entry in os.scandir(root/'archive/commits'):
        event=int(entry.name); selected=[]
        with open(Path(entry.path)/'records.txt') as f:
            for line in f:
                row=line.rstrip('\n').split(',')
                if row[0]=='belief':
                    key=row[0]+'_'+row[1];row.pop(12) # slot only, never payload
                elif row[0]=='absorbed':
                    key='absorbed';row.pop(6)
                else: continue
                counts[key]=counts.get(key,0)+1
                selected.append(','.join(row))
        if selected:result[event]=hashlib.sha256('\n'.join(selected).encode()).hexdigest()
    digest=hashlib.sha256()
    for event,h in sorted(result.items()):digest.update((str(event)+':'+h+'\n').encode())
    return result,counts,digest.hexdigest()

def check_mode(mode):
    orig=P/f'full_{mode}_receiver'/'receiver';candidate=R/mode
    a=json.loads((orig/'summary.json').read_text())
    b=json.loads((candidate/'summary.json').read_text())
    for k in a:
        if k!='peak_slots':assert a[k]==b[k],(mode,'summary conservation',k,a[k],b[k])
    n=0
    with open(orig/'stream_progress.csv') as fa,open(candidate/'stream_progress.csv') as fb:
        aa,bb=csv.DictReader(fa),csv.DictReader(fb)
        for x,y in zip(aa,bb):
            for k in ['event','kind','arrival_ns','publications','newest','oldest','active','pending','supports']:
                assert x[k]==y[k],(mode,n,k,x[k],y[k])
            n+=1
        assert next(aa,None) is None and next(bb,None) is None
    assert n==a['events']
    state_rows={}
    for name in ['online_states.csv','finalized_states.csv']:
        n=0
        with open(orig/name) as fa,open(candidate/name) as fb:
            aa,bb=csv.DictReader(fa),csv.DictReader(fb)
            for x,y in zip(aa,bb):
                for k in ['state_index','timestamp_ns','availability_ns']:
                    assert x[k]==y[k],(name,k)
                assert all(__import__('math').isfinite(float(v)) for v in y.values())
                n+=1
            assert next(aa,None) is None and next(bb,None) is None
        assert n==4282;state_rows[name]=n
    status('full_payload_audit_'+mode)
    pa,ca,ha=payloads(orig);pb,cb,hb=payloads(candidate)
    assert pa==pb and ca==cb and ha==hb,(mode,'source payload/action conservation')
    save(H/f'{mode}_validation.json',dict(pass_=True,events=a['events'],
        exact_event_metadata_rows=a['events'],state_rows=state_rows,
        unchanged_source_payload_commits=len(pa),source_action_counts=ca,
        normalized_payload_sha256=ha,summary=b,baseline_peak_slots=a['peak_slots']))


def main():
    disk()
    assert not R.exists(), 'fresh output root required'
    R.mkdir()
    save(H/'start.json',dict(pid=os.getpid(),time=time.time(),output=str(R),qc_multiplier=.1))
    previous=H.parent/'leica2_receiver_trace_cpv13_20261026'
    assert json.loads((previous/'closeout.json').read_text())['pass_']
    assert (H/'Live.cpp').read_bytes()==(previous/'Live.cpp').read_bytes()
    assert (H/'Trace.h').read_bytes()==(previous/'Trace.h').read_bytes()
    original=(previous/'Receiver.h').read_text()
    assert (H/'Receiver.h').read_text()==original.replace(
        'return noiseModel::Diagonal::Variances(q);',
        'return noiseModel::Diagonal::Variances(0.1*q);')
    save(H/'source_delta.json',dict(pass_=True,changed='Qc covariance multiplied by0.1 only',
          unchanged=['Live.cpp','Trace.h'],baseline_receiver_sha256=sha(previous/'Receiver.h')))
    half=H.parent/'leica2_motion_prior_cpv14_20261026'
    half_close=json.loads((half/'closeout.json').read_text());assert half_close['pass_']
    for p in [half/'results.json',half/'score.py',half/'Receiver.h',quarter/'closeout.json',quarter/'results.json',quarter/'score.py',quarter/'Receiver.h']:
        assert sha(p)==half_close['artifact_sha256'][str(p)]
    quarter=H.parent/'leica2_motion_prior_quarter_cpv15_20261026'
    quarter_close=json.loads((quarter/'closeout.json').read_text());assert quarter_close['pass_']
    for p in [quarter/'results.json',quarter/'score.py',quarter/'Receiver.h']:
        assert sha(p)==quarter_close['artifact_sha256'][str(p)]
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
    for p in [BUILD,OLD/'Receiver.h',OLD/'Live.cpp',H/'Receiver.h',H/'Live.cpp',H/'Trace.h',H/'run.py',H/'PROTOCOL.txt',H/'audit.cpp',previous/'closeout.json',previous/'Receiver.h',half/'closeout.json',half/'results.json',half/'score.py',half/'Receiver.h']:
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
    status('build_model_audit')
    audit_cmd=[x.replace(str(H/'Live.cpp'),str(H/'audit.cpp')) for x in cmd]
    audit_cmd[audit_cmd.index('-o')+1]=str(R/'model_audit')
    audit_cmd[audit_cmd.index('-MF')+1]=str(R/'audit_dependencies.d')
    with open(H/'audit_build.log','xb') as log:
        p=subprocess.run(audit_cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
    save(H/'audit_build.json',dict(command=audit_cmd,returncode=p.returncode))
    assert p.returncode==0,'audit build failed'
    deps2=(R/'audit_dependencies.d').read_text().replace('\\\n',' ').split(':',1)[1].split()
    extra={str(Path(x).resolve()):sha(x) for x in deps2}
    save(H/'audit_dependency_pins.json',extra);dep_pins.update(extra)
    p=subprocess.run([str(R/'model_audit')],env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    save(H/'model_audit.json',dict(returncode=p.returncode,stdout=p.stdout,stderr=p.stderr))
    assert p.returncode==0,'model-isolation audit failed'
    audit=json.loads(p.stdout);assert audit['pass']
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
        status('validate_'+mode)
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
