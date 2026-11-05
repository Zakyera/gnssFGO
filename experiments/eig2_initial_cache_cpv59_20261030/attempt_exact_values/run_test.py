"""Once-only CP59 standalone cache ablation. Never launches source scripts."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import traceback

H=Path(__file__).resolve().parent
ROOT=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/ROOT.name/H.name
I='/workspace/cbs_gtsam4.3/runs/'+ROOT.name+'/'+H.name
OLD=ROOT.parent/'eig2_refinement_refresh_cpv58_20261030'
Q=W/'runs/eig2_hesai_insertion_cpv51_20261030'
J='/workspace/cbs_gtsam4.3/runs/'+Q.name
LIB='/workspace/cbs_gtsam4.3/runs/kimera_on_incremental_refresh_cpv40_20261029/private_build'

def save(name,data):
    with (H/name).open('x') as f:json.dump(data,f,indent=2)

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def check(pins):
    for p,h in pins.items():assert sha(p)==h,p

def call(args):return subprocess.check_output(args,text=True)

def run(label,args):
    save(label+'_command.json',args)
    print('Starting',label,flush=True)
    with (H/(label+'.log')).open('x') as f:
        p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(label+'_process.json',dict(pid=p.pid,time=time.time()))
        rc=p.wait()
    save(label+'_receipt.json',dict(returncode=rc,finished=time.time()))
    assert rc==0,(label,rc)
    print('Passed',label,flush=True)

save('start.json',dict(pid=os.getpid(),time=datetime.datetime.now().astimezone().isoformat()))
pins={}
for file in [OLD/'protected_manifest.json',OLD/'retained_manifest.json',ROOT/'protected_manifest.json']:
    for p,h in json.loads(file.read_text()).items():
        if p in pins:assert pins[p]==h,p
        pins[p]=h
for p in [H/'PROTOCOL.txt',Path(__file__),P/'Cache.cpp',OLD/'complete.json',ROOT/'failure.json',ROOT/'cleanup.json',ROOT/'ablation.log',ROOT/'initial_byte_identity.json',ROOT/'inspect_initial_bytes.py']:
    pins[str(p)]=sha(p)
save('protected_manifest.json',pins);check(pins)
states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
assert shutil.disk_usage(P).free>4*1024**3
save('preflight.json',dict(containers=states,protected=len(pins),free_bytes=shutil.disk_usage(P).free))
started=False
try:
    run('start_container',['docker','start','cbsms_ws_gpu']);started=True
    source='/workspace/cbs_gtsam4.3/runs/'+OLD.name
    for mode in ['release','san']:
        for stage in ['compile','link']:
            args=json.loads((OLD/f'{mode}_{stage}_command.json').read_text())
            args=[a.replace(source,I).replace('/Refresh.cpp','/Cache.cpp') for a in args]
            run(mode+'_'+stage,args)
        env=['-e','LD_LIBRARY_PATH='+LIB+':/workspace/cbs_gtsam4.3/devel/lib']
        if mode=='san':env+=['-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1']
        run(mode+'_selftest',['docker','exec',*env,'cbsms_ws_gpu',I+'/'+mode,'selftest'])
    run('ablation',['docker','exec','-e','LD_LIBRARY_PATH='+LIB+':/workspace/cbs_gtsam4.3/devel/lib',
                    'cbsms_ws_gpu',I+'/release',J+'/prepared/full_on_source/checkpoints/update_6',I+'/results'])
    for phase in ['ordinary','refresh']:
        native=Q/'prepared/full_on_source/checkpoints/update_6'/('after_'+phase)/'checkpoint.bin'
        assert sha(native)==sha(P/'results'/(phase+'_before.bin'))==sha(P/'results'/(phase+'_after.bin'))
    assert sha(P/'results/B_restored_before.bin')==sha(P/'results/ordinary_before.bin')
    check(pins)
    save('test_passed.json',dict(passed=True,protected=len(pins),snapshot_nonmutation=2,
                                only_factor571_initial_cache_changed=True,real_GN_steps=4))
except Exception:
    save('failure.json',dict(traceback=traceback.format_exc(),time=time.time()));raise
finally:
    if started:
        top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
        assert all(row.split()[1] in ['tail','zsh'] for row in top.splitlines()[1:] if row.strip()),top
        run('restore_container',['docker','stop','cbsms_ws_gpu'])
        states=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
        save('cleanup.json',dict(jobs=top,containers=states,time=time.time()))
