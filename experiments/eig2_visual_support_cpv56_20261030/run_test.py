"""Exclusive standalone visual support readout. No source or optimizer call."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import traceback

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name
C=Path('/workspace/cbs_gtsam4.3')
CK=C/'runs'/H.name
Q=W/'runs/eig2_hesai_insertion_cpv51_20261030'
CQ=C/'runs'/Q.name
H53=H.parent/'eig2_first_rotation_cpv53_20261030'

def save(name,data):
    with (H/name).open('x') as f: json.dump(data,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def verify(pins):
    for p,h in pins.items():assert sha(p)==h,p
def call(label,args):
    save(label+'_command.json',args)
    with (H/(label+'.log')).open('x') as f:
        job=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
        save(label+'_process.json',dict(pid=job.pid,started=datetime.datetime.now().astimezone().isoformat()))
        code=job.wait()
    save(label+'_receipt.json',dict(exit=code))
    assert code==0,(label,code)

save('start.json',dict(pid=os.getpid(),time=datetime.datetime.now().astimezone().isoformat()))
pins=json.loads((W/'runs/eig2_glim_first_edge_cpv54_20261030/prepared/protected_manifest.json').read_text())
pins.update(json.loads((H.parent/'eig2_frame_extrinsic_cpv55_20261030/retained_manifest.json').read_text()))
for mode in ('off','on'):
    for phase in ('after_ordinary','after_refresh'):
        p=Q/f'prepared/full_{mode}_source/checkpoints/update_6/{phase}/checkpoint.bin'
        pins[str(p)]=sha(p)
for root in (Q/'include',Q/'prepared/private_build'):
    for p in root.rglob('*'):
        if p.is_file():pins[str(p)]=sha(p)
for p in (Q/'Checkpoint.h',Q/'Binary.h',Q/'OriginalChartPrior.h',K/'VisualSupport.cpp',H/'PROTOCOL.txt',H/'run_test.py'):
    pins[str(p)]=sha(p)
for p in (W/'runs/eig2_first_factor_cpv52_20261030/attempt_compile1/results').glob('*factors.csv'):
    pins[str(p)]=sha(p)
for path in ('gtsam_unstable/slam/SmartStereoProjectionFactor.h','gtsam/slam/SmartFactorBase.h','gtsam/geometry/triangulation.h'):
    p=W/'devel/include'/path;pins[str(p)]=sha(p)
save('protected_manifest.json',pins);verify(pins)
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n',states
free=os.statvfs(W).f_bavail*os.statvfs(W).f_frsize;assert free>4*1024**3
save('preflight.json',dict(protected=len(pins),containers=states,free_bytes=free))
call('start_container',['docker','start','cbsms_ws_gpu'])
try:
    for mode in ('release','san'):
        compile=json.loads((H53/'release_compile_command.json').read_text())
        compile=[str(CK/'VisualSupport.cpp') if x.endswith('/Extract.cpp') else str(CK/(mode+'.o')) if x.endswith('/release.o') else x for x in compile]
        link=json.loads((H53/'release_link_command.json').read_text())
        link=[str(CK/(mode+'.o')) if x.endswith('/release.o') else str(CK/mode) if x.endswith('/release') else x for x in link if 'opencv' not in x]
        # No camera/image APIs; preserve the established GTSAM/CBS dependency set.
        if mode=='san':
            flags=['-O1','-g1','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer']
            compile=[x for x in compile if x!='-O3'];compile[1:1]=flags+['-fno-pie','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
            link=[x for x in link if x!='-O3'];link[1:1]=flags+['-no-pie']
        call(mode+'_compile',['docker','exec','cbsms_ws_gpu',*compile])
        call(mode+'_link',['docker','exec','cbsms_ws_gpu',*link])
        env=['-e','LD_LIBRARY_PATH='+str(C/'runs/kimera_on_incremental_refresh_cpv40_20261029/private_build')+':'+str(C/'devel/lib')]
        if mode=='san':env+=['-e','ASAN_OPTIONS=detect_leaks=0:halt_on_error=1','-e','UBSAN_OPTIONS=halt_on_error=1']
        call(mode+'_selftest',['docker','exec',*env,'cbsms_ws_gpu',str(CK/mode),'selftest'])
    call('audit',['docker','exec',*env[:2],'cbsms_ws_gpu',str(CK/'release'),str(CQ/'prepared'),str(CK/'results')])
    for mode in ('off','on'):
        for phase in ('ordinary','refresh'):
            original=Q/f'prepared/full_{mode}_source/checkpoints/update_6/after_{phase}/checkpoint.bin'
            assert sha(original)==sha(K/'results'/f'{mode}_{phase}_before.bin')==sha(K/'results'/f'{mode}_{phase}_after.bin')
    verify(pins)
    save('test_passed.json',dict(passed=True,protected=len(pins),snapshot_nonmutation=4,graph_optimizer_calls=0,time=datetime.datetime.now().astimezone().isoformat()))
except Exception:
    save('failure.json',dict(traceback=traceback.format_exc()))
    raise
finally:
    top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'],text=True)
    assert all(row.split()[1] in ('tail','zsh') for row in top.splitlines()[1:] if row.strip()),top
    call('restore_container',['docker','stop','cbsms_ws_gpu'])
    save('cleanup.json',dict(jobs=top,containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)))
