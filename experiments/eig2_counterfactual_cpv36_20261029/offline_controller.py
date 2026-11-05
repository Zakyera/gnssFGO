"""One isolated diagnostic per checkpoint, sequentially after source cleanup."""
import hashlib,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
Q='/workspace/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029'
assert json.loads((H/'capture_verification.json').read_text())['passed']
assert not (P/'offline').exists() and not (H/'offline_start.json').exists()
def call(args):return subprocess.check_output(args,text=True)
def write(n,d):
    with (H/n).open('x') as f:json.dump(d,f,indent=2)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
for p,s in json.loads((P/'prepared_manifest.json').read_text()).items():assert sha(P/p)==s,p
before=call(['docker','inspect','--format','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert 'true' not in before
for gate in ('offline_4','offline_4san'):assert (P/('build_'+gate)/'passed.json').exists()
write('offline_start.json',dict(pid=os.getpid(),wall_ns=time.time_ns(),states=before,binary_sha256=sha(P/'build_offline_4/counter')))
results=[]
try:
    call(['docker','start','cbsms_ws_gpu'])
    for k in (800,934,1000,1335,1336,1337,1338):
        cmd=['docker','exec','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e',
             'LD_LIBRARY_PATH='+Q+'/private_build:/workspace/cbs_gtsam4.3/devel/lib:/workspace/cbs_gtsam4.3/devel:/workspace/cbs_gtsam4.3/devel/lib:/usr/local/cuda/targets/x86_64-linux/lib:/usr/local/lib:/usr/local/nvidia/lib:/usr/local/nvidia/lib64',
             'cbsms_ws_gpu',Q+'/build_offline_4/counter',Q+'/capture/snapshots/update_'+str(k)+'/checkpoint.bin',Q+'/offline/update_'+str(k)]
        t=time.monotonic()
        with (H/('offline_'+str(k)+'.log')).open('x') as f:
            job=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT)
            write('offline_process_'+str(k)+'.json',dict(pid=job.pid,argv=cmd))
            # The solver has per-start180s/30-iteration limits. A host timeout
            # does not authorize killing unrelated container jobs or retrying.
            rc=job.wait(timeout=1200)
        result=dict(update=k,returncode=rc,wall_seconds=time.monotonic()-t)
        results.append(result);write('offline_receipt_'+str(k)+'.json',result)
        print(json.dumps(result),flush=True)
    write('offline_finished.json',dict(results=results,all_gates_passed=all(r['returncode']==0 for r in results)))
finally:
    top=call(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'])
    assert not any('/counter ' in l for l in top.splitlines()[1:]),'owned solver remains active; do not stop container blindly'
    call(['docker','stop','-t','20','cbsms_ws_gpu'])
    states=call(['docker','inspect','--format','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
    write('offline_restoration.json',dict(states=states,top_before_stop=top))
