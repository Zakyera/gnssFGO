"""Read-only inspection of saved equations; no optimizer/source repeat."""
import json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
Q='/workspace/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029'
D=P/'inspection/build';D.mkdir(exist_ok=False)
commands=json.loads((P/'actual_test_commands.json').read_text())
for name,args in commands.items():
    commands[name]=[a.replace(Q+'/observer_test.cpp',Q+'/inspection/Inspect.cpp').replace(Q+'/test.o',Q+'/inspection/build/inspect.o') if a!=Q+'/test' else Q+'/inspection/build/inspect' for a in args]
with (D/'commands.json').open('x') as f:json.dump(commands,f,indent=2)
before=subprocess.check_output(['docker','inspect','--format','{{.State.Running}}','cbsms_ws_gpu'],text=True)
assert before.strip()=='false'
started=dict(pid=os.getpid(),wall_ns=time.time_ns())
with (H/'inspection_start.json').open('x') as f:json.dump(started,f)
try:
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
    for name,args in commands.items():
        with (D/(name+'.log')).open('x') as f:subprocess.run(['docker','exec','cbsms_ws_gpu']+args,stdout=f,stderr=subprocess.STDOUT,check=True)
    for k in (800,934,1000,1335,1336,1337,1338):
        subprocess.run(['docker','exec','-e','LD_LIBRARY_PATH='+Q+'/private_build:/workspace/cbs_gtsam4.3/devel/lib:/usr/local/lib','cbsms_ws_gpu',Q+'/inspection/build/inspect',Q+'/capture/snapshots/update_'+str(k)+'/checkpoint.bin',Q+'/inspection/update_'+str(k)],check=True)
    with (H/'inspection_finished.json').open('x') as f:json.dump(dict(passed=True,read_only=True),f)
finally:
    top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm'],text=True)
    assert all(l.split()[-1]=='tail' for l in top.splitlines()[1:]),top
    subprocess.run(['docker','stop','-t','20','cbsms_ws_gpu'],check=True)
    states=subprocess.check_output(['docker','inspect','--format','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
    with (H/'inspection_restoration.json').open('x') as f:json.dump(dict(states=states,top=top),f)
