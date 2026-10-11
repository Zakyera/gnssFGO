from pathlib import Path
import json,os,subprocess,time
root=Path('/audit')
assert not (root/'command.json').exists()
prior=json.loads(Path('/prior/command_build.json').read_text())['commands'][0]['command']
command=[x.replace('/audit/build/DiagnosticReplay.cpp','/audit/build/check.cpp').replace('/audit/build/replay','/audit/build/check') for x in prior]
command.insert(command.index('-I/runtime/include'),'-I/prior/build')
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',LD_LIBRARY_PATH='/runtime/lib')
results=[]
for name,cmd in [('build',command),('check',['/audit/build/check','/audit','/prior/cached'])]:
    start=time.monotonic()
    with (root/'logs'/f'{name}.log').open('x') as log:p=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
    results.append(dict(command=cmd,returncode=p.returncode,seconds=time.monotonic()-start))
    if p.returncode:break
with (root/'command.json').open('x') as f:json.dump(results,f,indent=2)
print(json.dumps([{k:v for k,v in x.items() if k!='command'} for x in results]))
raise SystemExit(p.returncode)
